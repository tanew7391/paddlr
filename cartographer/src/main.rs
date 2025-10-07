#[macro_use]
extern crate rocket;
mod face_node;

use dotenv::dotenv;
use geo_buffer::buffer_polygon;
use geo_postgis::FromPostgis;
use pathfinding::prelude::astar;
use postgis::ewkb;
use proj::{Proj, Transform};
use spade::PositionInTriangulation::OnFace;
use std::{
    env,
    fs::File,
    io::{Read, Write},
    process::{Command, Stdio},
};
use tokio_postgres::{Client, NoTls};

use geo::{coord, BooleanOps, BoundingRect, Coord, HasDimensions, Intersects, Line, MultiPoint};
use spade::{
    handles::{FixedFaceHandle, InnerTag},
    ConstrainedDelaunayTriangulation, Point2, Triangulation,
};
use std::{collections::HashMap, error::Error, vec};

use env_logger;
use log::{self, debug, info, warn};

use geojson::de::deserialize_geometry;
use geojson::Value;
use rocket::{
    serde::{json::Json, Deserialize},
    tokio,
};

pub use crate::face_node::FaceNode;
use crate::face_node::FaceNodeType;

const OVERPASS_API_URL: &str = "https://overpass-api.de/api/interpreter";
const PSEUDO_MERCATOR_CRS: &str = "+proj=merc +a=6378137 +b=6378137 +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0 +k=1 +units=m +nadgrids=@null +wktext +no_defs +type=crs";
const WGS_84_CRS: &str = "+proj=longlat +datum=WGS84 +no_defs +type=crs";

struct ConnectionParams {
    host: String,
    database: String,
    user: String,
    password: String,
    port: String,
}

//TODO: have this take a rect
async fn get_associated_water_features(
    bounding_rect: &geo_types::Rect,
) -> Result<String, Box<dyn Error>> {
    let client = reqwest::Client::new();

    let bounding_rect_as_string = format!(
        "{}, {}, {}, {}",
        bounding_rect.min().y,
        bounding_rect.min().x,
        bounding_rect.max().y,
        bounding_rect.max().x,
    );

    let query = format!(
        r#"
        [out:xml];
        (
        way["natural"="water"]({:})->.ways;
        relation["natural"="water"]({:})->.relations;
        way(r.relations)->.relationways;
        node(w.ways);
        node(w.relationways);
        );
        out meta;
        "#,
        bounding_rect_as_string, bounding_rect_as_string
    );

    info!("Overpass API query: {:?}", query);

    let res: String = client
        .post(OVERPASS_API_URL)
        .header("Content-Type", "application/x-www-form-urlencoded")
        .body(format!("data={:}", query))
        .send()
        .await?
        .text()
        .await?;

    if env::var("DEBUG").is_ok() && env::var("DEBUG").unwrap() == "true" {
        let mut file = File::create("testing/osm_response.xml").unwrap();
        file.write_all(res.as_bytes()).unwrap();
    }

    if res.len() < 1000 {
        info!("Overpass API returned  {:?}", res);
    }

    Ok(res)
}

async fn get_connected_client() -> Result<tokio_postgres::Client, Box<dyn Error>> {
    let ConnectionParams {
        host,
        database,
        user,
        password,
        port,
    } = get_connection_params();

    let connection_attempt = tokio_postgres::connect(
        &format!(
            "host={} port={} user={} password={} dbname={}",
            host, port, user, password, database
        ),
        NoTls,
    )
    .await;

    let (client, connection) = match connection_attempt {
        Ok(client_connection) => client_connection,
        Err(e) => {
            warn!("Error connecting to database: {:}", e);
            return Err(e.into());
        }
    };

    // The connection object performs the actual communication with the database,
    // so spawn it off to run on its own.
    tokio::spawn(async move {
        if let Err(e) = connection.await {
            warn!("Connection error: {:}", e);
        }
    });

    Ok(client)
}

async fn get_focus_polygon_from_db(client: &Client) -> Result<geo_types::Polygon, Box<dyn Error>> {
    let rows = client.query("SELECT polygon FROM focus_polygon", &[]).await;

    let rows = match rows {
        Ok(rows) => rows,
        Err(e) => {
            warn!("Error retrieving rows: {:}", e);
            return Err(e.into());
        }
    };

    let polygon: ewkb::Polygon = rows[0].get("polygon");
    let focus_polygon = Option::from_postgis(&polygon);

    let focus_polygon = match focus_polygon {
        Some(polygon) => polygon,
        None => {
            warn!("Could not find focus polygon! Rows[0]: {:?}", rows[0]);
            return Err("No focus polygon found for study area!".into());
        }
    };

    Ok(focus_polygon)
}

async fn retrieve_associated_faces_from_db(
    client: &Client,
    mut points: MultiPoint<f64>,
) -> Result<Vec<geo_types::Polygon<f64>>, Box<dyn Error>> {
    points
        .transform_crs_to_crs(WGS_84_CRS, PSEUDO_MERCATOR_CRS)
        .expect("Failed to transform user multipoint geometry to Pseudo-Mercator CRS");

    let mut values = Vec::new();
    let mut params = Vec::new();
    const COLUMN_NAME: &str = "unioned_geom";

    for (i, p) in points.0.iter().enumerate() {
        let x_param = format!("${}", i * 2 + 1);
        let y_param = format!("${}", i * 2 + 2);
        values.push(format!(
            "({}::double precision, {}::double precision)",
            x_param, y_param
        ));
        params.push(p.x());
        params.push(p.y());
    }

    let query = format!(
        r#"
    WITH input_points AS (
    SELECT ST_MakePoint(x, y) AS geom
    FROM (VALUES {values}) AS v(x, y)
    ),
    envelope AS (
    SELECT ST_Extent(geom)::geometry AS geom
    FROM input_points
    )
    SELECT ST_UnaryUnion(way) AS {COLUMN_NAME}
    FROM planet_osm_polygon, envelope
    WHERE "natural" = 'water'
    AND way && envelope.geom;
    "#,
        values = values.join(", "),
    );

    let param_refs: Vec<&(dyn tokio_postgres::types::ToSql + Sync)> =
        params.iter().map(|v| v as _).collect();

    let rows = client.query(&query, &param_refs).await;

    let rows = match rows {
        Ok(rows) => rows,
        Err(e) => {
            warn!("Error retrieving rows: {:}", e);
            return Err(e.into());
        }
    };

    let mut faces: Vec<geo_types::Polygon<f64>> = Vec::new();
    for row in &rows {
        let polygon: ewkb::Polygon = row.get(COLUMN_NAME);
        let face = Option::from_postgis(&polygon);
        if let Some(face) = face {
            faces.push(face);
        } else {
            warn!("Face is None for row: {:?}", row);
        }
    }
    Ok(faces)
}

async fn retrieve_exterior_and_interior_rings_of_focus_polygon_from_db(
    client: &Client,
) -> Result<(geo_types::Polygon, Vec<geo_types::Polygon<f64>>), Box<dyn Error>> {
    let rows = client
        .query("SELECT geom FROM holes WHERE path[1] = 0", &[])
        .await;

    let rows = match rows {
        Ok(rows) => rows,
        Err(e) => {
            warn!("Error retrieving rows: {:}", e);
            return Err(e.into());
        }
    };

    //TODO: strange error coming from here
    let polygon: ewkb::Polygon = rows[0].get("geom");
    let exterior_ring = Option::from_postgis(&polygon);

    let exterior_ring = match exterior_ring {
        Some(ring) => ring,
        None => {
            warn!("Could not find exterior ring! Rows[0]: {:?}", rows[0]);
            return Err("No exterior ring found for study area!".into());
        }
    };

    //TODO: Condense this into a single query
    let rows = client
        .query("SELECT geom FROM holes WHERE path[1] > 1", &[])
        .await;

    let rows = match rows {
        Ok(rows) => rows,
        Err(e) => {
            warn!("Error retrieving rows: {:}", e);
            return Err(e.into());
        }
    };

    let mut interior_rings: Vec<geo_types::Polygon<f64>> = Vec::new();
    for row in &rows {
        let polygon: ewkb::Polygon = row.get("geom");
        let interior_ring = Option::from_postgis(&polygon);
        if let Some(interior_ring) = interior_ring {
            interior_rings.push(interior_ring);
        } else {
            warn!("Interior ring is None for row: {:?}", row);
        }
    }
    Ok((exterior_ring, interior_rings))
}

// Simplifies the search area to the intersection of the focus polygon and the bounding rectangle defined by start and end coordinates.
// Only returns a polygon if both start and end points are contained within the intersection polygon.
// Otherwise we would not be able to find a path between the two points.
fn simplify_search_area(
    start: geo::Coord<f64>,
    end: geo::Coord<f64>,
    focus_polygon: &geo::Polygon,
) -> Option<geo::Polygon<f64>> {
    let bounding_rect: geo::Polygon = geo::Rect::new(start, end).into();
    //Buffered because floating point preceision issues can cause problems when checking if the start and end points are contained within the intersection.
    let buffered_bounding_rect = buffer_polygon(&bounding_rect, 0.0001);

    let intersection = focus_polygon.intersection(&buffered_bounding_rect);
    if intersection.is_empty() {
        warn!("No intersection found between focus polygon and bounding rectangle.");
        return None;
    }

    for polygon in intersection {
        if polygon.intersects(&start) && polygon.intersects(&end) {
            // If the start and end points are contained within the polygon, return it
            return Some(polygon);
        }
    }

    info!("Start or end point is not contained within the intersection polygon.");

    return None;
}

fn get_connection_params() -> ConnectionParams {
    ConnectionParams {
        host: env::var("PGHOST").unwrap_or("localhost".to_string()),
        database: env::var("PGDATABASE").unwrap_or("database".to_string()),
        user: env::var("PGUSER").unwrap_or("user".to_string()),
        password: env::var("PGPASSWORD").unwrap_or("NO PASS SET".to_string()),
        port: env::var("PGPORT").unwrap_or("5432".to_string()),
    }
}

fn write_osm_to_database(osm_data: String) -> Result<(), Box<dyn Error>> {
    let ConnectionParams {
        host,
        database,
        user,
        password,
        port,
    } = get_connection_params();

    let connection_string = format!(
        "postgresql://{}:{}@{}:{}/{}",
        user, password, host, port, database
    );

    info!("Connection string: {}", connection_string);

    let mut command = Command::new("osm2pgsql")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .arg("-d")
        .arg(connection_string)
        .arg("--append")
        .arg("--slim")
        .arg("-r")
        .arg("xml")
        .arg("/dev/stdin")
        .spawn()?;

    let mut stdin = command.stdin.take().expect("Failed to open stdin");
    std::thread::spawn(move || {
        stdin
            .write_all(osm_data.as_bytes())
            .expect("Failed to write to stdin");
    });

    let output = command.wait_with_output()?;
    info!("Command output: {:?}", output);
    Ok(())
}

#[derive(Deserialize, Debug, PartialEq)]
enum StopFlag {
    FocusPolygon,
    Triangulation,
    TriangulationPathfinding,
    FullPath,
    ProcessedFaces,
}

#[derive(Deserialize, Debug)]
struct RequestData {
    stop_flag: StopFlag,
    #[serde(deserialize_with = "deserialize_geometry")]
    geometry: geo_types::MultiPoint<f64>,
}

#[post("/api", data = "<data>", format = "application/json")]
async fn index(data: Json<RequestData>) -> Result<String, rocket::http::Status> {
    let data = data.into_inner();

    debug!("Received data: {:?}", data);

    let contained_multipoint_geometry = data.geometry;

    let bounding_rect = contained_multipoint_geometry
        .bounding_rect()
        .expect("Failed to get bounding rectangle from multipoint geometry");

    let associated_water_features_future = get_associated_water_features(&bounding_rect);
    //line_string_json.unwrap().to_string()
    let associated_water_features = match associated_water_features_future.await {
        Ok(features) => features,
        Err(_e) => {
            return Err(rocket::http::Status::NotFound);
        }
    };

    write_osm_to_database(associated_water_features).expect("Failed to write OSM data to database");

    let client = get_connected_client()
        .await
        .expect("Failed to connect to database");

    //Temp
    let mut start = contained_multipoint_geometry
        .0
        .first()
        .expect("Expected at least one coordinate in multipoint geometry set. Received 0")
        .0;
    let mut end = contained_multipoint_geometry
        .0
        .last()
        .expect("Expected at least one coordinate in multipoint geometry set. Received 0")
        .0;

    //TODO: use rust to merge instead of SQL
    let faces = retrieve_associated_faces_from_db(&client, contained_multipoint_geometry)
        .await
        .expect("Failed to retrieve associated faces from database");

    //TODO: extract holes with rust
    let (exterior_ring, interior_rings) =
        retrieve_exterior_and_interior_rings_of_focus_polygon_from_db(&client)
            .await
            .expect("Failed to retrieve exterior and interior rings");

    let mut focus_polygon = get_focus_polygon_from_db(&client)
        .await
        .expect("Failed to retrieve focus polygon");

    //Attention: This needs to be below ANY .awaits or the function will not work because Proj does not support async
    let transform_to_wgs84 = Proj::new_known_crs(PSEUDO_MERCATOR_CRS, WGS_84_CRS, None)
        .expect("Failed to create transformer");

    let transform_to_pseudo_mercator = Proj::new_known_crs(WGS_84_CRS, PSEUDO_MERCATOR_CRS, None)
        .expect("Failed to create transformer");

    if data.stop_flag == StopFlag::FocusPolygon {
        info!("Returning focus polygon as GeoJSON");
        focus_polygon
            .transform(&transform_to_wgs84)
            .expect("Failed to reproject focus polygon");
        let focus_polygon_geojson = Value::from(&focus_polygon).to_string();
        return Ok(focus_polygon_geojson);
    }

    start.transform(&transform_to_pseudo_mercator).unwrap();
    end.transform(&transform_to_pseudo_mercator).unwrap();

    //TODO combine with interior rings
    //TODO check when we should actually simplify the search area (what number of points is too few?)
    if focus_polygon.exterior().points().len() > 1000 {
        let new_search_area = simplify_search_area(start, end, &focus_polygon);
        if let Some(new_polygon) = new_search_area {
            // Use the new polygon for pathfinding
            focus_polygon = new_polygon;
        } else {
            info!("No valid search area found, using original focus polygon.");
        }
    }

    let mut cdt = ConstrainedDelaunayTriangulation::<Point2<_>>::new();

    exterior_ring
        .exterior()
        .points()
        .map(|p| Point2::new(p.x(), p.y()))
        .for_each(|p| {
            cdt.insert(p)
                .expect(&format!("Unable to insert a vertex? {:?}", p));
        });

    //let mut current_interior_index = vertices.len();
    let edges: Vec<[usize; 2]> = Vec::new();

    for interior_ring in &interior_rings {
        let points: Vec<Point2<f64>> = interior_ring
            .exterior()
            .points()
            .map(|p| Point2::new(p.x(), p.y()))
            .collect();

        for i in 0..points.len() {
            let new_edge: [Point2<f64>; 2] = [points[i], points[(i + 1) % points.len()]];
            cdt.add_constraint_edge(new_edge[0], new_edge[1])
                .expect(&format!("Unable to add constraint edge: {:?}", new_edge));
        }
    }

    info!("Last interior ring edge: {:?}", edges.last());

    info!(
        "Number of Undirected Edges: {:}",
        cdt.num_undirected_edges()
    );
    info!("Number of Directed Edges: {:}", cdt.num_directed_edges());
    info!("Number of Vertices: {:}", cdt.num_vertices());
    info!("Number of Faces: {:}", cdt.num_all_faces());
    info!("Number of Inner Faces: {:}", cdt.num_inner_faces());

    let start_cdt = Point2::new(start.x, start.y);
    let end_cdt = Point2::new(end.x, end.y);

    info!(
        "Focus Polygon Verticies: {:?}",
        focus_polygon.exterior().points().len()
    );

    //End Temp

    if data.stop_flag == StopFlag::Triangulation {
        info!("Returning as GeoJSON");
        let triangulation: geo_types::MultiPolygon<f64> = cdt
            .inner_faces()
            .map(|face_handle| {
                let face_node = FaceNode::new(face_handle);
                let mut geom = face_to_polygon(&face_node);
                geom.transform(&transform_to_wgs84)
                    .expect("Failed to reproject face polygon");
                geom
            })
            .collect();
        let focus_polygon_geojson = Value::from(&triangulation).to_string();
        return Ok(focus_polygon_geojson);
    }

    let polygon_path = compute_a_star_pathfinding_from_delaney(
        start_cdt,
        end_cdt,
        &cdt,
        &focus_polygon,
        data.stop_flag == StopFlag::ProcessedFaces,
    );

    if data.stop_flag == StopFlag::ProcessedFaces {
        info!("Returning processed faces as GeoJSON");
        let multipolygon_path: geo_types::MultiPolygon<f64> = polygon_path
            .iter()
            .map(|face_node| {
                let mut geom = face_to_polygon(face_node);
                geom.transform(&transform_to_wgs84)
                    .expect("Failed to reproject face polygon");
                geom
            })
            .collect();
        let processed_faces_geojson = Value::from(&multipolygon_path).to_string();
        return Ok(processed_faces_geojson);
    }

    if polygon_path.is_empty() {
        info!("No path found, returning empty string");
        return Err(rocket::http::Status::NotFound);
    }

    if data.stop_flag == StopFlag::TriangulationPathfinding {
        info!("Returning triangulation path as GeoJSON");
        let multipolygon_path: geo_types::MultiPolygon<f64> = polygon_path
            .iter()
            .map(|face_node| {
                let mut geom = face_to_polygon(face_node);
                geom.transform(&transform_to_wgs84)
                    .expect("Failed to reproject face polygon");
                geom
            })
            .collect();
        let triangulation_geojson = Value::from(&multipolygon_path).to_string();
        return Ok(triangulation_geojson);
    }

    let portals = generate_portals(&polygon_path, &start_cdt, &end_cdt);

    let mut portals_geo_collection = geo_types::GeometryCollection::from(
        portals
            .iter()
            .map(|p| geo::Point::from(coord! {x: p.x, y: p.y }))
            .collect::<Vec<geo::Point<f64>>>(),
    );

    portals_geo_collection
        .transform(&transform_to_wgs84)
        .expect("Failed to reproject geometry collection");

    let portals_geo_json = Value::from(&portals_geo_collection).to_string();

    let mut file = File::create("testing/portals.geojson").unwrap();
    file.write_all(portals_geo_json.as_bytes()).unwrap();

    /*     let portals_transformed_TEST = portals.iter()
    .map(|p|
        {
            let mut pt = geo::Point::from(coord! {x: p.x, y: p.y });
            pt.transform(&transform_to_wgs84).unwrap();
            Point2::new(pt.x(), pt.y())
        }
    )
    .collect::<Vec<Point2<f64>>>(); */

    let path = string_pull(&portals, portals.len());

    let path_as_geom: Vec<geo::Coord<f64>> = path
        .iter()
        .map(|p| {
            coord! {x: p.x, y: p.y }
        })
        .collect();

    let mut test: Vec<Line> = vec![];

    path_as_geom.iter().enumerate().for_each(|(i, val)| {
        if i < path_as_geom.len() - 1 {
            test.push(Line::new(*val, path_as_geom[i + 1]))
        }
    });

    let mut line = geo_types::GeometryCollection::from(test);

    line.transform(&transform_to_wgs84)
        .expect("Failed to reproject geometry collection");
    let info_string = Value::from(&line).to_string();

    Ok(info_string)
}

fn face_to_polygon(face_node: &FaceNode) -> geo_types::Polygon<f64> {
    let face = face_node.get_face();
    let vertices = face.positions();
    let linestring = geo_types::LineString::from(
        vertices
            .into_iter()
            .map(|c| Coord { x: c.x, y: c.y })
            .collect::<Vec<Coord<f64>>>(),
    );
    geo_types::Polygon::new(linestring, vec![])
}
//End FaceNode

fn compute_a_star_pathfinding_from_delaney<'a>(
    start: Point2<f64>,
    end: Point2<f64>,
    cdt: &'a ConstrainedDelaunayTriangulation<Point2<f64>>,
    focus_polygon: &geo_types::Polygon<f64>,
    return_processed_faces: bool,
) -> Vec<FaceNode<'a>> {
    let mut processed_face_indices: HashMap<usize, FaceNodeType> = HashMap::new();
    let position_in_triangulation = cdt.locate(start);

    //Todo: test if a start or end point is on a vertex or an edge
    let start_face_handle: Option<FixedFaceHandle<InnerTag>>;
    if let OnFace(face_handle) = position_in_triangulation {
        info!("On face: {:?}", face_handle.index());
        start_face_handle = Some(face_handle);
    } else {
        info!("Not on face, {:?}", position_in_triangulation);
        panic!("Start point is not on a face, no way to handle this yet");
    }

    //Todo compress this into a single function
    let position_in_triangulation = cdt.locate(end);
    let end_face_handle: Option<FixedFaceHandle<InnerTag>>;
    if let OnFace(face_handle) = position_in_triangulation {
        info!("On face: {:?}", face_handle.index());
        end_face_handle = Some(face_handle);
    } else {
        info!("Not on face, {:?}", position_in_triangulation);
        panic!("Start point is not on a face, no way to handle this yet");
        //TODO handle by assigning the face to one of the adjacent faces to this point, it should be on a line if not a face
    }

    let start_face_center_vertex = cdt.face(start_face_handle.unwrap());
    let end_face_center_vertex = cdt.face(end_face_handle.unwrap());

    let start_face_node = FaceNode::new(start_face_center_vertex);
    let end_face_node = FaceNode::new(end_face_center_vertex);

    let heuristic_fn = |face_node: &FaceNode| face_node.distance(&end_face_node);

    let sucessors_fn = |face_node: &FaceNode<'a>| -> Vec<(FaceNode<'a>, u32)> {
        face_node.successors(focus_polygon, &mut processed_face_indices)
    };

    let success_fn = |face_node: &FaceNode| face_node.eq(&end_face_node);

    let a_star_results = astar(&start_face_node, sucessors_fn, heuristic_fn, success_fn);

    if return_processed_faces {
        let mut processed_faces: Vec<FaceNode> = vec![];
        for face in cdt.inner_faces() {
            if processed_face_indices.contains_key(&face.index()) {
                let face_node = FaceNode::new(face);
                processed_faces.push(face_node);
            }
        }
        return processed_faces;
    }

    let a_star_results = match a_star_results {
        Some((path, _cost)) => path,
        None => {
            info!("No path found");
            return vec![];
        }
    };

    a_star_results
}

fn triangle_area_2(a: &Point2<f64>, b: &Point2<f64>, c: &Point2<f64>) -> f32 {
    let ax = b.x as f32 - a.x as f32;
    let ay = b.y as f32 - a.y as f32;
    let bx = c.x as f32 - a.x as f32;
    let by = c.y as f32 - a.y as f32;
    let cross_product = bx * ay - ax * by;
    cross_product as f32
}

fn generate_portals(
    face_nodes: &Vec<FaceNode>,
    start_node: &Point2<f64>,
    end_node: &Point2<f64>,
) -> Vec<Point2<f64>> {
    let mut portals: Vec<Point2<f64>> = Vec::new();
    portals.push(start_node.clone());
    portals.push(start_node.clone());
    for i in 0..face_nodes.len() {
        let current = &face_nodes[i];
        let next = face_nodes.get(i + 1);
        let next = match next {
            Some(next) => next,
            None => {
                continue;
            }
        };
        let current_face = current.get_face();
        let next_face = next.get_face();
        for current_edge in current_face.adjacent_edges() {
            for next_face_edge in next_face.adjacent_edges() {
                if current_edge.rev() == next_face_edge {
                    //This order matters. TODO: see if producing correct portals
                    portals.push(current_edge.to().position());
                    portals.push(current_edge.from().position());
                }
            }
        }
    }

    portals.push(end_node.clone());
    portals.push(end_node.clone());

    portals
}

fn vequal(a: &Point2<f64>, b: &Point2<f64>) -> bool {
    let tolerance = 0.001 * 0.001;

    let equal = ((b.x as f32 - a.x as f32) * (b.x as f32 - a.x as f32)
        + (b.y as f32 - a.y as f32) * (b.y as f32 - a.y as f32))
        < tolerance;
    equal
}

fn string_pull(portals: &Vec<Point2<f64>>, max_points: usize) -> Vec<Point2<f64>> {
    let mut points: Vec<Point2<f64>> = vec![];
    let mut portal_apex: Point2<f64> = portals[0];
    let mut portal_left: Point2<f64> = portals[0];
    let mut portal_right: Point2<f64> = portals[1];
    let mut apex_index: usize;
    let mut left_index = 0;
    let mut right_index = 0;

    points.push(portal_apex);

    let mut i = 1;
    while (i < portals.len()) && (points.len() < max_points) {
        i = i + 1;
        let left = portals.get(i * 2);
        let right = portals.get((i * 2) + 1);

        let left = match left {
            Some(point) => point,
            None => {
                break;
            }
        };

        let right = match right {
            Some(point) => point,
            None => {
                break;
            }
        };

        if triangle_area_2(&portal_apex, &portal_right, right) <= 0.0 {
            if vequal(&portal_apex, &portal_right)
                || triangle_area_2(&portal_apex, &portal_left, right) > 0.0
            {
                //Check if the left and right points are inside the current funnel (described by the blue and red lines), if they are, we simple narrow the funnel (A-D).
                //Tighten funnel
                portal_right = *right;
                right_index = i;
            } else {
                // Right over left, insert left to path and restart scan from portal left point.
                points.push(portal_left);

                portal_apex = portal_left;
                apex_index = left_index;

                portal_left = portal_apex;
                portal_right = portal_apex;
                left_index = apex_index;
                right_index = apex_index;

                i = apex_index;
                continue;
            }
        }

        if triangle_area_2(&portal_apex, &portal_left, left) >= 0.0 {
            if vequal(&portal_apex, &portal_left)
                || triangle_area_2(&portal_apex, &portal_right, left) < 0.0
            {
                //Tighten funnel
                portal_left = *left;
                left_index = i;
            } else {
                points.push(portal_right);

                portal_apex = portal_right;
                apex_index = right_index;

                portal_left = portal_apex;
                portal_right = portal_apex;
                left_index = apex_index;
                right_index = apex_index;

                i = apex_index;
                continue;
            }
        }
    }

    if points.len() < max_points {
        points.push(portals.last().unwrap().clone());
    }

    info!("String Pull Points len: {:}", points.len());

    points
}

//TODO: test
fn cdt_face_to_polygon(face_positions: &[Point2<f64>; 3]) -> geo_types::Polygon<f64> {
    let linestring = geo_types::LineString::from(
        face_positions
            .into_iter()
            .map(|c| Coord { x: c.x, y: c.y })
            .collect::<Vec<Coord<f64>>>(),
    );
    geo_types::Polygon::new(linestring, vec![])
}

#[launch]
fn rocket() -> _ {
    dotenv().ok();
    env_logger::init();
    rocket::build().mount("/", routes![index])
}

/*  #[tokio::main]
async fn main() {
    test().await;
} */
