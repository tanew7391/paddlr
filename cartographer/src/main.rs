#[macro_use]
extern crate rocket;

use std::env;
use geo_postgis::FromPostgis;
use pathfinding::prelude::astar;
use postgis::ewkb;
use proj::{Proj, Transform};
use spade::PositionInTriangulation::OnFace;
use tokio_postgres::{Client, NoTls};
use dotenv::dotenv;

use geo::{Contains, Coord};
use spade::{
    handles::{FaceHandle, FixedFaceHandle, InnerTag},
    ConstrainedDelaunayTriangulation, Point2, Triangulation,
};
use std::{
    collections::HashMap,
    error::Error,
    vec,
};

use log::{self, debug, info, warn};

use geo_types::GeometryCollection;

use geojson::{Bbox, GeoJson, Geometry, PointType, Value};
use rocket::{
    data::{Data, ToByteUnit},
    tokio,
};

const OVERPASS_API_URL: &str = "https://overpass-api.de/api/interpreter";

async fn get_associated_water_features(
    bounding_area: Bbox,
) -> Result<GeometryCollection, Box<dyn Error>> {
    let client = reqwest::Client::new();

    let bounding_box_as_string = bounding_area
        .iter()
        .map(|n| n.to_string())
        .collect::<Vec<_>>()
        .join(", ");

    let query = format!(
        r#"
        [out:json];
        (
        way["natural"="water"]({:});
        relation["natural"="water"]({:});
        );
        out geom;
        "#,
        bounding_box_as_string, bounding_box_as_string
    );

    println!("{:}", query);

    let res: String = client
        .post(OVERPASS_API_URL)
        .header("Content-Type", "application/x-www-form-urlencoded")
        .body(format!("data={:}", query))
        .send()
        .await?
        .text()
        .await?;
    debug!("{:#?}", res);

    //let geometry_collection = convert_osmjson_to_geo(&res);

    //Ok(geometry_collection)

    //This is a temp value until I figure things out
    Ok(GeometryCollection::default())
}

fn extract_bounding_box_from_multipoint_geometry(
    multipoint_geom: &Vec<PointType>,
) -> Result<Bbox, Box<dyn Error>> {
    let failure_message = "Expected at least one coordinate in multipoint geometry set. Received 0";

    let mut owned_multipoint_geom = multipoint_geom.to_owned();

    owned_multipoint_geom.sort_by(|point_a, point_b| point_a[0].partial_cmp(&point_b[0]).unwrap());
    let westmost_value = owned_multipoint_geom.first().expect(failure_message)[0];
    let eastmost_value = owned_multipoint_geom.last().expect(failure_message)[0];

    owned_multipoint_geom.sort_by(|point_a, point_b| point_a[1].partial_cmp(&point_b[1]).unwrap());
    let southmost_value = owned_multipoint_geom.first().expect(failure_message)[1];
    let northmost_value = owned_multipoint_geom.last().expect(failure_message)[1];

    Ok(Bbox::from([
        southmost_value,
        westmost_value,
        northmost_value,
        eastmost_value,
    ]))
}

fn extract_multipoint_geometry(geojson: GeoJson) -> Result<Vec<PointType>, Box<dyn Error>> {
    if let GeoJson::Feature(feature) = geojson {
        if let Some(Geometry {
            value: Value::MultiPoint(multipoints),
            ..
        }) = feature.geometry
        {
            return Ok(multipoints);
        }
    }

    Err("GeoJSON does not include a multipoint geometry".into())
}

//Deprecated
/*
fn multipoint_to_linestring(multipoints: Vec<PointType>) -> GeoJson {
    // Convert MultiPoint coordinates to LineString
    let linestring_coords = multipoints;

    // Create the LineString GeoJSON geometry
    let linestring_geometry = Geometry::new(Value::LineString(linestring_coords));

    // Create a new GeoJSON feature with the LineString geometry
    let linestring_feature = geojson::Feature {
        geometry: Some(linestring_geometry),
        properties: linestring_feature.properties,
        ..Default::default()
    };

    // Wrap the feature in GeoJson and return it
    return GeoJson::Feature(linestring_feature);
} */

// Deprecated
async fn index_old(data: Data<'_>) -> String {
    //let data_string = data.open(1.megabytes()).into_string().await.unwrap();

    //let json = data_string.parse::<GeoJson>().unwrap();

    //let contained_multipoint_geometry: Vec<PointType> = extract_multipoint_geometry(json).unwrap();

    //let bounding_box: Bbox =
    //    extract_bounding_box_from_multipoint_geometry(&contained_multipoint_geometry).unwrap();

    //let line_string_json = multipoint_to_linestring(json);
    //let associated_water_features_future = get_associated_water_features(bounding_box);
    //line_string_json.unwrap().to_string()
    //let associated_water_features = associated_water_features_future.await.unwrap();
    //println!("Water features: {:}", associated_water_features.to_string());

    //gdal_geom::from_wkb(wkb)

    //let defined_point = Point::new(
    //    contained_multipoint_geometry[0][0],
    //    contained_multipoint_geometry[0][1],
    //);

    //println!("After conv: {:#?}", collection);

    //let possible_containing_polygon =
    //    find_containing_polygon(&defined_point, &associated_water_features);

    //let geometry_container_reference_holder = Vec::from_iter(associated_water_features.iter());

    //let geojson_of_associated_water_features = Value::from(&geo::Geometry::GeometryCollection(associated_water_features));
    //println!("{:#?}", geojson_of_associated_water_features);
    //return geojson_of_associated_water_features.to_string();

    //let containing_polygon = match possible_containing_polygon {
    //    Some(polygon) => polygon,
    //    None => panic!("No containing polygon!!!")
    //};

    // Connect to the database.
    let (client, connection) = tokio_postgres::connect(
        "host=localhost port=5432 user=postgres password=postgres dbname=gisdb",
        NoTls,
    )
    .await
    .unwrap();

    // The connection object performs the actual communication with the database,
    // so spawn it off to run on its own.
    tokio::spawn(async move {
        if let Err(e) = connection.await {
            eprintln!("connection error: {}", e);
        }
    });

    let mut polygons: Vec<geo_types::Geometry> = Vec::new();

    let rows = client
        .query(
            "SELECT ST_Multi(ST_Transform(polygon, 4326)) as polygon FROM focus_polygon",
            &[],
        )
        .await
        .unwrap();

    for row in &rows {
        let polygon: ewkb::MultiPolygon = row.get("polygon");
        let focus_polygon = geo_types::MultiPolygon::from_postgis(&polygon);
        polygons.push(geo_types::Geometry::from(focus_polygon));
    }

    //temporarily disabling
    Value::from(&geo::Geometry::GeometryCollection(
        GeometryCollection::new_from(polygons),
    ))
    .to_string();

    //test().await
    "placeholder".to_string()
}

async fn get_connected_client() -> Result<tokio_postgres::Client, Box<dyn Error>> {
    let connection_attempt = tokio_postgres::connect(
        &format!("host=localhost port=5432 user=postgres password={} dbname=gisdb", env::var("db_password").unwrap_or("NO PASS SET".to_string())),
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

#[post("/api", data = "<data>")]
async fn index(data: Data<'_>) -> String {
    let data_string = data.open(1.megabytes()).into_string().await.unwrap();

    let json = data_string.parse::<GeoJson>().unwrap();

    let contained_multipoint_geometry: Vec<PointType> = extract_multipoint_geometry(json).unwrap();

    let bounding_box: Bbox =
        extract_bounding_box_from_multipoint_geometry(&contained_multipoint_geometry).unwrap();

    //Eventually we can use this to plot a direct line between our points
    //let line_string_json = multipoint_to_linestring(contained_multipoint_geometry);

    let associated_water_features_future = get_associated_water_features(bounding_box);
    //line_string_json.unwrap().to_string()
    let associated_water_features = associated_water_features_future.await.unwrap();
    //println!("Water features: {:}", associated_water_features.to_string());

    let client = get_connected_client()
        .await
        .expect("Failed to connect to database");

    let (exterior_ring, interior_rings) =
        retrieve_exterior_and_interior_rings_of_focus_polygon_from_db(&client)
            .await
            .expect("Failed to retrieve exterior and interior rings");

    let focus_polygon = get_focus_polygon_from_db(&client)
        .await
        .expect("Failed to retrieve focus polygon");

    let transform_to_wgs84 = Proj::new_known_crs(
        "+proj=merc +a=6378137 +b=6378137 +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0 +k=1 +units=m +nadgrids=@null +wktext +no_defs +type=crs",
             "+proj=longlat +datum=WGS84 +no_defs +type=crs",
            None,
        ).expect("Failed to create transformer");

    let transform_to_pseudo_mercator = Proj::new_known_crs(
        "+proj=longlat +datum=WGS84 +no_defs +type=crs",
        "+proj=merc +a=6378137 +b=6378137 +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0 +k=1 +units=m +nadgrids=@null +wktext +no_defs +type=crs",
        None,
    ).expect("Failed to create transformer");

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
    let mut edges: Vec<[usize; 2]> = Vec::new();

    for interior_ring in &interior_rings {
        let points: Vec<Point2<f64>> = interior_ring
            .exterior()
            .points()
            .map(|p| Point2::new(p.x(), p.y()))
            .collect();

        for i in 0..points.len() {
            let new_edge: [Point2<f64>; 2] = [points[i], points[(i + 1) % points.len()]];
            //edges.push(new_edge);
            cdt.add_constraint_edge(new_edge[0], new_edge[1])
                .expect(&format!("Unable to add constraint edge: {:?}", new_edge));
        }
        //current_interior_index += points.len();

        //vertices.extend(points);
    }

    println!("Last interior ring edge: {:?}", edges.last());

    //Debug mode

    //let cdt: Result<ConstrainedDelaunayTriangulation<Point2<f64>>, spade::InsertionError> = ConstrainedDelaunayTriangulation::<_>::bulk_load_cdt(vertices, edges);

    println!(
        "Number of Undirected Edges: {:}",
        cdt.num_undirected_edges()
    );
    println!("Number of Directed Edges: {:}", cdt.num_directed_edges());
    println!("Number of Vertices: {:}", cdt.num_vertices());
    println!("Number of Faces: {:}", cdt.num_all_faces());
    println!("Number of Inner Faces: {:}", cdt.num_inner_faces());

    //Temp
    let mut start = Coord {
        x: contained_multipoint_geometry[0][0],
        y: contained_multipoint_geometry[0][1],
    };
    let mut end = Coord {
        x: contained_multipoint_geometry[1][0],
        y: contained_multipoint_geometry[1][1],
    };

    start.transform(&transform_to_pseudo_mercator).unwrap();
    end.transform(&transform_to_pseudo_mercator).unwrap();

    let start = Point2::new(start.x, start.y);
    let end = Point2::new(end.x, end.y);

    //End Temp

    let polygon_path = compute_A_star_pathfinding_from_delaney(start, end, &cdt, &focus_polygon);


    let test: Vec<geo_types::Geometry<f64>> = polygon_path
        .iter()
        .map(|face_node| geo_types::Geometry::from(face_to_polygon_TEST(face_node)))
        .collect();
    let mut test = geo_types::GeometryCollection::new_from(test);
    test.transform(&transform_to_wgs84).expect("Failed to reproject geometry collection");
    let debug_string = Value::from(&test).to_string();

    debug_string
}

//Start FaceNode
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct FaceNode<'a> {
    face: FaceHandle<
        'a,
        InnerTag,
        Point2<f64>,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::DirectedEdge,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::UndirectedEdge,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::Face,
    >,
}

#[derive(PartialEq, Eq)]
enum FaceNodeType {
    Inner,
    Outer,
}

impl<'a> FaceNode<'a> {
    fn successors(&self, focus_polygon: &geo_types::Polygon<f64>, checked_faces: &mut HashMap<usize, FaceNodeType>) -> Vec<(FaceNode<'a>, u32)> {
        let face_node = self.face;
        let mut successor_faces: Vec<(FaceNode<'a>, u32)> = vec![];
        for edge in face_node.adjacent_edges() {
            let adjacent_face = edge.rev().face().as_inner();
            let adjacent_face = match adjacent_face {
                Some(face) => face,
                None => {
                    // Handle the case where the adjacent face is outer (outer being outside of convex hull in CDT), we don't want to process it.
                    continue;
                }
            };

            let adjacent_center_point = adjacent_face.center();
            let adjacent_center_point = Coord {
                x: adjacent_center_point.x,
                y: adjacent_center_point.y,
            };

            let adjacent_face_index = adjacent_face.index();

            if !checked_faces.contains_key(&adjacent_face_index) {
                //We need only process the faces that are inside the focus polygon
                let face_node_type = match focus_polygon.contains(&adjacent_center_point) {
                    true => FaceNodeType::Inner,
                    false => FaceNodeType::Outer,
                };
                checked_faces.insert(adjacent_face_index, face_node_type);
            } 
            
            if checked_faces[&adjacent_face_index] == FaceNodeType::Outer {
                //We need only process the faces that are inside the focus polygon
                continue;
            }

            let adjacent_face_node: FaceNode<'a> = FaceNode {
                face: adjacent_face,
            };
            let distance_between_faces = self.distance(&adjacent_face_node);
            successor_faces.push((adjacent_face_node, distance_between_faces))
        }

        successor_faces
    }
}

impl FaceNode<'_> {
    //TODO: see if this is discarding SigFigs which could be important for pathfinding
    fn distance(&self, other: &FaceNode) -> u32 {
        self.face
            .center()
            .distance_2(other.face.center())
            .sqrt()
            .round() as u32
    }
}

fn face_to_polygon_TEST(face_node: &FaceNode) -> geo_types::Polygon<f64> {
    let face = face_node.face;
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

fn compute_A_star_pathfinding_from_delaney<'a>(
    start: Point2<f64>,
    end: Point2<f64>,
    cdt: &'a ConstrainedDelaunayTriangulation<Point2<f64>>,
    focus_polygon: &geo_types::Polygon<f64>,
) -> Vec<FaceNode<'a>> {
    //This should return a vector of faces from cdt
    // Implement A* algorithm here using the CDT
    // This is a placeholder implementation

    let mut processed_face_indices: HashMap<usize, FaceNodeType> = HashMap::new();
    //let start_transformed = start.transform(&transformer).unwrap();
    println!("Start transformed: {:?}", start);
    let position_in_triangulation = cdt.locate(start);

    //Todo: test if a start or end point is on a vertex or an edge
    let mut start_face_handle: Option<FixedFaceHandle<InnerTag>> = None;
    if let OnFace(face_handle) = position_in_triangulation {
        println!("On face: {:?}", face_handle.index());
        start_face_handle = Some(face_handle);
    } else {
        println!("Not on face, {:?}", position_in_triangulation);
        panic!("Start point is not on a face, no way to handle this yet");
    }

    //Todo compress this into a single function
    let position_in_triangulation = cdt.locate(end);
    let mut end_face_handle: Option<FixedFaceHandle<InnerTag>> = None;
    if let OnFace(face_handle) = position_in_triangulation {
        println!("On face: {:?}", face_handle.index());
        end_face_handle = Some(face_handle);
    } else {
        println!("Not on face, {:?}", position_in_triangulation);
        panic!("Start point is not on a face, no way to handle this yet");
    }

    let start_face_center_vertex = cdt.face(start_face_handle.unwrap());
    let end_face_center_vertex = cdt.face(end_face_handle.unwrap());

    let start_face_node = FaceNode {
        face: start_face_center_vertex,
    };

    let end_face_node = FaceNode {
        face: end_face_center_vertex,
    };

    let heuristic_fn = |face_node: &FaceNode| face_node.distance(&end_face_node);

    let sucessors_fn = |face_node: &FaceNode<'a>| -> Vec<(FaceNode<'a>, u32)> {
        face_node.successors(focus_polygon, &mut processed_face_indices)
    };

    //TODO: this eq might not be correct if it refers to the actual handle and not the face.
    let success_fn = |face_node: &FaceNode| face_node.face == end_face_node.face;

    let a_star_results = astar(&start_face_node, sucessors_fn, heuristic_fn, success_fn);
    let a_star_results = match a_star_results {
        Some((path, _cost)) => path,
        None => {
            println!("No path found");
            return vec![];
        }
    };

    a_star_results
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
    rocket::build().mount("/", routes![index])
}

/*  #[tokio::main]
async fn main() {
    test().await;
} */
