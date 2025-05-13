#[macro_use]
extern crate rocket;


use pathfinding::prelude::astar;
use geo_postgis::FromPostgis;
use postgis::ewkb;
use proj::{Proj, Transform};
use tokio_postgres::{Client, NoTls};
use spade::{PositionInTriangulation::{
    OnEdge, OnFace, OnVertex
}};

use geo::{
    Contains, Coord, Within
};
use spade::{handles::{self, FaceHandle, FixedFaceHandle, InnerTag}, ConstrainedDelaunayTriangulation, Point2, Triangulation};
use std::{
    collections::{HashMap, HashSet},
    error::Error,
    vec,
};

use log::{self, debug, info, warn};

use geo_types::{GeometryCollection};

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
        "host=localhost port=5432 user=postgres password=postgres dbname=gisdb",
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
    
    let rows = client
        .query(
            "SELECT polygon FROM focus_polygon",
            &[],
        )
        .await;

    let rows = match rows {
        Ok(rows) => rows,
        Err(e) => {
            warn!("Error retrieving rows: {:}", e);
            return Err(e.into());
        }
    };

    let mut focus_polygon: Option<geo_types::Polygon<f64>> = None;

    let polygon: ewkb::Polygon = rows[0].get("polygon");
    focus_polygon = Option::from_postgis(&polygon);

    let focus_polygon = match focus_polygon {
        Some(polygon) => polygon,
        None => {
            warn!("Could not find focus polygon! Rows[0]: {:?}", rows[0]);
            return Err("No focus polygon found for study area!".into());
        }
    };

    Ok(focus_polygon)
}

async fn retrieve_exterior_and_interior_rings_of_focus_polygon_from_db(client: &Client
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

    let mut exterior_ring: Option<geo_types::Polygon<f64>> = None;

    let polygon: ewkb::Polygon = rows[0].get("geom");
    exterior_ring = Option::from_postgis(&polygon);

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
    Ok((
        exterior_ring,
        interior_rings,
    ))
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

    let (exterior_ring, interior_rings) = retrieve_exterior_and_interior_rings_of_focus_polygon_from_db(&client)
        .await
        .expect("Failed to retrieve exterior and interior rings");

    let focus_polygon = get_focus_polygon_from_db(&client).await
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
    let mut geometry_collection_vector: Vec<geo_types::Geometry> = vec![];
    for inner_face in cdt.inner_faces() {
        let vertices = inner_face.positions();
        let mut linestring = geo_types::LineString::from(
            vertices
                .into_iter()
                .map(|c| Coord { x: c.x, y: c.y })
                .collect::<Vec<Coord<f64>>>(),
        );
        let polygon = geo_types::Polygon::new(linestring, vec![]);
        geometry_collection_vector.push(geo_types::Geometry::from(polygon));
    }

    //let cdt: Result<ConstrainedDelaunayTriangulation<Point2<f64>>, spade::InsertionError> = ConstrainedDelaunayTriangulation::<_>::bulk_load_cdt(vertices, edges);

    let mut geometry_collection =
        geo_types::GeometryCollection::new_from(geometry_collection_vector);
    geometry_collection
        .transform(&transform_to_wgs84)
        .expect("Failed to reproject geometry collection");

    let debug_string = Value::from(&geometry_collection).to_string();

    println!(
        "Number of Undirected Edges: {:}",
        cdt.num_undirected_edges()
    );
    println!("Number of Directed Edges: {:}", cdt.num_directed_edges());
    println!("Number of Vertices: {:}", cdt.num_vertices());
    println!("Number of Faces: {:}", cdt.num_all_faces());
    println!("Number of Inner Faces: {:}", cdt.num_inner_faces());


    //Temp
    let mut start = Coord{
        x: contained_multipoint_geometry[0][0],
        y: contained_multipoint_geometry[0][1],
    };
    let mut end = Coord{
        x: contained_multipoint_geometry[1][0],
        y: contained_multipoint_geometry[1][1],
    };
    
    start.transform(&transform_to_pseudo_mercator).unwrap();
    end.transform(&transform_to_pseudo_mercator).unwrap();

    let start = Point2::new(start.x, start.y);
    let end = Point2::new(end.x, end.y);

    //End Temp
    


    compute_A_star_pathfinding_from_delaney(
        start,
        end,
        &cdt,
        &focus_polygon
    );

    debug_string
}


//Start FaceNode
#[derive(Clone, Debug)]
struct FaceNode<'a>{
    face: FaceHandle<'a, 
        InnerTag, 
        Point2<f64>, 
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::DirectedEdge,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::UndirectedEdge,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::Face
    >,
    focus_polygon: &'a geo_types::Polygon<f64>
}

impl PartialEq for FaceNode<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.face.eq(&other.face)
    }
}

impl Eq for FaceNode<'_>{}

impl std::hash::Hash for FaceNode<'_>{
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        //TODO: test if this is the correct way to hash
        self.face.hash(state);
    }
}

impl Ord for FaceNode<'_>{
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.face.cmp(&other.face)
    }
}

impl PartialOrd for FaceNode<'_>{
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.face.cmp(&other.face))
    }
}

impl FaceNode<'_>{

    //TODO: see if this is discarding SigFigs which could be important for pathfinding
    fn distance(&self, other: &FaceNode) -> u32 {
        self.face.center().distance_2(other.face.center()).sqrt().round() as u32
    }

  
    fn successors(&self) -> Vec<(FaceNode, u32)> {

        let face_node = self.face;
        let mut successor_faces: Vec<(FaceNode<'_>, u32)> = vec![];
        for edge in face_node.adjacent_edges() {
            let adjacent_face = edge.rev().face().as_inner();
            let adjacent_face = match adjacent_face {
                Some(face) => face,
                None => {
                    // Handle the case where the face is outer, we don't want to process it.
                    continue;
                }
            };
            let adjacent_face_node = FaceNode{
                face: adjacent_face
            };
            let distance_between_faces = self.distance(&adjacent_face_node);
            successor_faces.push((   
                adjacent_face_node,
                distance_between_faces
            ))
        }

      successor_faces
    }
  }

//End FaceNode

fn compute_A_star_pathfinding_from_delaney(
    start: Point2<f64>,
    end: Point2<f64>,
    cdt: &ConstrainedDelaunayTriangulation<Point2<f64>>,
    focus_polygon: &geo_types::Polygon<f64>,
) -> Vec<Point2<f64>> { //This should return a vector of faces from cdt
    // Implement A* algorithm here using the CDT
    // This is a placeholder implementation
    

    let mut exterior_face_indices: HashSet<usize> = HashSet::new();
    let mut processed_face_indices: HashMap<usize, Point2<f64>> = HashMap::new();
    let mut start_face_index: Option<usize> = None;
    let mut end_face_index: Option<usize> = None;
    //let start_transformed = start.transform(&transformer).unwrap();
    println!("Start transformed: {:?}", start);
    let position_in_triangulation = cdt.locate(start);
    //Todo: test if a start or end point is on a vertex or an edge
    let start_face_handle: Option<FixedFaceHandle<InnerTag>> = None; 
    if let OnFace(face_handle) = position_in_triangulation {
        println!("On face: {:?}", face_handle.index());
        let start_face_handle = Some(face_handle);
    } else {
        println!("Not on face, {:?}", position_in_triangulation);
        panic!("Start point is not on a face, no way to handle this yet");
    }


    let start_face_center_vertex = cdt.face(start_face_handle.unwrap());


    let a_star_results = astar(start_face_center_vertex.center(), successors, heuristic, success)

    //Uncomment when using for A star
    // First, loop through the faces of the CDT and test for intersection with the focus polygon
    /* for face in cdt.inner_faces() {
        let center_point = face.center();
        let center_point = Coord {
            x: center_point.x,
            y: center_point.y,
        };

        let face_index = face.index();

        if exterior_face_indices.contains(&face_index) {
            // Skip if the face is already in the exterior face indices
            continue;
        }
        
        if !focus_polygon.contains(&center_point) {
            //We need only process the faces that are inside the focus polygon
            exterior_face_indices.insert(face_index);
            continue;
        }

        // Check if the face includes the start or end point
        let polygon_for_face = cdt_face_to_polygon(&face.positions());

        if (Coord{x: start.x, y: start.y}).is_within(&polygon_for_face){
            //Handle start point
        }

        if (Coord{x: end.x, y: end.y}).is_within(&polygon_for_face){
            //Handle end point
        }


        processed_face_indices.insert(face_index, face.center());
    } */
    
    let mut path = vec![start, end];
    path
}

//TODO: test
fn cdt_face_to_polygon(
    face_positions: &[Point2<f64>; 3],
) -> geo_types::Polygon<f64> {
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
    rocket::build().mount("/", routes![index])
}

/*  #[tokio::main]
async fn main() {
    test().await;
} */
