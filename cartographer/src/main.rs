#[macro_use]
extern crate rocket;

use geo_postgis::FromPostgis;
use postgis::ewkb;
use proj::{Proj, Transform};
use tokio_postgres::NoTls;

use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use geo::{BoundingRect, Contains, Coord, CoordNum, Geometry as GeoGeometry, PreparedGeometry, Relate};
use std::{
    collections::{HashMap, HashSet},
    error::Error,
    f64::consts::E,
    vec,
};

use log::{self, debug, info, warn};
use std::fs;

use geo_types::{GeometryCollection, Point};

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

fn multipoint_to_linestring(geojson: GeoJson) -> Result<GeoJson, Box<dyn Error>> {
    // Extract the MultiPoint geometry
    if let GeoJson::Feature(feature) = geojson {
        if let Some(Geometry {
            value: Value::MultiPoint(multipoints),
            ..
        }) = feature.geometry
        {
            // Convert MultiPoint coordinates to LineString
            let linestring_coords = multipoints;

            // Create the LineString GeoJSON geometry
            let linestring_geometry = Geometry::new(Value::LineString(linestring_coords));

            // Create a new GeoJSON feature with the LineString geometry
            let linestring_feature = geojson::Feature {
                geometry: Some(linestring_geometry),
                properties: feature.properties,
                ..Default::default()
            };

            // Wrap the feature in GeoJson and return it
            return Ok(GeoJson::Feature(linestring_feature));
        }
    }

    // Return an error if input is not a valid MultiPoint GeoJSON
    Err("Input is not a valid MultiPoint geometry".into())
}

#[post("/api", data = "<data>")]
async fn index(data: Data<'_>) -> String {
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

    test().await
}

#[launch]
fn rocket() -> _ {
    rocket::build().mount("/", routes![index])
}

async fn test() -> String {
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

    let rows = client
        .query("SELECT geom FROM holes WHERE path[1] = 0", &[])
        .await
        .unwrap();
    let mut exterior_ring: Option<geo_types::Polygon<f64>> = None;

    let polygon: ewkb::Polygon = rows[0].get("geom");
    exterior_ring = Option::from_postgis(&polygon);

    if exterior_ring.is_none() {
        warn!("Could not find exterior ring! Rows[0]: {:?}", rows[0]);
        panic!("No exterior ring found for study area!");
    }
    let exterior_ring = exterior_ring.unwrap();

    let rows = client
        .query("SELECT geom FROM holes WHERE path[1] > 1", &[])
        .await
        .unwrap();
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

    let transformer = Proj::new_known_crs(
        "+proj=merc +a=6378137 +b=6378137 +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0 +k=1 +units=m +nadgrids=@null +wktext +no_defs +type=crs",
             "+proj=longlat +datum=WGS84 +no_defs +type=crs",
            None,
        ).expect("Failed to create transformer");




    let mut vertices: Vec<Point2<f64>> = exterior_ring
        .exterior()
        .points()
        .map(|p| Point2::new(p.x(), p.y()))
        .collect();

    let mut current_interior_index = vertices.len();
    let mut edges: Vec<[usize; 2]> = Vec::new();

    for interior_ring in &interior_rings {
        let points: Vec<Point2<f64>> = interior_ring
            .exterior()
            .points()
            .map(|p| Point2::new(p.x(), p.y()))
            .collect();

        for i in 0..points.len() {
            let new_edge: [usize; 2] = [
                i,
                (i+1) % points.len()
            ];
            edges.push(new_edge);
        }
        current_interior_index += points.len();

        vertices.extend(points);
    }

    println!("Last interior ring edge: {:?}", edges.last());


    /* while !triangulation_is_complete {
        triangles = cdt::Triangulation::build_with_edges(&exterior_ring_pts, &edges);
        if let Err(cdt::Error::PointOnFixedEdge(index)) = triangles {


            let prev_point = exterior_ring_pts[index - 1];
            let point = exterior_ring_pts[index];
            let next_point = exterior_ring_pts[index + 1];
            let mut prev_point_transformed = geo_types::Point::from(prev_point);
            let mut next_point_transformed = geo_types::Point::from(next_point);
            let mut point_transformed = geo_types::Point::from(point);
            prev_point_transformed.transform(&transformer).unwrap();
            next_point_transformed.transform(&transformer).unwrap();
            point_transformed.transform(&transformer).unwrap();
            let line = geo_types::LineString::from(vec![
                prev_point_transformed,
                point_transformed,
                next_point_transformed,
            ]);

            println!("Point is on fixed edge: {}\n Here is the line it belongs to: {:?}", index, 
                Value::from(&line).to_string()
            );
            // Remove the point from the triangulation
            remove_point_from_exterior_ring(
                &mut exterior_ring_pts,
                &mut edges,
                index,
            );
        } else {
            triangulation_is_complete = true;
        }
    } */

    //Debug mode
    let mut geometry_collection_vector: Vec<geo_types::Geometry> = vec![];
    for &edge in &edges {
        println!("HERE Edge: {:?} {:?}", vertices[edge[0]], vertices[edge[1]]);
        if edge[0] > edge[1] {
            continue;
        }
        let line_segment = vertices[edge[0]..edge[1]].to_vec();
        let linestring = geo_types::LineString::from(line_segment.into_iter().map(|c| Coord{
            x: c.x,
            y: c.y,
        }).collect::<Vec<Coord<f64>>>());
        geometry_collection_vector.push(geo_types::Geometry::from(linestring));
    }

    let cdt: Result<ConstrainedDelaunayTriangulation<Point2<f64>>, spade::InsertionError> = ConstrainedDelaunayTriangulation::<_>::bulk_load_cdt(vertices, edges);


    let mut geometry_collection =
        geo_types::GeometryCollection::new_from(geometry_collection_vector);
    geometry_collection.transform(&transformer).expect("Failed to reproject geometry collection");

    let debug_string = Value::from(&geometry_collection).to_string();

    //End debug mode

 /*    if let Err(e) = &triangles {
        println!("Error creating triangulation: {:?}", e);
        println!("Error source: {:?}", e.source());
        //panic!("Error creating triangulation");
    } */

    debug_string
}

/*  #[tokio::main]
async fn main() {
    test().await;
} */
