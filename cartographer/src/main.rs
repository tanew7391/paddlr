#[macro_use]
extern crate rocket;

mod convert;

use convert::convert_osmjson_to_geo;
use geo::{BoundingRect, Contains, Polygon};
use serde::Deserialize;
use serde_json::Map;
use std::{collections::HashMap, error::Error};

use geo_types::{GeometryCollection, Point};


use geojson::{
    feature::Id::Number, quick_collection, Bbox, Feature, FeatureCollection, GeoJson, Geometry,
    PointType, Value,
};
use rocket::data::{Data, ToByteUnit};

const OVERPASS_API_URL: &str = "https://overpass-api.de/api/interpreter";

#[derive(Deserialize)]
struct OSMResponse {
    version: Option<f32>,
    generator: Option<String>,
    osm3s: Option<HashMap<String, String>>,
    elements: Vec<Element>,
}

#[derive(Deserialize)]
struct Element {
    r#type: Option<String>,
    id: Option<u32>,
    tags: Option<Map<String, serde_json::Value>>,
    geometry: Geometry,
}

async fn get_associated_water_features(bounding_area: Bbox) -> Result<GeometryCollection, Box<dyn Error>> {
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

    println!("Query: \n{query}");

    let res = client
        .post(OVERPASS_API_URL)
        .header("Content-Type", "application/x-www-form-urlencoded")
        .body(format!("data={:}", query))
        .send()
        .await?
        .text()
        .await?;

    let geometry_collection = convert_osmjson_to_geo(&res);

    Ok(geometry_collection)
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



fn find_containing_polygon<'a>(
    point: &Point<f64>,
    geometries: &'a GeometryCollection<f64>,
) -> Option<&'a geo_types::Polygon> {
    // Iterate over each geometry in the collection
    println!("Searching for location: {:?}", point);
    println!("Geometry: {:#?}", geometries);

    for geometry in geometries {
        //println!("Geometry: {:#?}", geometry);
        //Start debug



        if let geo::Geometry::Polygon(polygon) = geometry {
            println!("Within polygon: {:?}", polygon);
            if let Some(bbox) = geometry.bounding_rect() {
                println!("Bounding box {:?} :", bbox);
                if bbox.contains(point) {
                    println!(
                        "Bounding box {:?} contains point!, Point is: {:?}",
                        bbox, point
                    );
                    // Perform the more precise contains check
                    println!("This is for geometry: {:?}", polygon);
                    if polygon.contains(point) {
                        println!("Geometry {:?} contains point!", polygon);
                        return Some(&polygon); // Return the first containing polygon
                    }
                }
            }
        }

    }
    None // No containing polygon found
}

#[post("/api", data = "<data>")]
async fn index(data: Data<'_>) -> String {
    let data_string = data.open(1.megabytes()).into_string().await.unwrap();
    
    let json = data_string.parse::<GeoJson>().unwrap();

    let contained_multipoint_geometry: Vec<PointType> = extract_multipoint_geometry(json).unwrap();

    let bounding_box: Bbox =
        extract_bounding_box_from_multipoint_geometry(&contained_multipoint_geometry).unwrap();

    //let line_string_json = multipoint_to_linestring(json);
    let associated_water_features_future = get_associated_water_features(bounding_box);
    //line_string_json.unwrap().to_string()
    let associated_water_features = associated_water_features_future.await.unwrap();
    //println!("Water features: {:}", associated_water_features.to_string());


    let defined_point = Point::new(
        contained_multipoint_geometry[0][0],
        contained_multipoint_geometry[0][1],
    );

    //println!("After conv: {:#?}", collection);


    let possible_containing_polygon = find_containing_polygon(&defined_point, &associated_water_features);

    match possible_containing_polygon {
        Some(polygon) => Value::from(&geo::Geometry::Polygon(polygon.clone())).to_string(),
        None => Value::from(&associated_water_features).to_string(),
    }
}

#[launch]
fn rocket() -> _ {
    rocket::build().mount("/", routes![index])
}
