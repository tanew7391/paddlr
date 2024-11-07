#[macro_use]
extern crate rocket;

use std::{collections::HashMap, error::Error};
use serde::Deserialize;
use serde_json::Map;

use geojson::{feature::Id::Number, Bbox, Feature, FeatureCollection, GeoJson, Geometry, PointType, Value};
use rocket::{data::{Data, ToByteUnit}};

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


async fn get_associated_water_features(bounding_area: Bbox) -> Result<GeoJson, Box<dyn Error>> {
    let client = reqwest::Client::new();

    let bounding_box_as_string = bounding_area.iter().map(|n| n.to_string()).collect::<Vec<_>>().join(", ");


    //TODO: Switch from requesting geoJson from OSM to using GDAL to convert. This will save time when requesting data from OSM
    let query = format!(
        r#"
        [out:json];
        (
        way["natural"="water"]({:});
        relation["natural"="water"]({:});
        );
        convert item ::=::,::geom=geom(),_osm_type=type();
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

    let json: OSMResponse = serde_json::from_str(&res)?;
    let features = json.elements.into_iter().map(|element| 
        Feature {
            geometry: Some(element.geometry),
            bbox: None,
            id: match element.id {
                Some(id) => Some(Number(id.into())),
                None => None
            },
            foreign_members: None,
            properties: match element.tags {
                Some(tags) => Some(tags),
                None => None
            }            
        }
    ).collect();

    let full_geojson = GeoJson::from(FeatureCollection {
        features,
        bbox: None,
        foreign_members: None
    });
    
    Ok(
        full_geojson
    )
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
    let data_string = data.open(1.megabytes()).into_string().await.unwrap();
    let json = data_string.parse::<GeoJson>().unwrap();

    let contained_multipoint_geometry = extract_multipoint_geometry(json).unwrap();

    let bounding_box: Bbox =
        extract_bounding_box_from_multipoint_geometry(&contained_multipoint_geometry).unwrap();

    //let line_string_json = multipoint_to_linestring(json);
    let associated_water_features_future = get_associated_water_features(bounding_box);
    //line_string_json.unwrap().to_string()
    let associated_water_features = associated_water_features_future.await.unwrap();
    associated_water_features.to_string()
}

#[launch]
fn rocket() -> _ {
    rocket::build().mount("/", routes![index])
}
