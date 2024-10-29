#[macro_use] extern crate rocket;

use std::error::Error;

use geojson::{GeoJson, Geometry, Value};
use rocket::data::{ToByteUnit, Data};

fn multipoint_to_linestring(geojson: GeoJson) -> Result<GeoJson, Box<dyn Error>> {
    
    // Extract the MultiPoint geometry
    if let GeoJson::Feature(feature) = geojson {
        if let Some(Geometry { value: Value::MultiPoint(multipoints), .. }) = feature.geometry {
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
    println!("data_string: \n{:?}", data_string);
    let json = data_string.parse::<GeoJson>().unwrap();

    let line_string_json = multipoint_to_linestring(json);
    line_string_json.unwrap().to_string()
}

#[launch]
fn rocket() -> _ {
    rocket::build().mount("/", routes![index])
}