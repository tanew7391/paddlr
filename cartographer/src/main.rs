#[macro_use]
extern crate rocket;

mod convert;

use convert::convert_osmjson_to_geo;
use geo::{BoundingRect, Contains, Geometry as GeoGeometry, PreparedGeometry, Relate};
use std::{
    collections::{HashMap, HashSet},
    error::Error,
};

use log::{info, warn, debug, self};
use std::fs;

use geo_types::{GeometryCollection, Point};

use geojson::{Bbox, GeoJson, Geometry, PointType, Value};
use rocket::data::{Data, ToByteUnit};

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

    let res = client
        .post(OVERPASS_API_URL)
        .header("Content-Type", "application/x-www-form-urlencoded")
        .body(format!("data={:}", query))
        .send()
        .await?
        .text()
        .await?;

    debug!("{:#?}", res);

    let geometry_collection = convert_osmjson_to_geo(&res);

    debug!("{:#?}", geometry_collection);

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
) -> Option<geo_types::Geometry> {
    // Iterate over each geometry in the collection
    //println!("Searching for location: {:?}", point);
    //println!("Geometry: {:#?}", geometries);

    for geometry in geometries {
        //println!("Geometry: {:#?}", geometry);
        //Start debug

        if let Some(bbox) = geometry.bounding_rect() {
            debug!("Bounding box {:?} :", bbox);
            if bbox.contains(point) {
                println!(
                    "Bounding box {:?} contains point!, Point is: {:?}",
                    bbox, point
                );
                // Perform the more precise contains check
                //println!("This is for geometry: {:?}", geometry);
                if geometry.contains(point) {
                    //println!("Geometry {:?} contains point!", geometry);
                    
                    return Some(geometry.clone()); // Return the first containing polygon
                }
            }
        }
    }
    warn!("Could not find bounding polygon");
    warn!("Point: {:?}", point);
    None // No containing polygon found
}

fn proof_of_concept_find_adjacent_geometries(
    geom_subject: &PreparedGeometry,
    geom_search_set: &HashMap<u32, PreparedGeometry>,
    adjacent_geom_set: &mut HashSet<u32>,
) {
    //TODO: add bounding box check
    /* //println!("Looking for Polygon Neighbours ;)");
    if let Err(error) = io::stdout().flush() {
        println!("{}", error);
    } */
    println!("Size of list: {:}", adjacent_geom_set.len());
    for (&id, polygon_search) in geom_search_set.iter() {
        if adjacent_geom_set.contains(&id) {
            continue;
        }
        let de9im_matrix: geo::relate::IntersectionMatrix = geom_subject.relate(polygon_search);
        println!("Not equal");
        if de9im_matrix.is_touches() || de9im_matrix.is_overlaps() {
            adjacent_geom_set.insert(id);
            proof_of_concept_find_adjacent_geometries(
                polygon_search,
                geom_search_set,
                adjacent_geom_set,
            );
        }
    }
}

fn test<'a>(
    start_polygon: &'a GeoGeometry,
    geom_search_set: Vec<&'a GeoGeometry>,
) -> Vec<&'a GeoGeometry> {
    //goal -  return a collection of polygons that represent a graph of the shortest path
    //https://stackoverflow.com/questions/57476033/algorithm-to-trim-a-graph

    //Take starting polygon out of the search set - assign ID of 0
    //Associate ID with each polygon starting from 1
    //Send search set to algorithm
    //Construct Vector of polygons from returned hashset of IDs

    let mut tagged_polygons: HashMap<u32, &GeoGeometry> = HashMap::new();
    let mut tagged_prepared_polygons: HashMap<u32, PreparedGeometry> = HashMap::new();

    //Insert the id 0, as this is the id of the geom_subject which will always be in the adjacent geom set
    let mut adjacent_geom_set: HashSet<u32> = HashSet::from([0]);

    tagged_polygons.insert(0, start_polygon);
    tagged_prepared_polygons.insert(0, PreparedGeometry::from(start_polygon));

    let mut index: u32 = 1;

    for polygon_to_search in geom_search_set {
        if polygon_to_search.eq(start_polygon) {
            continue;
        }

        tagged_polygons.insert(index, polygon_to_search);
        tagged_prepared_polygons.insert(index, PreparedGeometry::from(polygon_to_search));
        index += 1;
    }

    let geom_subject_prepared_polygon = tagged_prepared_polygons.get(&0).unwrap();
    proof_of_concept_find_adjacent_geometries(
        geom_subject_prepared_polygon,
        &tagged_prepared_polygons,
        &mut adjacent_geom_set,
    );

    let set: Vec<&'a GeoGeometry> = tagged_polygons
        .iter()
        .filter(|(&k, _)| adjacent_geom_set.contains(&k))
        .map(|(_, &v)| v)
        .collect();

    set
}

#[post("/api_test", data = "<data>")]
async fn test_index(data: Data<'_>) -> String {
    let data_string = data.open(1.megabytes()).into_string().await.unwrap();

    let json = data_string.parse::<GeoJson>().unwrap();

    let contained_multipoint_geometry: Vec<PointType> = extract_multipoint_geometry(json).unwrap();

    let bounding_box: Bbox =
        extract_bounding_box_from_multipoint_geometry(&contained_multipoint_geometry).unwrap();

    let associated_water_features_string = fs::read_to_string(
        "/home/tanew/Documents/Projects/Rust/Paddlr/test_files/square_lake.geojson",
    )
    .expect("Should have been able to read the file");


    let associated_water_features = associated_water_features_string.parse::<GeoJson>().unwrap();
    let mut collection: GeometryCollection<f64> = geojson::quick_collection(&associated_water_features).unwrap();



    let defined_point = Point::new(
        contained_multipoint_geometry[0][0],
        contained_multipoint_geometry[0][1],
    );

    //println!("After conv: {:#?}", collection);

    let possible_containing_polygon =
        find_containing_polygon(&defined_point, &collection);

    let geometry_container_reference_holder = Vec::from_iter(collection.iter());
    
    //let geojson_of_associated_water_features = Value::from(&geo::Geometry::GeometryCollection(associated_water_features));
    //println!("{:#?}", geojson_of_associated_water_features);
    //return geojson_of_associated_water_features.to_string();
   
    let containing_polygon = match possible_containing_polygon {
        Some(polygon) => polygon,
        None => panic!("No containing polygon!!!")
    };
    
    let set = test(&containing_polygon, geometry_container_reference_holder);

    Value::from(&geo::Geometry::GeometryCollection(
        GeometryCollection::new_from(Vec::from_iter(
            set.into_iter().map(|geom| geom.to_owned()),
        )),
    ))
    .to_string()

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

    let possible_containing_polygon =
        find_containing_polygon(&defined_point, &associated_water_features);

    let geometry_container_reference_holder = Vec::from_iter(associated_water_features.iter());
    
    //let geojson_of_associated_water_features = Value::from(&geo::Geometry::GeometryCollection(associated_water_features));
    //println!("{:#?}", geojson_of_associated_water_features);
    //return geojson_of_associated_water_features.to_string();
   
    let containing_polygon = match possible_containing_polygon {
        Some(polygon) => polygon,
        None => panic!("No containing polygon!!!")
    };
    
    let set = test(&containing_polygon, geometry_container_reference_holder);

    Value::from(&geo::Geometry::GeometryCollection(
        GeometryCollection::new_from(Vec::from_iter(
            set.into_iter().map(|geom| geom.to_owned()),
        )),
    ))
    .to_string()

    
}

#[launch]
fn rocket() -> _ {
    rocket::build().mount("/", routes![index, test_index])
}
