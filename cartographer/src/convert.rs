use std::collections::{hash_map, HashMap};

use serde::Deserialize;

pub mod convert {
    pub fn convert_osmjson_to_geojson() {}
}

#[derive(Deserialize, Debug)]
struct OSMResponse {
    version: Option<f32>,
    generator: Option<String>,
    osm3s: Option<HashMap<String, String>>,
    elements: Vec<Element>,
}

#[derive(Deserialize)]
enum ElementTypes {
    Node,
    Way,
    Relation,
}

#[derive(Deserialize, Debug)]
#[serde(untagged)]
enum Element {
    node(Node),
    way(Way),
    relation(Relation),
}

#[derive(Deserialize, Debug)]
struct Node {
    r#type: String,
    id: u64,
    lat: f32,
    lon: f32,
    tags: Option<HashMap<String, String>>,
}

#[derive(Deserialize, Debug)]
struct Way {
    r#type: String,
    id: u64,
    bounds: Option<Bounds>,
    nodes: Option<Vec<u64>>,
    geometry: Option<Vec<Coord>>,
    tags: Option<HashMap<String, String>>,
}

#[derive(Deserialize, Debug)]
struct Relation {
    r#type: String,
    id: u64,
    bounds: Option<Bounds>,
    members: Vec<Member>,
    tags: Option<HashMap<String, String>>,
}

#[derive(Deserialize, Clone, Copy, Debug)]
enum MemberTypes {
    Way,
    Node,
}

#[derive(Deserialize, Debug)]
//#[serde(untagged)]
enum Member {
    Way(WayMember),
    Node(NodeMember),
}

#[derive(Deserialize, Debug)]
struct WayMember {
    r#type: String,
    r#ref: u64,
    role: String,
    geometry: Option<Vec<Coord>>,
}

#[derive(Deserialize, Debug)]
struct NodeMember {
    r#type: String,
    r#ref: u64,
    role: String,
    lat: f32,
    lon: f32,
}

#[derive(Deserialize, Debug)]
struct Bounds {
    minlat: f32,
    minlon: f32,
    maxlat: f32,
    maxlon: f32,
}

#[derive(Deserialize, Debug)]
struct Coord {
    lat: f32,
    lon: f32,
}

fn convert_node_member_to_node(node_member: NodeMember) -> Node {
    let mut tags: HashMap<String, String> = HashMap::new();
    tags.insert("role".to_string(), node_member.role);
    return Node {
        r#type: node_member.r#type,
        id: node_member.r#ref,
        lat: node_member.lat,
        lon: node_member.lon,
        tags: Some(tags),
    };
}

fn convert_way_member_to_way(way_member: WayMember) -> Way {
    return Way {
        r#type: way_member.r#type,
        id: way_member.r#ref,
        bounds: None,
        nodes: None,
        geometry: way_member.geometry,
        tags: None,
    };
}

fn combine_ways(mut original_way: &mut Way, supplementary_way: Way) -> &Way {
    if original_way.bounds.is_none() && supplementary_way.bounds.is_some() {
        original_way.bounds = supplementary_way.bounds;
    }

    if original_way.geometry.is_none() && supplementary_way.geometry.is_some() {
        original_way.geometry = supplementary_way.geometry;
    }

    if original_way.nodes.is_none() && supplementary_way.nodes.is_some() {
        original_way.nodes = supplementary_way.nodes;
    }

    if supplementary_way.tags.is_some() {
        if let Some(ref mut original_tags) = original_way.tags {
            for tag in supplementary_way.tags.unwrap() {
                if !original_tags.contains_key(&tag.0){
                    original_tags.insert(tag.0, tag.1);
                }
            }
        } else {
            original_way.tags = supplementary_way.tags;
        }
    }

    return original_way;
}

pub fn convert_osmjson_to_geojson(input_json_string: &str) {
    let jd = &mut serde_json::Deserializer::from_str(input_json_string);
    println!("Json string: {:#?}", input_json_string);

    let result: Result<OSMResponse, _> = serde_path_to_error::deserialize(jd);
    //todo: refactor
    let response = match result {
        Ok(valid_response) => valid_response,
        Err(err) => {
            eprintln!("{:?}", err);
            let path = err.path().to_string();
            assert_eq!(path, "dependencies.serde.version");
            return;
        }
    };

    let mut ways: HashMap<u64, Way> = HashMap::new();
    let mut nodes: HashMap<u64, Node> = HashMap::new();
    let mut multipolygons: HashMap<u64, Relation> = HashMap::new();
    let mut multlinestrings: HashMap<u64, Relation> = HashMap::new();

    for element in response.elements {
        if let Element::node(node_instance) = element {
            if !nodes.contains_key(&node_instance.id) {
                nodes.insert(node_instance.id, node_instance);
            }
        } else if let Element::way(way_instance) = element {
            if !ways.contains_key(&way_instance.id){
                ways.insert(way_instance.id, way_instance);
            } else {
                let original_way = ways.get_mut(&way_instance.id).unwrap();
                println!("Way before update: {:?}", original_way);
                combine_ways(original_way, way_instance);
                println!("Way after update: {:?}", original_way);
            }
        } else if let Element::relation(relation_instance) = element {

        }
    }

    /*
    let osmjson: OSMResponse = serde_json::from_str(input_json_string).unwrap();
    println!("Deserialized: {:#?}", osmjson); */
}

#[cfg(test)]
mod tests {
    use std::fs;

    use super::*;

    #[test]
    fn exploration() {
        let contents = fs::read_to_string(
            "/home/tanew/Documents/Projects/Rust/Paddlr/cartographer/src/test.json",
        )
        .expect("Should have been able to read the file");

        convert_osmjson_to_geojson(&contents);
        assert!(true);
    }
}
