use std::{
    borrow::BorrowMut, collections::{HashMap, HashSet}, error::Error
};

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

#[derive(Deserialize, Debug, Clone)]
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

#[derive(Deserialize, Clone, Debug, PartialEq)]
enum MemberTypes {
    Way,
    Node,
}

#[derive(Deserialize, Debug)]
#[serde(untagged)]
enum Member {
    way(WayMember),
    node(NodeMember),
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

#[derive(Deserialize, Debug, Clone)]
struct Bounds {
    minlat: f32,
    minlon: f32,
    maxlat: f32,
    maxlon: f32,
}

#[derive(Deserialize, Debug, Clone)]
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

fn combine_ways(original_way: &Way, supplementary_way: Way) -> Way {
    let mut new_way = original_way.clone();

    if new_way.bounds.is_none() && supplementary_way.bounds.is_some() {
        new_way.bounds = supplementary_way.bounds;
    }

    if new_way.geometry.is_none() && supplementary_way.geometry.is_some() {
        new_way.geometry = supplementary_way.geometry;
    }

    if new_way.nodes.is_none() && supplementary_way.nodes.is_some() {
        new_way.nodes = supplementary_way.nodes;
    }

    if supplementary_way.tags.is_some() {
        if let Some(ref mut original_tags) = new_way.tags {
            for tag in supplementary_way.tags.unwrap() {
                if !original_tags.contains_key(&tag.0) {
                    original_tags.insert(tag.0, tag.1);
                }
            }
        } else {
            new_way.tags = supplementary_way.tags;
        }
    }

    return new_way;
}

fn extract_nodes_ways_relations_from_elements(
    elements: Vec<Element>,
) -> (
    HashMap<u64, Way>,
    HashMap<u64, Node>,
    HashMap<u64, Relation>,
) {
    let mut ways: HashMap<u64, Way> = HashMap::new();
    let mut nodes: HashMap<u64, Node> = HashMap::new();
    let mut relations: HashMap<u64, Relation> = HashMap::new();

    for element in elements {
        if let Element::node(node_instance) = element {
            if !nodes.contains_key(&node_instance.id) {
                nodes.insert(node_instance.id, node_instance);
            }
        } else if let Element::way(way_instance) = element {
            if !ways.contains_key(&way_instance.id) {
                ways.insert(way_instance.id, way_instance);
            } else {
                /*                 let original_way = ways.get_mut(&way_instance.id).unwrap();
                println!("Way before update: {:?}", original_way);
                ways.insert(way_instance.id, combine_ways(original_way, way_instance));
                println!("Way after update: {:?}", original_way); */
            }
        } else if let Element::relation(relation_instance) = element {
            relations.insert(relation_instance.id, relation_instance);
            /*             let mut relation_member_type: Option<MemberTypes> = None;
            let mut member_type_has_changed = false;
            for member in relation_instance.members {

                //Todo: store this logic in an impl
                let current_member_type = match member {
                    Member::way(..) => MemberTypes::Way,
                    Member::node(..) => MemberTypes::Node,
                };

                if let Some(member_type) = relation_member_type {
                    if member_type != current_member_type {
                        //This is necessary to tell if nodes and ways are both present in the relationship.
                        //In which case we will store both in regular node and way hashmaps
                        member_type_has_changed = true;
                    }
                } else {
                    relation_member_type = Some(current_member_type);
                }



            } */
        }
    }

    return (ways, nodes, relations);
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

    let (ways, nodes, relations) = extract_nodes_ways_relations_from_elements(response.elements);

    let mut borrowed_ways: HashSet<u64> = HashSet::new();
    let mut borrowed_nodes: HashSet<u64> = HashSet::new();
    for (relation_id, relation) in relations {
        let mut relation_member_nodes: HashMap<u64, Node> = HashMap::new();
        let mut relation_ways: Vec<Way> = Vec::new();

        if relation.members.len() == 0 {
            eprintln!("Relation {:} has no members!", relation_id);
            continue;
        }

        let referenced_way_ids: Vec<u64> = vec![];
        for member in relation.members {
            if let Member::node(node_member_instance) = member {
                relation_member_nodes.insert(
                    node_member_instance.r#ref,
                    convert_node_member_to_node(node_member_instance),
                );
            } else if let Member::way(way_member_instance) = member {
                let way_ref = way_member_instance.r#ref;
                let original_way_wrapped = ways.get(&way_ref);
                let relationship_way_instance = convert_way_member_to_way(way_member_instance);
                if let Some(original_way) = original_way_wrapped {
                    let mut new_way = combine_ways(original_way, relationship_way_instance);
                    {
                        //mutable block
                        let tet = ways.get_mut(&way_ref).unwrap();
                        tet = 5;
                    }
                    new_way = attach_geometry_to_nodes_for_way(new_way, &nodes).unwrap();
                    relation_ways.push(new_way);
                } else {
                    relation_ways.push(relationship_way_instance);
                }
            }
        }
        if relation_member_nodes.is_empty() {
            //Only Ways in relation
            for way in relation_ways {
                //Keep track of ways to delete after all relations are processed
                //There may be more than one relation that uses the same way, which is why we hold onto them for now.
                borrowed_ways.insert(way.id);
            }
        } else if relation_ways.is_empty() {
            //Only Nodes in relation
        } else {
            //Both nodes and ways in relation
            //Dissolve relationship and add all nodes and ways to the main node and way hashmaps (geojson has no feature for this relationship)
            //ways.insert(1, relation_ways.remove(0));
        }
    }

    /*
    let osmjson: OSMResponse = serde_json::from_str(input_json_string).unwrap();
    println!("Deserialized: {:#?}", osmjson); */
}

fn attach_geometry_to_nodes_for_way(
    mut way: Way,
    nodes: &HashMap<u64, Node>,
) -> Result<Way, Box<dyn Error>> {
    if way.geometry.is_some() {
        return Ok(way); //geometry already attached
    }
    let mut geometry: Vec<Coord> = Vec::new();
    if let Some(node_refs) = &way.nodes {
        for node_ref in node_refs {
            if let Some(node) = nodes.get(node_ref) {
                geometry.push(Coord {
                    lat: node.lat,
                    lon: node.lon,
                });
            } else {
                return Err(format!("No associated node found for referenced node in way. \n Node ref: {:} \n Way: {:?}", node_ref, way).into());
            }
        }

        way.geometry = Some(geometry);
        return Ok(way);
    } else {
        return Err(format!(
            "No nodes nor geometry exist for combined way object! \n {:#?}",
            way
        )
        .into());
    }
}

fn convert_ways_to_multi_geometry(ways: Vec<Way>) {}

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
