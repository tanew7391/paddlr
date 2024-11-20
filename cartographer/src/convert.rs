use std::collections::{HashMap, HashSet};

use geo::{
    GeometryCollection, LineString, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon,
};
use serde::Deserialize;

pub mod convert {
    pub fn convert_osmjson_to_geojson() {}
}

#[derive(Deserialize, Debug)]
struct OSMResponse {
    version: Option<f64>,
    generator: Option<String>,
    osm3s: Option<HashMap<String, String>>,
    elements: Vec<Element>,
}

#[derive(Deserialize, Debug)]
#[serde(tag = "type")]
enum Element {
    node(Node),
    way(Way),
    relation(Relation),
}

#[derive(Deserialize, Debug, Clone)]
struct Node {
    id: u64,
    lat: f64,
    lon: f64,
    tags: Option<HashMap<String, String>>,
}

impl From<Node> for geo_types::Point {
    fn from(node: Node) -> Self {
        geo_types::Point(geo::Coord {
            x: node.lon,
            y: node.lat,
        })
    }
}

#[derive(Deserialize, Debug, Clone)]
struct Way {
    id: u64,
    bounds: Option<Bounds>,
    nodes: Option<Vec<u64>>,
    geometry: Option<Vec<Coord>>,
    tags: Option<HashMap<String, String>>,
}

#[derive(Deserialize, Debug)]
struct Relation {
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
#[serde(tag = "type")]
enum Member {
    way(WayMember),
    node(NodeMember),
}

#[derive(Deserialize, Debug)]
struct WayMember {
    r#ref: u64,
    role: String,
    geometry: Option<Vec<Coord>>,
}

#[derive(Deserialize, Debug)]
struct NodeMember {
    r#ref: u64,
    role: String,
    lat: f64,
    lon: f64,
}

#[derive(Deserialize, Debug, Clone)]
struct Bounds {
    minlat: f64,
    minlon: f64,
    maxlat: f64,
    maxlon: f64,
}

#[derive(Deserialize, Debug, Clone, Copy)]
struct Coord {
    lat: f64,
    lon: f64,
}

impl From<Coord> for geo_types::Coord {
    fn from(co: Coord) -> Self {
        geo_types::Coord {
            x: co.lon,
            y: co.lat,
        }
    }
}

#[derive(PartialEq, Debug)]
struct IncompletePolygon {
    exterior: LineString,
    interior: Vec<LineString>,
}

impl From<IncompletePolygon> for geo_types::Polygon {
    fn from(polygon_incomplete: IncompletePolygon) -> Self {
        geo_types::Polygon::new(polygon_incomplete.exterior, polygon_incomplete.interior)
    }
}

fn convert_node_member_to_node(node_member: NodeMember) -> Node {
    let mut tags: HashMap<String, String> = HashMap::new();
    tags.insert("role".to_string(), node_member.role);
    return Node {
        id: node_member.r#ref,
        lat: node_member.lat,
        lon: node_member.lon,
        tags: Some(tags),
    };
}

fn convert_way_member_to_way(way_member: WayMember) -> Way {
    return Way {
        id: way_member.r#ref,
        bounds: None,
        nodes: None,
        geometry: way_member.geometry,
        tags: None,
    };
}

fn combine_ways(original_way: &mut Way, supplementary_way: Way) -> &Way {
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
                if !original_tags.contains_key(&tag.0) {
                    original_tags.insert(tag.0, tag.1);
                }
            }
        } else {
            original_way.tags = supplementary_way.tags;
        }
    }

    return original_way;
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
        }
    }

    return (ways, nodes, relations);
}

pub fn convert_osmjson_to_geo(input_json_string: &str) -> GeometryCollection {
    let jd = &mut serde_json::Deserializer::from_str(input_json_string);

    let result: Result<OSMResponse, _> = serde_path_to_error::deserialize(jd);
    //todo: refactor
    let response = match result {
        Ok(valid_response) => valid_response,
        Err(err) => {
            eprintln!("{:?}", err);
            let path = err.path().to_string();
            panic!("dependencies.serde.version, should eq {:}", path);
        }
    };

    println!("result: {:#?}", response);

    let (mut ways, mut nodes, relations) =
        extract_nodes_ways_relations_from_elements(response.elements);

    let mut borrowed_ways: HashSet<u64> = HashSet::new();
    let mut borrowed_nodes: HashSet<u64> = HashSet::new();
    let mut geometry_collection: GeometryCollection<f64> = GeometryCollection::default();
    for (relation_id, relation) in relations {
        println!("Relation found, id: {:}", relation_id);
        //let mut relation_member_nodes: HashMap<u64, Node> = HashMap::new();
        //let mut relation_ways: HashMap<u32, Way> = HashMap::new();
        let mut referenced_relation_ways: HashMap<u32, u64> = HashMap::new();
        let mut linestrings: Vec<LineString> = vec![];
        let mut incomplete_polygons: Vec<IncompletePolygon> = vec![];
        let mut points: Vec<Point> = vec![];
        let mut index = 0;
        let mut encountered_nodes = false;
        let mut encountered_ways = false;

        if relation.members.len() == 0 {
            eprintln!("Relation {:} has no members!", relation_id);
            continue;
        }

        for member in relation.members {
            if let Member::node(node_member_instance) = member {
                let node_instance = convert_node_member_to_node(node_member_instance);
                points.push(Point::from(node_instance.clone()));
                borrowed_nodes.insert(node_instance.id);
                encountered_nodes = true;

                //TODO: do we need this map?
                //relation_member_nodes.insert(node_instance.id, node_instance);
            } else if let Member::way(way_member_instance) = member {
                encountered_ways = true;
                let way_ref = way_member_instance.r#ref;

                let is_outer = way_member_instance.role.eq("outer");
                let is_inner = way_member_instance.role.eq("inner");

                let geo_linestring: LineString;

                let original_way_wrapped = ways.get_mut(&way_ref);
                let mut relationship_way_instance = convert_way_member_to_way(way_member_instance);
                if let Some(original_way) = original_way_wrapped {
                    referenced_relation_ways.insert(index, way_ref);
                    combine_ways(original_way, relationship_way_instance);
                    attach_geometry_to_nodes_for_way(original_way, &nodes);
                    borrowed_ways.insert(original_way.id);
                    //TODO: test
                    geo_linestring = LineString::from(original_way.geometry.clone().unwrap());
                } else {
                    attach_geometry_to_nodes_for_way(&mut relationship_way_instance, &nodes);
                    //This may not have geometry, but if there does not exist a way external to the relationship, then there should have to be geom
                    geo_linestring =
                        LineString::from(relationship_way_instance.geometry.clone().unwrap());
                    //relation_ways.insert(index, relationship_way_instance);
                }

                if is_outer || is_inner {
                    add_linestring_to_polygons(geo_linestring, &mut incomplete_polygons, is_outer);
                } else {
                    linestrings.push(geo_linestring);
                }
            }
            //index += 1;
        }

        if !encountered_nodes {
            //Only Ways in relation

            //https://wiki.openstreetmap.org/wiki/Relation:multipolygon#Two_disjunct_outer_rings

            let multi_polygon =
                MultiPolygon::from_iter(incomplete_polygons.into_iter().map(|x| Polygon::from(x)));

            geometry_collection
                .0
                .push(geo::Geometry::MultiLineString(MultiLineString::new(
                    linestrings,
                )));

            geometry_collection
                .0
                .push(geo::Geometry::MultiPolygon(multi_polygon));

        } else if !encountered_ways {
            //Only nodes in relation

            geometry_collection
                .0
                .push(geo::Geometry::MultiPoint(MultiPoint::new(points)));

        } else {
            //Both nodes and ways in relation
            //Dissolve relationship and add all nodes and ways to the main node and way hashmaps (geojson has no feature for this relationship)
            //ways.insert(1, relation_ways.remove(0));
            
            println!("Relation: {:}, has both nodes and ways", relation_id);

            for incomplete_polygon in incomplete_polygons {
                geometry_collection.0.push(geo::Geometry::Polygon(incomplete_polygon.into()));
                println!("Adding polygon");
            }

            for linestring in linestrings {
                geometry_collection.0.push(
                    geo::Geometry::LineString(linestring)
                );
                println!("Adding linestring");
            }

            for point in points {
                geometry_collection.0.push(
                    geo::Geometry::Point(point)
                );
            }
        }
    }

    for way_id in borrowed_ways.iter() {
        ways.remove(way_id);
    }

    for (_way_id, way) in ways.iter_mut() {
        println!("Standalone way found, id: {:}", _way_id);
        //ensure all geometry is attached to ways before deleting nodes
        attach_geometry_to_nodes_for_way(way, &nodes);
    }

    for node_id in borrowed_nodes.iter() {
        nodes.remove(node_id);
    }

    for (_way_id, way) in ways.drain() {
        geometry_collection.0.push(geo::Geometry::LineString(LineString::from(way.geometry.unwrap())));
    }

    for (_node_id, node) in nodes.drain() {
        geometry_collection.0.push(geo::Geometry::Point(Point::from(node)));
    }


    return geometry_collection;
}

fn add_linestring_to_polygons(
    mut linestring: LineString,
    polygons: &mut Vec<IncompletePolygon>,
    is_outer: bool,
) {
    let push_new_polygon = |linestring: LineString, polygons: &mut Vec<IncompletePolygon>| {
        polygons.push(IncompletePolygon {
            exterior: linestring,
            interior: Vec::new(),
        });
    };

    if polygons.is_empty() {
        if !is_outer {
            panic!(
                "No polygon exists to hold the inner linestring boundary!\n linestring: {:?}",
                linestring
            );
        }

        push_new_polygon(linestring, polygons);

        return;
    }

    //  Cases
    //  Previous polygon in polygons has no inners
    //      If is_outer: check to see if the first coord of linestring is the last coord of the previous outer linestring, if so, merge them
    //                      If not, create a new polygon
    //      Else !is_outer: Append linestring to previous polygon inners
    //  Else prev has inners:
    //      If is outer: create a new polygon
    //      Else: check to see if the first coord of linestring is the last coord of the previous inner linestring, if so merge them.
    //              If not, append to the vector of interior linestrings

    let last_polygon = polygons.last_mut().unwrap();
    let last_polygon_last_exterior_coord = last_polygon
        .exterior
        .0
        .last()
        .expect("A polygon should have at least one coordinate on outer linestring");
    let linestring_first_coord = linestring
        .0
        .first()
        .expect("The provided linestring should have at least two coordinates");

    if is_outer && last_polygon.interior.is_empty() {
        if last_polygon_last_exterior_coord.eq(linestring_first_coord) {
            //Last coord of exterior linestring == first coord of new linestring. Merge
            last_polygon.exterior.0.pop();
            last_polygon.exterior.0.append(&mut linestring.0);
        } else {
            // New outer linestring, create new polygon.
            push_new_polygon(linestring, polygons);
        }
    } else if is_outer && !last_polygon.interior.is_empty() {
        //linestring is_outer and there exists inner linestrings on previous polygon
        push_new_polygon(linestring, polygons);
    } else {
        //linestring is inner
        attach_linestring_to_interior_of_polygon(linestring, &mut last_polygon.interior);
    }
}

fn attach_linestring_to_interior_of_polygon(
    mut linestring: LineString,
    interior: &mut Vec<LineString>,
) {
    let err_msg = "A linestring should have at least two coordinates";
    let linestring_first_coord = linestring.0.first().expect(&err_msg);

    if let Some(last_interior_linestring) = interior.last_mut() {
        let last_interior_linestring_last_coord =
            last_interior_linestring.0.last().expect(&err_msg);
        if last_interior_linestring_last_coord.eq(linestring_first_coord) {
            last_interior_linestring.0.pop();
            last_interior_linestring.0.append(&mut linestring.0);
            return;
        }
    }

    interior.push(linestring);
}

fn attach_geometry_to_nodes_for_way(way: &mut Way, nodes: &HashMap<u64, Node>) {
    if way.geometry.is_some() {
        return; //geometry already attached
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
                panic!("No associated node found for referenced node in way. \n Node ref: {:} \n Way: {:?}", node_ref, way);
            }
        }

        way.geometry = Some(geometry);
    } else {
        panic!(
            "No nodes nor geometry exist for combined way object! \n {:#?}",
            way
        );
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use geojson::Value;

    use super::*;

    #[test]
    fn exploration() {
        let contents = fs::read_to_string(
            "/home/tanew/Documents/Projects/Rust/Paddlr/test_files/no_turn_restriction.osm.json",
        )
        .expect("Should have been able to read the file");

        let geo = convert_osmjson_to_geo(&contents);
        let geojson = Value::from(&geo);
        println!("\n\n\n{:#?}", geojson.to_string());
        assert!(true);
    }

    #[test]
    fn test_add_linestring_to_polygons_handles_multiple_differing_outer_linestrings() {
        let linestring_one: LineString = vec![(0., 0.), (10., 0.)].into();

        let linestring_two: LineString = vec![(11., 0.), (20., 0.)].into();

        let mut polygons: Vec<IncompletePolygon> = vec![];
        let correct_polygons: Vec<IncompletePolygon> = vec![
            IncompletePolygon {
                exterior: linestring_one.clone(),
                interior: vec![],
            },
            IncompletePolygon {
                exterior: linestring_two.clone(),
                interior: vec![],
            },
        ];

        add_linestring_to_polygons(linestring_one, &mut polygons, true);
        add_linestring_to_polygons(linestring_two, &mut polygons, true);

        assert_eq!(polygons, correct_polygons);
    }

    #[test]
    fn test_add_linestring_to_polygons_handles_multiple_chained_outer_linestrings() {
        let linestring_one: LineString = vec![(0., 0.), (10., 0.)].into();

        let linestring_two: LineString = vec![(10., 0.), (20., 0.)].into();

        let combined_linestring: LineString = vec![(0., 0.), (10., 0.), (20., 0.)].into();

        let mut polygons: Vec<IncompletePolygon> = vec![IncompletePolygon {
            exterior: linestring_one,
            interior: vec![],
        }];
        let correct_polygons: Vec<IncompletePolygon> = vec![IncompletePolygon {
            exterior: combined_linestring,
            interior: vec![],
        }];

        add_linestring_to_polygons(linestring_two, &mut polygons, true);

        assert_eq!(polygons, correct_polygons);
    }

    #[test]
    fn test_add_linestring_to_polygons_handles_creating_new_polygon_when_previous_polygon_has_inners_with_is_outer_flag(
    ) {
        let linestring_one: LineString = vec![(0., 0.), (10., 0.)].into();

        let linestring_two: LineString = vec![(10., 0.), (20., 0.)].into();

        let mut polygons: Vec<IncompletePolygon> = vec![IncompletePolygon {
            exterior: linestring_one.clone(),
            interior: vec![linestring_one.clone()],
        }];

        let correct_polygons: Vec<IncompletePolygon> = vec![
            IncompletePolygon {
                exterior: linestring_one.clone(),
                interior: vec![linestring_one],
            },
            IncompletePolygon {
                exterior: linestring_two.clone(),
                interior: vec![],
            },
        ];

        add_linestring_to_polygons(linestring_two, &mut polygons, true);

        assert_eq!(polygons, correct_polygons);
    }

    #[test]
    fn test_add_linestring_to_polygons_handles_adding_to_interior_when_previous_polygon_has_mismatched_last_inner(
    ) {
        let linestring_outer: LineString = vec![(0., 0.), (5., 0.)].into();
        let linestring_inner_one: LineString = vec![(0., 0.), (10., 0.)].into();

        let linestring_inner_two: LineString = vec![(11., 0.), (20., 0.)].into();

        let mut polygons: Vec<IncompletePolygon> = vec![IncompletePolygon {
            exterior: linestring_outer.clone(),
            interior: vec![linestring_inner_one.clone()],
        }];

        let correct_polygons: Vec<IncompletePolygon> = vec![IncompletePolygon {
            exterior: linestring_outer.clone(),
            interior: vec![linestring_inner_one.clone(), linestring_inner_two.clone()],
        }];

        add_linestring_to_polygons(linestring_inner_two, &mut polygons, false);

        assert_eq!(polygons, correct_polygons);
    }

    #[test]
    fn test_add_linestring_to_polygons_handles_appending_interior_when_previous_polygon_has_mismatched_last_inner(
    ) {
        let linestring_outer: LineString = vec![(0., 0.), (5., 0.)].into();

        let linestring_inner_one: LineString = vec![(0., 0.), (10., 0.)].into();

        let linestring_inner_two: LineString = vec![(10., 0.), (20., 0.)].into();

        let combined_inner_linestring: LineString = vec![(0., 0.), (10., 0.), (20., 0.)].into();

        let mut polygons: Vec<IncompletePolygon> = vec![IncompletePolygon {
            exterior: linestring_outer.clone(),
            interior: vec![linestring_inner_one.clone()],
        }];

        let correct_polygons: Vec<IncompletePolygon> = vec![IncompletePolygon {
            exterior: linestring_outer.clone(),
            interior: vec![combined_inner_linestring],
        }];

        add_linestring_to_polygons(linestring_inner_two, &mut polygons, false);

        assert_eq!(polygons, correct_polygons);
    }

}
