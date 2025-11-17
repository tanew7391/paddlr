use std::collections::HashMap;

//Start FaceNode
use geo::{Contains, Coord};
use spade::{
    handles::{FaceHandle, InnerTag},
    ConstrainedDelaunayTriangulation, Point2, Triangulation,
};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FaceNode<'a> {
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
pub enum FaceNodeType {
    Inner,
    Outer,
}

impl<'a> FaceNode<'a> {
    pub fn new(
        face: FaceHandle<
            'a,
            InnerTag,
            Point2<f64>,
            <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::DirectedEdge,
            <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::UndirectedEdge,
            <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::Face,
        >,
    ) -> Self {
        Self { face }
    }

    pub fn successors(
        &self,
        focus_polygon: &geo_types::Polygon<f64>,
        checked_faces: &mut HashMap<usize, FaceNodeType>,
    ) -> Vec<(FaceNode<'a>, u32)> {
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

            checked_faces.entry(adjacent_face_index).or_insert_with(|| {
                //We need only process the faces that are inside the focus polygon
                //TODO: This is a very expensive operation - working on this would improve performance
                
                match focus_polygon.contains(&adjacent_center_point) {
                    true => FaceNodeType::Inner,
                    false => FaceNodeType::Outer,
                }
            });

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
    pub fn get_face(
        &self,
    ) -> FaceHandle<
        InnerTag,
        Point2<f64>,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::DirectedEdge,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::UndirectedEdge,
        <ConstrainedDelaunayTriangulation<Point2<f64>> as Triangulation>::Face,
    > {
        self.face
    }

    pub fn distance(&self, other: &FaceNode) -> u32 {
        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        let mut num_verticies = 0;
        for vertex in self.face.vertices() {
            x_sum += vertex.position().x;
            y_sum += vertex.position().y;
            num_verticies += 1;
        }

        let x = x_sum / num_verticies as f64;
        let y = y_sum / num_verticies as f64;

        let centroid = Point2::new(x, y);

        let mut shared_edge_option = None;
        for edge in self.face.adjacent_edges() {
            if edge.rev().face().index().eq(&other.get_face().index()) {
                shared_edge_option = Some(edge);
                break;
            }
        }

        let shared_edge_midpoint = match shared_edge_option {
            Some(edge) => edge.center(),
            None => return 15000,
        };

        shared_edge_midpoint.distance_2(centroid) as u32
    }
}
