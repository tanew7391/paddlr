use std::{
    collections::HashMap,
    ops::{Div, Mul},
};

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

            if !checked_faces.contains_key(&adjacent_face_index) {
                //We need only process the faces that are inside the focus polygon
                //TODO: This is a very expensive operation - working on this would improve performance
                let face_node_type = match focus_polygon.contains(&adjacent_center_point) {
                    true => FaceNodeType::Inner,
                    false => FaceNodeType::Outer,
                };
                checked_faces.insert(adjacent_face_index, face_node_type);
            }

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

    //TODO: see if this is discarding SigFigs which could be important for pathfinding
    pub fn distance(&self, other: &FaceNode) -> u32 {
        self.face
            .center()
            .distance_2(other.face.center())
            .sqrt()
            .mul(1000.0) // For conserving precision
            .round()
            .div(1000.0) as u32
    }
}
