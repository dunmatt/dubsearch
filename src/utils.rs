use crate::{Circle, DubinsPathSegment, Landmark, Point};

pub fn convert_landmarks_to_dubins_path(landmarks: Vec<Landmark>) -> Vec<DubinsPathSegment> {
    unimplemented!()
}

pub struct SuccessorIter<'a> {
    parent: Landmark,
    target: Point,
    all_nodes: &'a Vec<Circle>,
}

impl<'a> SuccessorIter<'a> {
    pub fn new(parent: Landmark, target: Point, all_nodes: &Vec<Circle>) -> SuccessorIter {
        SuccessorIter { parent, target, all_nodes }
    }
}

impl<'a> Iterator for SuccessorIter<'a> {
    type Item = (Landmark, i64);

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}
