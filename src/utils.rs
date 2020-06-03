use crate::{Circle, DubinsPathSegment, Landmark, Point};

pub fn convert_landmarks_to_dubins_path(landmarks: Vec<Landmark>) -> Vec<DubinsPathSegment> {
    unimplemented!()
}

pub struct SuccessorIter<'a> {
    pub parent: Landmark,
    pub target: Point,
    pub all_nodes: &'a Vec<Circle>,
}

impl<'a> Iterator for SuccessorIter<'a> {
    type Item = (Landmark, i64);

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}
