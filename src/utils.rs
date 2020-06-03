use crate::{Circle, DubinsPathSegment, Landmark, Point};

pub fn convert_landmarks_to_dubins_path(landmarks: Vec<Landmark>) -> Vec<DubinsPathSegment> {
    unimplemented!()
}

pub struct SuccessorIter<'a> {
    parent: Landmark,
    target: Landmark,
    all_nodes: &'a Vec<Circle>,
    inner: Box<dyn Iterator<Item = Landmark> + 'a>,
}

impl<'a> SuccessorIter<'a> {
    pub fn new(parent: Landmark, target: Point, all_nodes: &Vec<Circle>) -> SuccessorIter {
        let target = Landmark::Destination(target);
        // TODO: if we want to trade RAM for CPU, sort all_nodes by deviation from the bee line to the target
        let inner = std::iter::once(target).chain(all_nodes.iter().flat_map(|node| {
            (0..2_i32).map(move |side| {
                if side == 0 {
                    Landmark::LeftOf(*node)
                } else {
                    Landmark::RightOf(*node)
                }
            })
        }));
        SuccessorIter { parent, target, all_nodes, inner: Box::new(inner) }
    }

    fn safe_cost_to(&self, destination: &Landmark) -> Option<i64> {
        unimplemented!()
    }
}

impl<'a> Iterator for SuccessorIter<'a> {
    type Item = (Landmark, i64);

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(successor) = self.inner.next() {
            if let Some(cost) = self.safe_cost_to(&successor) {
                return Some((successor, cost));
            }
        }
        None
    }
}
