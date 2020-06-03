//! This is a throw away library for doing path planning on a 2D plane with circular obstacles.
//!
//! TODO: document me more and add examples
//!
//!
//!
//!
//!
//!
//!
//!
//!
//!

#![deny(missing_docs)]
#![forbid(unsafe_code)]

use pathfinding::directed::astar::astar;

mod ffi;
pub use ffi::*;
mod utils;

/// Type alias to assign Point2D's type parameters and simplify the code.
pub type Point = euclid::default::Point2D<i32>;

/// The set of points some fixed distance away from a single center point.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(C)]
pub struct Circle {
    /// The point from which the distances to points in the set are measured.
    pub center: Point,
    /// The distance between the points in the set and the center.
    pub radius: u32,
}

/// Landmarks are a means of discretizing the search space without building an occupancy grid.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Landmark {
    /// Where the search begins.
    Origin(Point),
    /// Where the search is trying to find a path to.
    Destination(Point),
    /// An obstacle that the path goes left of.  Note that when you are going around
    /// something and are to the left of it you are making a right turn.
    LeftOf(Circle),
    /// An obstacle that the path goes right of.  Note that when you are going around
    /// something and are to the right of it you are making a left turn.
    RightOf(Circle),
}

/// Dubins paths are made of straight lines and circular arcs, which makes them an ideal
/// representation of the shortest path amidst a field of circular obstacles.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(C)]
pub enum DubinsPathSegment {
    /// A bounded segment of the geometric primitive known as a line.
    Line(Point, Point),
    /// A circular arc, beginning at the first point and continuing counter-clockwise to the second.
    LeftTurn(Circle, Point, Point), // TODO: come up with a more idiot-proof representation of the endpoints
    /// A circular arc, beginning at the first point and continuing clockwise to the second.
    RightTurn(Circle, Point, Point),
}

/// Computes a minimum length path from origin to destination that doesn't cross into any
/// of the given circles.
pub fn shortest_path_for_point(
    origin: Point,
    destination: Point,
    obstacles: &Vec<Circle>,
) -> Option<Vec<DubinsPathSegment>> {
    astar(
        &Landmark::Origin(origin),
        |node| node.enumerate_successors(&destination, obstacles),
        |node| node.distance_to(&destination),
        |node| node.is_at(&destination),
    )
    .map(|(path, _cost)| utils::convert_landmarks_to_dubins_path(path))
}

/// Computes a minimum length path from origin to destination that doesn't get within
/// `radius` of any of the given circles.
pub fn shortest_path_for_circle(
    origin: Point,
    destination: Point,
    obstacles: &Vec<Circle>,
    radius: u32,
) -> Option<Vec<DubinsPathSegment>> {
    let larger_obstacles =
        obstacles.iter().map(|o| Circle { center: o.center, radius: o.radius + radius }).collect();
    shortest_path_for_point(origin, destination, &larger_obstacles)
}

impl Landmark {
    fn distance_to(&self, target: &Point) -> i64 {
        self.location().to_f64().distance_to(target.to_f64()) as i64
    }

    fn is_at(&self, point: &Point) -> bool {
        self.location() == point
    }

    fn location(&self) -> &Point {
        match self {
            Landmark::Origin(pt) => pt,
            Landmark::Destination(pt) => pt,
            Landmark::LeftOf(o) => &o.center,
            Landmark::RightOf(o) => &o.center,
        }
    }

    fn enumerate_successors<'a>(
        &self,
        target: &Point,
        nodes: &'a Vec<Circle>,
    ) -> utils::SuccessorIter<'a> {
        utils::SuccessorIter { parent: *self, target: *target, all_nodes: nodes }
    }
}
