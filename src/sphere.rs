use cgmath::{Vector3, InnerSpace};
use crate::box_::FBox;

type FVector = Vector3<f32>;

pub struct FSphere {
    pub origin: FVector,
    pub radius: f32
}

impl FSphere {

    pub fn new() -> FSphere {
        FSphere {
            origin: FVector::new(0.0, 0.0, 0.0),
            radius: 0.0
        }
    }

    pub fn new_from_origin_and_radius(origin: &FVector, radius: f32) -> FSphere {
        FSphere {
            origin: *origin,
            radius
        }
    }

    /// Compute a bounding sphere from an array of points.
    pub fn new_from_points(points: &[FVector]) -> FSphere {
        let box_ = FBox::new_from_points(points);
        // Get the maximum distance from the center of the sphere to any point in the box.
        let origin = box_.center();
        let mut max_distance_squared: f32 = 0.0;
        for point in points {
            let distance_squared = (point - origin).magnitude2();
            max_distance_squared = max_distance_squared.max(distance_squared);
        }
        let radius = max_distance_squared.sqrt() * 1.001;
        FSphere::new_from_origin_and_radius(&origin, radius)
    }
}