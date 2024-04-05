use cgmath::{Vector3, InnerSpace};
use crate::coords::FCoords;


pub type FVector = Vector3<f32>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FPlane {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl FPlane {
    pub fn normal(&self) -> FVector {
        FVector::new(self.x, self.y, self.z)
    }

    pub fn plane_dot(&self, p: FVector) -> f32 {
        (self.x * p.x) + (self.y * p.y) + (self.z * p.z) - self.w
    }
}

// Floating point constants.

/// Lengths of normalized vectors (These are half their maximum values
/// to assure that dot products with normalized vectors don't overflow).
pub const FLOAT_NORMAL_THRESH: f32 = 0.0001;

// Magic numbers for numerical precision.

/// Thickness of plane for front/back/inside test
pub const THRESH_POINT_ON_PLANE: f32 = 0.10;
/// Thickness of polygon side's side-plane for point-inside/outside/on side test
pub const THRESH_POINT_ON_SIDE: f32 = 0.20;
/// Two points are same if within this distance
pub const THRESH_POINTS_ARE_SAME: f32 = 0.002;
/// Two points are near if within this distance and can be combined if imprecise math is ok
pub const THRESH_POINTS_ARE_NEAR: f32 = 0.015;
/// Two normal points are same if within this distance
/// Making this too large results in incorrect CSG classification and disaster
pub const THRESH_NORMALS_ARE_SAME: f32 = 0.00002;
/// Two vectors are near if within this distance and can be combined if imprecise math is ok
/// Making this too large results in lighting problems due to inaccurate texture coordinates
pub const THRESH_VECTORS_ARE_NEAR: f32 = 0.0004;
///  A plane splits a polygon in half
pub const THRESH_SPLIT_POLY_WITH_PLANE: f32 = 0.25;
/// A plane exactly splits a polygon
pub const THRESH_SPLIT_POLY_PRECISELY: f32 = 0.01;
/// Size of a unit normal that is considered "zero", squared
pub const THRESH_ZERO_NORM_SQUARED: f32 = 0.0001;
/// Vectors are parallel if dot product varies less than this
pub const THRESH_VECTORS_ARE_PARALLEL: f32 = 0.02;

pub const SMALL_NUMBER: f32 = 1.0e-8;
pub const KINDA_SMALL_NUMBER: f32 = 1.0e-4;

pub fn points_are_same(p: &FVector, q: &FVector) -> bool {
    for i in 0..3 {
        let temp = (p[i] - q[i]).abs();
        if temp >= THRESH_POINTS_ARE_SAME {
            return false
        }
    }
    true
}


// Compare two points and see if they're the same, using a threshold.
// Uses fast distance approximation.
pub fn points_are_near(point1: &FVector, point2: &FVector, distance: f32) -> bool {
	if (point1.x - point2.x).abs() >= distance {
        return false;
    }
    if (point1.y - point2.y).abs() >= distance {
        return false;
    }
    if (point1.z - point2.z).abs() >= distance {
        return false;
    }
	true
}


/// Calculate the signed distance (in the direction of the normal) between a point and a plane.
pub fn point_plane_distance(point: &FVector, plane_base: &FVector, plane_normal: &FVector) -> f32 {
    (point - plane_base).dot(*plane_normal)
}

/// Find the intersection of an infinite line (defined by two points) and
/// a plane.  Assumes that the line and plane do indeed intersect; you must
/// make sure they're not parallel before calling.
pub fn line_plane_intersection(point1: &FVector, point2: &FVector, plane_base: &FVector, plane_normal: &FVector) -> FVector {
    point1
        + ((point2 - point1) *
            ((plane_base - point1).dot(*plane_normal)
                / (point2 - point1).dot(*plane_normal)))
}

// ============================================================================
// FVector functions
// ============================================================================

/// Transform a directional vector by a coordinate system.
/// Ignore's the coordinate system's origin.
/// 
/// Previously `FVector::TransformVectorBy`.
pub fn transform_vector_by_coords(v: &FVector, coords: &FCoords) -> FVector {
    FVector {
        x: v.dot(coords.x_axis),
        y: v.dot(coords.y_axis),
        z: v.dot(coords.z_axis),
    }
}

/// Mirror a vector about a normal vector.
/// 
/// Previously `FVector::MirrorByVector`.
pub fn mirror_vector_by_normal(v: FVector, normal: FVector) -> FVector {
    v - normal * (2.0 * v.dot(normal))
}

/// Mirror a vector about a plane.
///
/// Previously `FVector::MirrorByPlane`.
pub fn mirror_vector_by_plane(v: FVector, plane: FPlane) -> FVector {
    v - plane.normal() * (2.0 * plane.plane_dot(v))
}
