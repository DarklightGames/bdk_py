use crate::math::FVector;
use crate::math::{transform_vector_by_coords, mirror_vector_by_normal};

#[derive(Clone, Copy, Debug)]
pub struct FCoords {
    pub origin: FVector,
    pub x_axis: FVector,
    pub y_axis: FVector,
    pub z_axis: FVector,
}

impl FCoords {

    pub fn new() -> FCoords {
        FCoords {
            origin: FVector::new(0.0, 0.0, 0.0),
            x_axis: FVector::new(1.0, 0.0, 0.0),
            y_axis: FVector::new(0.0, 1.0, 0.0),
            z_axis: FVector::new(0.0, 0.0, 1.0),
        }
    }

    pub fn new_from_origin(origin: &FVector) -> FCoords {
        FCoords {
            origin: *origin,
            x_axis: FVector::new(1.0, 0.0, 0.0),
            y_axis: FVector::new(0.0, 1.0, 0.0),
            z_axis: FVector::new(0.0, 0.0, 1.0),
        }
    }

    pub fn new_from_origin_and_axes(origin: &FVector, x_axis: &FVector, y_axis: &FVector, z_axis: &FVector) -> FCoords {
        FCoords {
            origin: *origin,
            x_axis: *x_axis,
            y_axis: *y_axis,
            z_axis: *z_axis,
        }
    }

    pub fn mirror_by_vector(&self, mirror_normal: FVector) -> FCoords {
        FCoords {
            origin: mirror_vector_by_normal(self.origin, mirror_normal),
            x_axis: mirror_vector_by_normal(self.x_axis, mirror_normal),
            y_axis: mirror_vector_by_normal(self.y_axis, mirror_normal),
            z_axis: mirror_vector_by_normal(self.z_axis, mirror_normal),
        }
    }

    /// Return this coordinate system's transpose.
    /// If the coordinate system is orthogonal, this is equivalent to its inverse.
    pub fn transpose(&self) -> FCoords {
        FCoords {
            origin: -transform_vector_by_coords(&self.origin, self),    // TODO: order may be incorrect, original is `-Origin.TransformVectorBy(*this)`
            x_axis: FVector::new(self.x_axis.x, self.y_axis.x, self.z_axis.x),
            y_axis: FVector::new(self.x_axis.y, self.y_axis.y, self.z_axis.y),
            z_axis: FVector::new(self.x_axis.z, self.y_axis.z, self.z_axis.z),
        }
    }

}

/// A model coordinate system, describing both the covariant and contravariant
/// transformation matrices to transform points and normals by.
#[derive(Clone, Copy, Debug)]
pub struct FModelCoords {
    /// Coordinates to transform points by  (covariant). 
    pub covariant: FCoords,
    /// Coordinates to transform vectors by (contravariant).
    pub contravariant: FCoords,
}

impl Default for FModelCoords {
    fn default() -> Self {
        Self::new()
    }
}

impl FModelCoords {
    pub fn new() -> FModelCoords {
        FModelCoords {
            covariant: FCoords::new(),
            contravariant: FCoords::new(),
        }
    }

    pub fn new_with_coords(covariant: &FCoords, contravariant: &FCoords) -> FModelCoords {
        FModelCoords {
            covariant: *covariant,
            contravariant: *contravariant,
        }
    }

    pub fn inverse(&self) -> FModelCoords {
        FModelCoords {
            covariant: self.contravariant.transpose(),
            contravariant: self.covariant.transpose(),
        }
    }
}