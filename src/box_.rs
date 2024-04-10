use cgmath::Vector3;
use std::ops::{Index, IndexMut};

type FVector = Vector3<f32>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FBox {
    pub min: FVector,
    pub max: FVector,
    pub is_valid: bool
}

impl Default for FBox {
    fn default() -> Self {
        Self::new()
    }
}

// A rectangular minimum bounding volume.
impl FBox {

    pub fn new() -> FBox {
        FBox {
            min: FVector::new(0.0, 0.0, 0.0),
            max: FVector::new(0.0, 0.0, 0.0),
            is_valid: false
        }
    }

    pub fn new_from_min_max(min: FVector, max: FVector) -> FBox {
        FBox { min, max, is_valid: true }
    }

    pub fn new_from_points(points: &[FVector]) -> FBox {
        let mut box_ = FBox::new();
        box_.add_points(points);
        box_
    }

    pub fn corners(&self) -> [FVector; 8] {
        [
            FVector::new(self.min.x, self.min.y, self.min.z),
            FVector::new(self.min.x, self.min.y, self.max.z),
            FVector::new(self.min.x, self.max.y, self.min.z),
            FVector::new(self.min.x, self.max.y, self.max.z),
            FVector::new(self.max.x, self.min.y, self.min.z),
            FVector::new(self.max.x, self.min.y, self.max.z),
            FVector::new(self.max.x, self.max.y, self.min.z),
            FVector::new(self.max.x, self.max.y, self.max.z),
        ]
    }

    // Returns the midpoint between the min and max points.
    pub fn center(&self) -> FVector {
        (self.min + self.max) * 0.5
    }

    /// Returns the extent around the center
    pub fn extent(&self) -> FVector {
        (self.max - self.min) * 0.5
    }

    pub fn add_point(&mut self, point: &FVector) {
        if self.is_valid {
            self.min.x = self.min.x.min(point.x);
            self.min.y = self.min.y.min(point.y);
            self.min.z = self.min.z.min(point.z);
            self.max.x = self.max.x.max(point.x);
            self.max.y = self.max.y.max(point.y);
            self.max.z = self.max.z.max(point.z);
        } else {
            self.min = point.clone();
            self.max = point.clone();
            self.is_valid = true;
        }
    }

    pub fn add_points(&mut self, points: &[FVector]) {
        for point in points {
            self.add_point(point);
        }
    }

    pub fn add_box(&mut self, other: &FBox) {
        if self.is_valid && other.is_valid {
            self.add_point(&other.min);
            self.add_point(&other.max);
        } else {
            *self = *other;
        }
    }

    pub fn expand_by(&self, w: f32) -> FBox {
        FBox::new_from_min_max(
            self.min - FVector::new(w, w, w), 
            self.max + FVector::new(w, w, w)
        )
    }

    pub fn intersect(&self, other: &FBox) -> bool {
        self.min.x <= other.max.x && self.max.x >= other.min.x &&
        self.min.y <= other.max.y && self.max.y >= other.min.y &&
        self.min.z <= other.max.z && self.max.z >= other.min.z
    }

}

impl Index<usize> for FBox {
    type Output = FVector;
    fn index<'a>(&'a self, index: usize) -> &'a FVector {
        match index {
            0 => &self.min,
            1 => &self.max,
            _ => panic!("Index out of bounds")
        }
    }
}

impl IndexMut<usize> for FBox {
    fn index_mut<'a>(&'a mut self, index: usize) -> &'a mut FVector {
        match index {
            0 => &mut self.min,
            1 => &mut self.max,
            _ => panic!("Index out of bounds")
        }
    }
}