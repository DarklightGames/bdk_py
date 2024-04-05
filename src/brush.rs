use crate::model::UModel;
use crate::math::FVector;
use crate::coords::{FCoords, FModelCoords};
use std::rc::Rc;
use std::cell::RefCell;

#[derive(Debug, PartialEq)]
pub struct ABrush {
    pub model: Rc<RefCell<UModel>>,
    pub location: FVector,
    pub pre_pivot: FVector,
}

impl ABrush {
    pub fn build_coords() -> (FModelCoords, FModelCoords, f32) {
        let coords = FModelCoords {
            covariant: FCoords::new(),
            contravariant: FCoords::new().transpose(),
        };
        let uncoords = FModelCoords {
            covariant: FCoords::new(),
            contravariant: FCoords::new().transpose(),
        };
        (coords, uncoords, 0.0)
    }
}