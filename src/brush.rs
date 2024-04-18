use crate::bsp::ECsgOper;
use crate::fpoly::EPolyFlags;
use crate::model::UModel;
use crate::math::FVector;
use crate::coords::{FCoords, FModelCoords};

#[derive(Debug, PartialEq)]
pub struct ABrush {
    pub model: UModel,
    pub location: FVector,
    pub pre_pivot: FVector,
    pub poly_flags: EPolyFlags,
    pub csg_operation: ECsgOper,
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