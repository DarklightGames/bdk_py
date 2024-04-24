use crate::bsp::{bsp_validate_brush, ECsgOper};
use crate::fpoly::{FPoly, EPolyFlags};
use crate::model::UModel;
use crate::math::FVector;
use crate::coords::{FCoords, FModelCoords};

#[derive(Debug, PartialEq)]
pub struct ABrush {
    pub model: UModel,
    pub id: usize,
    pub name: String,
    pub location: FVector,
    pub pre_pivot: FVector,
    pub poly_flags: EPolyFlags,
    pub csg_operation: ECsgOper,
}

impl ABrush {
    // TODO: consider just nuking the location & prepivot and expect the user to pass in the world-space polys.
    pub fn new(id: usize, name: String, polys: &[FPoly], poly_flags: EPolyFlags, csg_operation: ECsgOper) -> Self {
        let mut model = UModel::new(true);
        model.polys = polys.to_vec();
        bsp_validate_brush(&mut model, false);
        ABrush {
            id,
            name,
            model,
            location: FVector::new(0.0, 0.0, 0.0),
            pre_pivot: FVector::new(0.0, 0.0, 0.0),
            poly_flags,
            csg_operation,
        }
    }

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