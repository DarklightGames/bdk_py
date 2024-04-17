use crate::bsp::{bsp_brush_csg, ECsgOper};
use crate::fpoly::EPolyFlags;
use crate::model::UModel;
use crate::brush::ABrush;

struct ULevel {
    model: UModel,
    brushes: Vec<ABrush>,
}

impl ULevel {
    pub fn new() -> Self {
        ULevel {
            model: UModel::new(),
            brushes: Vec::new(),
        }
    }
}

fn csg_rebuild(level: &mut ULevel) {
    // Empty the model out.
    level.model.empty_model(true, true);

    // Compose all structural brushes and portals.
    for brush in &level.brushes {
        if !brush.poly_flags.contains(EPolyFlags::Semisolid) || brush.csg_operation != ECsgOper::Add || brush.poly_flags.contains(EPolyFlags::Portal) {
            // Treat portals as solids for cutting.
            let mut poly_flags = brush.poly_flags;
            if poly_flags.contains(EPolyFlags::Portal) {
                poly_flags = (poly_flags & !EPolyFlags::Semisolid) | EPolyFlags::NotSolid;
                bsp_brush_csg(brush, &mut level.model, poly_flags, brush.csg_operation, false);
            }
        }
    }
}