use crate::bsp::{bsp_brush_csg, ECsgOper, bsp_repartition, bsp_build_bounds};
use crate::fpoly::EPolyFlags;
use crate::model::UModel;
use crate::brush::ABrush;

pub struct ULevel {
    pub model: UModel,
    pub brushes: Vec<ABrush>,
}

impl ULevel {
    pub fn new(brushes: Vec<ABrush>) -> Self {
        ULevel {
            model: UModel::new(false),
            brushes: brushes,
        }
    }
}

/// Build list of leaves.
pub fn enlist_leaves(model: &UModel, front_node_indices: &mut Vec<usize>, back_node_indices: &mut Vec<usize>) {
    // BDK: This was done with recursion in the original code, but a stack is cleaner and easier to debug.
    let mut node_index_stack = vec![0usize];

    if model.nodes.is_empty() {
        return;
    }

    while let Some(node_index) = node_index_stack.pop() {
        let node = &model.nodes[node_index];

        match node.front_node_index {
            None => {
                front_node_indices.push(node_index);
            }
            Some(front_node_index) => {
                node_index_stack.push(front_node_index);
            }
        }
        
        match node.back_node_index {
            None => {
                back_node_indices.push(node_index);
            }
            Some(back_node_index) => {
                node_index_stack.push(back_node_index);
            }
        }
    }
}

pub fn csg_rebuild(level: &mut ULevel) {
    // Empty the model out.
    level.model.empty_model(true, true);

    // Compose all structural brushes and portals.
    for brush in &level.brushes {
        if !brush.poly_flags.contains(EPolyFlags::Semisolid) || brush.csg_operation != ECsgOper::Add || brush.poly_flags.contains(EPolyFlags::Portal) {
            // Treat portals as solids for cutting.
            let mut poly_flags = brush.poly_flags;
            if poly_flags.contains(EPolyFlags::Portal) {
                poly_flags = (poly_flags & !EPolyFlags::Semisolid) | EPolyFlags::NotSolid;
            }

            bsp_brush_csg(brush, &mut level.model, poly_flags, brush.csg_operation, false);
        }
    }

    // Repartition the structural BSP.
    bsp_repartition(&mut level.model, 0, false);

    // test_visibility(level, level.model, 0, false);  // TODO: borrowing issues here obviously.

    // Remember leaves
    let mut front_node_indices = vec![];
    let mut back_node_indices = vec![];
    if level.model.nodes.is_empty() {
        enlist_leaves(&level.model, &mut front_node_indices, &mut back_node_indices);
    }

    return ();

    // Compose all detail brushes.
    for brush in &level.brushes {
        if brush.poly_flags.contains(EPolyFlags::Semisolid) && brush.csg_operation == ECsgOper::Add && !brush.poly_flags.contains(EPolyFlags::Portal) {
            bsp_brush_csg(brush, &mut level.model, brush.poly_flags, brush.csg_operation, false);
        }
    }
    
    // TODO: the "2" is a number that dodges either of the model-emptying calls downstream in bspBuild.
    // Come up with a better way to handle this 'cause it's confusing af.
	// Optimize the sub-bsp's.
    // for front_node_index in front_node_indices {
    //     bsp_repartition(&mut level.model, front_node_index, 2);
    // }
    // for back_node_index in back_node_indices {
    //     bsp_repartition(&mut level.model, back_node_index, 2);
    // }

    // Build bounding volumes.
    // bsp_opt_geom(&mut level.model);
    bsp_build_bounds(&mut level.model);

    // Rebuild dynamic brush BSP's. // BDK: skip.

    // TODO: more to this, obviously.
}