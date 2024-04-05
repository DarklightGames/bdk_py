use cgmath::InnerSpace;

use crate::coords::{FCoords, FModelCoords};
use crate::math::{FVector, points_are_same};
use crate::fpoly::{EPolyFlags, FPoly, RemoveColinearsResult, FPOLY_MAX_VERTICES};
use crate::model::{EBspNodeFlags, UModel, BSP_NODE_MAX_NODE_VERTICES};
use crate::brush::ABrush;
use std::borrow::BorrowMut;
use std::rc::Rc;
use std::cell::RefCell;

/// Possible positions of a child Bsp node relative to its parent (for `BspAddToNode`).
pub enum ENodePlace {
    /// Node is in back of parent              -> `Bsp[iParent].iBack`.
    Back,
    /// Node is in front of parent             -> `Bsp[iParent].iFront`.
    Front,
    /// Node is coplanar with parent           -> `Bsp[iParent].iPlane`.
    Plane,
    /// Node is the Bsp root and has no parent -> `Bsp[0]`.
    Root,
}

/// Status of filtered polygons:
pub enum EPolyNodeFilter {
    /// Leaf is an exterior leaf (visible to viewers).
   Outside,
   // Leaf is an interior leaf (non-visible, hidden behind backface).
   Inside,
   /// Poly is coplanar and in the exterior (visible to viewers).
   CoplanarOutside,
   /// Poly is coplanar and inside (invisible to viewers).
   CoplanarInside,
   /// Poly is coplanar, cospatial, and facing in.
   CospatialFacingIn,
   /// Poly is coplanar, cospatial, and facing out.
   CospatialFacingOut,
}

/// Trys to merge two polygons.  If they can be merged, replaces Poly1 and emptys Poly2
/// and returns true.  Otherwise, returns false.
pub fn try_to_merge(poly1: &mut FPoly, poly2: &mut FPoly) -> bool {
    // Vertex count reasonable?
    if poly1.vertices.len() + poly2.vertices.len() > FPOLY_MAX_VERTICES {
        return false
    }

    // Find one overlapping point.
    let mut matching_indices: Option<(usize, usize)> = None;
    for start1 in 0..poly1.vertices.len() {
        for start2 in 0..poly2.vertices.len() {
            if points_are_same(&poly1.vertices[start1], &poly2.vertices[start2]) {
                matching_indices = Some((start1, start2));
                break;
            }
        }
    }

    if matching_indices.is_none() {
        return false
    }

    let (start1, start2) = matching_indices.unwrap();

    // Wrap around trying to merge.
    let mut end1 = start1;
    let mut end2 = start2;
    let mut test1 = (start1 + 1) % poly1.vertices.len();
    let mut test2 = if start2 == 0 { poly2.vertices.len() - 1 } else { start2 - 1 };

    if points_are_same(&poly1.vertices[test1], &poly2.vertices[test2]) {
        end1 = test1;
        // start2 = test2;
    } else {
        test1 = if start1 == 0 { poly1.vertices.len() - 1 } else { start1 - 1 };
        test2 = (start2 + 1) % poly2.vertices.len();

        if points_are_same(&poly1.vertices[test1], &poly2.vertices[test2]) {
            // start1 = test1;
            end2 = test2;
        } else {
            return false
        }
    }

	// Build a new edpoly containing both polygons merged.
    let mut new_poly = poly1.clone();
    new_poly.vertices.clear();
    let mut vertex = end1;

    for _ in 0..poly1.vertices.len() {
        new_poly.vertices.push(poly1.vertices[vertex]);
        vertex = (vertex + 1) % poly1.vertices.len();
    }

    vertex = end2;

    for _ in 0..poly2.vertices.len() - 2 {
        vertex = (vertex + 1) % poly2.vertices.len();
        new_poly.vertices.push(poly2.vertices[vertex]);
    }

    // Remove colinear vertices and check convexity.
    match new_poly.remove_colinears() {
        RemoveColinearsResult::Convex => {
            if new_poly.vertices.len() <= BSP_NODE_MAX_NODE_VERTICES {
                *poly1 = new_poly;
                poly2.vertices.clear();
                true
            } else {
                false
            }
        }
        _ => return false
    }
}

/// Merge all polygons in coplanar list that can be merged convexly.
/// 
/// - Note that this assumes that all the polygons in the list are coplanar.
/// - This function now returns the number of polygons merged.
pub fn merge_coplanars(polys: &mut [FPoly], poly_indices: &[usize]) -> usize {
    let mut merged_count: usize = 0;
    let mut merge_again = true;
    while merge_again {
        merge_again = false;
        for i in 0..poly_indices.len() {
            let poly1 = &polys[poly_indices[i]];
            if poly1.vertices.len() == 0 {
                // Polygon has already been merged away.
                continue
            }
            for j in i + 1..poly_indices.len() {
                if let Ok([poly1, poly2]) = polys.get_many_mut([poly_indices[i], poly_indices[j]]) {
                    if poly2.vertices.len() == 0 {
                        continue
                    }
                    if try_to_merge(poly1, poly2) {
                        println!("Merged polygons {} into {}", poly_indices[j], poly_indices[i]);
                        merged_count += 1;
                        merge_again = true;
                    }
                }
            }
        }
    }
    merged_count
}

// TODO: not tested!
/// Merge near points that are near.
///
/// - Returns the number of points merged and the number of points collapsed.
pub fn merge_near_points(model: &mut UModel, dist: f32) -> (usize, usize) {
    let mut merged = 0usize;
    let mut collapsed = 0usize;
    let mut point_remap: Vec<usize> = (0..model.points.len()).collect();
    let dist2 = dist * dist;

    // Find nearer point for all points.
    for i in 0..model.points.len() {
        let point = &model.points[i];
        for j in 0..i {
            let test_point = &model.points[j];
            if (test_point - point).magnitude2() < dist2 {
                point_remap[i] = j;
                merged += 1;
                break
            }
        }
    }

    // Remap VertPool.
    for i in 0..model.vertices.len() {
        // TODO: peril abound, vertex_index is INT in original code.
        if model.vertices[i].vertex_index >= 0 && model.vertices[i].vertex_index < model.points.len() {
            model.vertices[i].vertex_index = point_remap[model.vertices[i].vertex_index];
        }
    }

    // Remap Surfs.
    for i in 0..model.surfaces.len() {
        if model.surfaces[i].base_point_index >= 0 && model.surfaces[i].base_point_index < model.points.len() {
            model.surfaces[i].base_point_index = point_remap[model.surfaces[i].base_point_index];
        }
    }

    // Remove duplicate points from nodes.
    for node in &mut model.nodes {
        let pool = &mut model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count as usize];
        let mut k = 0usize;
        for j in 0..node.vertex_count as usize {
            let a = &pool[j];
            let b = &pool[if j == 0 { node.vertex_count as usize - 1 } else { j - 1 }];

            if a.vertex_index != b.vertex_index {
                pool[k] = pool[j];
                k += 1;
            }
        }
        node.vertex_count = if k >= 3 { k as u8 } else { 0 };

        if k < 3 {
            collapsed += 1;
        }
    }

    (merged, collapsed)
}


fn bsp_add_node(model: &mut UModel, node_index: Option<usize>, node_place: ENodePlace, node_flags: EBspNodeFlags, ed_poly: &mut FPoly) {
    !todo!()
}


/// Global variables shared between bspBrushCSG and AddWorldToBrushFunc.  These are very
/// tightly tied into the function AddWorldToBrush, not general-purpose.
struct RebuildContext {
    /// Level map Model we're adding to.
    g_model: Rc<UModel>,
    /// Node AddBrushToWorld is adding to.
    g_node: usize,
    /// Last coplanar beneath GNode at start of AddWorldToBrush.
    g_last_coplanar: Option<usize>,
    /// Number of polys discarded and not added.
    g_discarded: usize,
    /// Errors encountered in Csg operation.
    g_errors: usize,
}

fn add_brush_to_world(model: &mut UModel, node_index: Option<usize>, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside => {
            bsp_add_node(model, node_index, node_place, EBspNodeFlags::IsNew, ed_poly);
        }
        EPolyNodeFilter::CospatialFacingOut => {
            if !ed_poly.poly_flags.contains(EPolyFlags::Semisolid) {
                bsp_add_node(model, node_index, node_place, EBspNodeFlags::IsNew, ed_poly);
            }
        }
        _ => {}
    }
}

fn add_world_to_brush(context: &mut RebuildContext, model: &mut UModel, node_index: Option<usize>, ed_poly: &mut FPoly, filter: EPolyNodeFilter, _: ENodePlace) {
    // Get a mutable refernce from the Rc<UModel> and add the node.
    let g_model = Rc::get_mut(&mut context.g_model).unwrap();
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside => {
            // Only affect the world poly if it has been cut.
            if ed_poly.poly_flags.contains(EPolyFlags::EdCut) {
                bsp_add_node(g_model, context.g_last_coplanar, ENodePlace::Plane, EBspNodeFlags::IsNew, ed_poly);
            }
        }
        _ => {
            // Discard original poly.
            if g_model.nodes[context.g_node].vertex_count > 0 {
                g_model.nodes[context.g_node].vertex_count = 0;
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ECsgOper {
    Add,
    Subtract,
    Intersect,
    Deintersect,
}

type BspFilterFunc = fn(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace);

fn bsp_filter_fpoly(filter_func: BspFilterFunc, model: &mut UModel, ed_poly: &mut FPoly) {
    !todo!()
}

fn add_brush_to_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    !todo!();
}

fn subtract_brush_from_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    !todo!();
}

fn intersect_brush_with_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    !todo!();
}

fn deintersect_brush_with_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    !todo!();
}

/// Perform any CSG operation between the brush and the world.
fn bsp_brush_csg(actor: &mut Rc<RefCell<ABrush>>, model: &mut UModel, poly_flags: EPolyFlags, csg_operation: ECsgOper, should_build_bounds: bool, should_merge_polys: bool) {
    // Non-solid and semisolid stuff can only be added.
    let not_poly_flags = match csg_operation {
        ECsgOper::Add => EPolyFlags::Semisolid | EPolyFlags::NotSolid,
        _ => EPolyFlags::empty()
    };

    let mut num_polys_from_brush = 0usize;

    // TODO: we're making a whole new model here, but the original code uses a global and clears it.
    let mut temp_model: UModel = UModel::new();
    temp_model.empty_model(true, true);

    // Build the brush's coordinate system and find orientation of scale
	// transform (if negative, edpolyTransform will reverse the clockness
	// of the EdPoly points and invert the normal).
    let (coords, uncoords, orientation) = ABrush::build_coords();

	// Transform original brush poly into same coordinate system as world
	// so Bsp filtering operations make sense.
    let pre_pivot = actor.borrow().pre_pivot;
    let location = actor.borrow().location;
    let actor_copy = actor.clone();
    let brush_ptr = Rc::get_mut(actor).unwrap().borrow_mut().get_mut();
    let brush = Rc::get_mut(&mut brush_ptr.model).unwrap().get_mut();
    for (poly_index, poly) in brush.polys.iter().enumerate() {
		// Set texture the first time.
        // SKIPPED

        // Get the brush poly.
        let mut dest_ed_poly = poly.clone();
        assert!(brush.polys[poly_index].link.is_none() && brush.polys[poly_index].link.unwrap() < brush.polys.len());

        // Set its backward brush link.
        dest_ed_poly.actor = Some(actor_copy.clone());
        dest_ed_poly.brush_poly_index = Some(poly_index);

        // Update its flags.
        dest_ed_poly.poly_flags = (dest_ed_poly.poly_flags | poly_flags) & !not_poly_flags;

        // Set its internal link.
        if dest_ed_poly.link.is_none() {
            dest_ed_poly.link = Some(poly_index);
        }

        // Transform it.
        dest_ed_poly.transform(&coords, &pre_pivot, &location, orientation);

        // Add poly to the temp model.
        temp_model.polys.push(dest_ed_poly);
    }

    let filter_func = match csg_operation {
        ECsgOper::Add => add_brush_to_world_func,
        ECsgOper::Subtract => subtract_brush_from_world_func,
        ECsgOper::Intersect => intersect_brush_with_world_func,
        ECsgOper::Deintersect => deintersect_brush_with_world_func,
    };

    // Pass the brush polys through the world Bsp.
    match csg_operation {
        ECsgOper::Intersect | ECsgOper::Deintersect => {
            // Empty the brush.
            brush.empty_model(true, true);

            // Intersect and deintersect.
            for poly in &mut temp_model.polys {
                // TODO: global variable GModel is not defined or used yet.
                // GModel = Brush;
                bsp_filter_fpoly(filter_func, model, poly);
            }
            num_polys_from_brush = brush.polys.len();
        },
        ECsgOper::Add | ECsgOper::Subtract => {
            for (i, poly) in brush.polys.iter().enumerate() {
                let mut ed_poly = poly.clone();
                // Mark the polygon as non-cut so that it won't be harmed unless it must
                 // be split, and set iLink so that BspAddNode will know to add its information
                 // if a node is added based on this poly.
                ed_poly.poly_flags &= !EPolyFlags::EdCut;

                if ed_poly.link == Some(i) {
                    temp_model.polys[i].link = Some(model.surfaces.len());
                    ed_poly.link = Some(model.surfaces.len());
                } else {
                    ed_poly.link = temp_model.polys[ed_poly.link.unwrap()].link;
                }

                // Filter brush through the world.
                bsp_filter_fpoly(filter_func, model, &mut ed_poly);
            }
        }
        _ => {}
    }
}