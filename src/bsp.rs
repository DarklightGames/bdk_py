use cgmath::InnerSpace;

use crate::box_::FBox;
use crate::math::{points_are_near, points_are_same, FVector, FPlane, THRESH_VECTORS_ARE_NEAR};
use crate::fpoly::{EPolyFlags, ESplitType, FPoly, RemoveColinearsResult, FPOLY_MAX_VERTICES, FPOLY_VERTEX_THRESHOLD};
use crate::model::{EBspNodeFlags, FBspNode, FBspSurf, FVert, UModel, BSP_NODE_MAX_NODE_VERTICES};
use crate::brush::ABrush;
use crate::sphere::FSphere;
use std::borrow::BorrowMut;
use arrayvec::ArrayVec;
use std::iter;
use std::rc::Rc;
use std::cell::RefCell;


/// Possible positions of a child Bsp node relative to its parent (for `BspAddToNode`).
#[derive(Clone, Copy, Debug, PartialEq)]
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
#[derive(Clone, Copy, Debug, PartialEq)]
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EBspOptimization {
	Lame,
	Good,
	Optimal,
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
        node.vertex_count = if k >= 3 { k } else { 0 };

        if k < 3 {
            collapsed += 1;
        }
    }

    (merged, collapsed)
}


/// Add an editor polygon to the Bsp, and also stick a reference to it
/// in the editor polygon's BspNodes list. If the editor polygon has more sides
/// than the Bsp will allow, split it up into several sub-polygons.
///
/// Returns: Index to newly-created node of Bsp.  If several nodes were created because
/// of split polys, returns the parent (highest one up in the Bsp).
pub fn bsp_add_node(model: &mut UModel, mut parent_node_index: usize, node_place: ENodePlace, mut node_flags: EBspNodeFlags, ed_poly: &FPoly) -> usize {
    if node_place == ENodePlace::Plane {
		// Make sure coplanars are added at the end of the coplanar list so that 
		// we don't insert NF_IsNew nodes with non NF_IsNew coplanar children.
        while let Some(plane_index) = model.nodes[parent_node_index].plane_index {
            parent_node_index = plane_index
        }
    }
    
    let (surface, surface_index) = match ed_poly.link {
        None => {
            let mut surf = FBspSurf::default();

            // This node has a new polygon being added by bspBrushCSG; must set its properties here.
            surf.base_point_index = model.bsp_add_point(ed_poly.base, true);
            surf.normal_index = model.bsp_add_vector(ed_poly.normal, true);
            surf.texture_u_index = model.bsp_add_vector(ed_poly.texture_u, false);
            surf.texture_v_index = model.bsp_add_vector(ed_poly.texture_v, false);
            // Surf->Material = EdPoly->Material;
            // Surf->Actor = EdPoly->Actor;
            surf.poly_flags = ed_poly.poly_flags & !EPolyFlags::NoAddToBSP;
            surf.light_map_scale = ed_poly.light_map_scale;
            surf.brush_polygon_index = ed_poly.brush_poly_index;
            surf.plane = FPlane::new_from_origin_and_normal(ed_poly.vertices.first().unwrap(), &ed_poly.normal);
            
            let surface_index = model.surfaces.len();
            model.surfaces.push(surf);
            (&model.surfaces[surface_index], surface_index)
        }
        Some(link) => {
            assert!(link < model.surfaces.len());
            (&model.surfaces[link], link)
        }
    };

	// Set NodeFlags.
    if surface.poly_flags.contains(EPolyFlags::NotSolid) {
        node_flags |= EBspNodeFlags::NotCsg;
    }
    if surface.poly_flags.contains(EPolyFlags::Invisible | EPolyFlags::Portal) {
        node_flags |= EBspNodeFlags::NotVisBlocking;
    }
    if surface.poly_flags.contains(EPolyFlags::Masked) {
        node_flags |= EBspNodeFlags::ShootThrough;
    }

    if ed_poly.vertices.len() > BSP_NODE_MAX_NODE_VERTICES {
		// Split up into two coplanar sub-polygons (one with MAX_NODE_VERTICES vertices and
		// one with all the remaining vertices) and recursively add them.

        // Copy first bunch of verts.
        // TODO: test and extract this to a function.
        let mut ed_poly1 = ed_poly.clone();
        ed_poly1.vertices.truncate(BSP_NODE_MAX_NODE_VERTICES);
        let mut ed_poly2 = ed_poly.clone();
        ed_poly2.vertices.drain(1..BSP_NODE_MAX_NODE_VERTICES);

        let node_index = bsp_add_node(model, parent_node_index, node_place, node_flags, &mut ed_poly1);
        bsp_add_node(model, node_index, ENodePlace::Plane, node_flags, &mut ed_poly2);

        node_index
    } else {
        // Add node.
        let node_index = model.nodes.len();
        model.nodes.push(FBspNode::new());

        {
            let node = &mut model.nodes[node_index];
            // Set node properties.
            node.surface_index = surface_index;
            node.node_flags = node_flags;
            node.render_bound = None;
            node.collision_bound = None;
            node.plane = FPlane::new_from_origin_and_normal(ed_poly.vertices.first().unwrap(), &ed_poly.normal);
            node.vertex_pool_index = model.vertices.len();
            model.vertices.extend(iter::repeat_with(|| FVert::new()).take(ed_poly.vertices.len()));
            node.front_node_index = None;
            node.back_node_index = None;
            node.plane_index = None;
        }

        // TODO: get_many_mut will fail here in the Root case since node_index and parent_node_index are the same.
        // Tell transaction tracking system that parent is about to be modified.
        {
            match node_place {
                ENodePlace::Root => {
                    assert_eq!(0, node_index);
                    let node = &mut model.nodes[node_index];
                    node.leaf_indices[0] = None;
                    node.leaf_indices[1] = None;
                    node.zone[0] = 0;
                    node.zone[1] = 0;
                    node.zone_mask = !0u64;
                },
                _ => {
                    let [node, parent_node] = model.nodes.get_many_mut([node_index, parent_node_index]).unwrap();

                    node.zone_mask = parent_node.zone_mask;
                    
                    match node_place {
                        ENodePlace::Front | ENodePlace::Back => {
                            let zone_front_index = if node_place == ENodePlace::Front { 1usize } else { 0usize };
                            node.leaf_indices[0] = parent_node.leaf_indices[zone_front_index];
                            node.leaf_indices[1] = parent_node.leaf_indices[zone_front_index];
                            node.zone[0] = parent_node.zone[zone_front_index];
                            node.zone[1] = parent_node.zone[zone_front_index];
                        }
                        _ => {
                            let is_flipped_index = if node.plane.plane_dot(parent_node.plane.normal()) < 0.0 { 1usize } else { 0usize };
                            node.leaf_indices[0] = parent_node.leaf_indices[is_flipped_index];
                            node.leaf_indices[1] = parent_node.leaf_indices[1 - is_flipped_index];
                            node.zone[0] = parent_node.zone[is_flipped_index];
                            node.zone[1] = parent_node.zone[1 - is_flipped_index];
                        }
                    }

                    // Link parent to this node.
                    match node_place {
                        ENodePlace::Front => { parent_node.front_node_index = Some(node_index); }
                        ENodePlace::Back => { parent_node.back_node_index = Some(node_index); }
                        ENodePlace::Plane => { parent_node.plane_index = Some(node_index); }
                        _ => {}
                    }
                }
            }
        }
            
        // Add all points to point table, merging nearly-overlapping polygon points
        // with other points in the poly to prevent criscrossing vertices from
        // being generated.
        
        // Must maintain Node->NumVertices on the fly so that bspAddPoint is always
        // called with the Bsp in a clean state.
        let mut points: ArrayVec<FVector, BSP_NODE_MAX_NODE_VERTICES> = ArrayVec::new();
        let n = ed_poly.vertices.len();
        let vertex_pool_range = {
            let node = &mut model.nodes[node_index];
            node.vertex_count = 0;
            node.vertex_pool_index..node.vertex_pool_index + n
        };
        for ed_poly_vertex in ed_poly.vertices.iter() {
            let vertex_index = model.bsp_add_point(*ed_poly_vertex, false);
            let vert_pool = &mut model.vertices[vertex_pool_range.clone()];
            let node = &mut model.nodes[node_index];
            if node.vertex_count == 0 || vert_pool[node.vertex_count - 1].vertex_index != vertex_index {
                points.push(*ed_poly_vertex);
                vert_pool[node.vertex_count].side_index = None;
                vert_pool[node.vertex_count].vertex_index = vertex_index;
                node.vertex_count += 1;
            }
        }

        let node = &mut model.nodes[node_index];
        let vert_pool = &mut model.vertices[vertex_pool_range.clone()];

        if node.vertex_count >= 2 && vert_pool[0].vertex_index == vert_pool[node.vertex_count - 1].vertex_index {
            node.vertex_count -= 1;
        }

        if node.vertex_count < 3 {
            // GErrors++;
            println!("bspAddNode: Infinitesimal polygon {} ({})", node.vertex_count, n);
            node.vertex_count = 0;
        }

        node.section_index = None;
        node.light_map_index = None;

        // Calculate a bounding sphere for this node.
        node.exclusive_sphere_bound = FSphere::new_from_points(points.as_slice());

        node_index
    }
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

fn add_world_to_brush_func(context: &mut RebuildContext, model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, _: ENodePlace) {
    // Get a mutable refernce from the Rc<UModel> and add the node.
    let g_model = Rc::get_mut(&mut context.g_model).unwrap();
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside => {
            // Only affect the world poly if it has been cut.
            if ed_poly.poly_flags.contains(EPolyFlags::EdCut) {
                // TODO: how do we know that g_last_coplanar is not None?
                bsp_add_node(g_model, context.g_last_coplanar.unwrap(), ENodePlace::Plane, EBspNodeFlags::IsNew, ed_poly);
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

fn subtract_world_to_brush_func(context: &mut RebuildContext, model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, _: ENodePlace) {
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside | EPolyNodeFilter::CospatialFacingIn => {
			// Only affect the world poly if it has been cut.
            if ed_poly.poly_flags.contains(EPolyFlags::EdCut) {
                bsp_add_node(model, context.g_last_coplanar.unwrap(), ENodePlace::Plane, EBspNodeFlags::IsNew, ed_poly);
            }
        }
        EPolyNodeFilter::Inside | EPolyNodeFilter::CoplanarInside | EPolyNodeFilter::CospatialFacingOut => {
			// Discard original poly.
            context.g_discarded += 1;
            if context.g_model.nodes[context.g_node].vertex_count > 0 {
                context.g_model.nodes[context.g_node].vertex_count = 0;
            }
        }
    }
}

fn intersect_world_with_brush_func(context: &mut RebuildContext, model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, _: ENodePlace) {
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside | EPolyNodeFilter::CospatialFacingIn => {
            // Ignore.

        }
        EPolyNodeFilter::Inside | EPolyNodeFilter::CoplanarInside | EPolyNodeFilter::CospatialFacingOut => {
            if ed_poly.fix() >= 3 {
                context.g_model.polys.push(ed_poly.clone());
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

#[derive(Clone, Copy, Debug, PartialEq)]
struct FCoplanarInfo {
    original_node: Option<usize>,
    back_node_index: Option<usize>,
    is_processing_back: bool,
    is_back_node_outside: bool,
    front_leaf_outside: bool,
}

impl Default for FCoplanarInfo {
    fn default() -> Self {
        FCoplanarInfo {
            original_node: None,
            back_node_index: None,
            is_processing_back: false,
            is_back_node_outside: false,
            front_leaf_outside: false,
        }
    }
}

/// Regular entry into FilterEdPoly (so higher-level callers don't have to
/// deal with unnecessary info). Filters starting at root.
fn bsp_filter_fpoly(filter_func: BspFilterFunc, model: &mut UModel, ed_poly: &mut FPoly) {
    let starting_coplanar_info = FCoplanarInfo::default();
     if model.nodes.is_empty() {
        // If Bsp is empty, process at root.
        let filter = if model.is_root_outside { EPolyNodeFilter::Outside } else { EPolyNodeFilter::Inside };
        filter_func(model, 0, ed_poly, filter, ENodePlace::Root);
     } else {
        // Filter through BSP.
        filter_ed_poly(filter_func, model, 0, ed_poly, starting_coplanar_info, model.is_root_outside)
     }
}

/// Handle a piece of a polygon that was filtered to a leaf.
fn filter_leaf(filter_func: BspFilterFunc, model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, mut coplanar_info: FCoplanarInfo, mut leaf_outside: bool, node_place: ENodePlace) {

    let mut done_filtering_back = false;

    if coplanar_info.original_node.is_none() {
		// Processing regular, non-coplanar polygons.
        let filter_type = if leaf_outside { EPolyNodeFilter::Outside } else { EPolyNodeFilter::Inside };

        filter_func(model, node_index, ed_poly, filter_type, node_place);
    } else if coplanar_info.is_processing_back {
		// Finished filtering polygon through tree in back of parent coplanar.
        done_filtering_back = true;
    } else {
        coplanar_info.front_leaf_outside = leaf_outside;

        match coplanar_info.back_node_index {
            None => {
                // Back tree is empty.
                leaf_outside = coplanar_info.is_back_node_outside;
                done_filtering_back = true;
            }
            Some(back_node_index) => {
                // Call FilterEdPoly to filter through the back.  This will result in
                // another call to FilterLeaf with iNode = leaf this falls into in the
                // back tree and EdPoly = the final EdPoly to insert.
                coplanar_info.is_processing_back = true;

                filter_ed_poly(filter_func, model, back_node_index, ed_poly, coplanar_info, coplanar_info.is_back_node_outside);
            }
        }
    }

    if done_filtering_back {
        let filter_type = match (leaf_outside, coplanar_info.front_leaf_outside) {
            (false, false) => EPolyNodeFilter::CoplanarInside,
            (true, true) => EPolyNodeFilter::CoplanarOutside,
            (false, true) => EPolyNodeFilter::CospatialFacingOut,
            (true, false) => EPolyNodeFilter::CospatialFacingIn,
        };
        filter_func(model, coplanar_info.original_node.unwrap(), ed_poly, filter_type, ENodePlace::Plane);
    }

}

// Filter an EdPoly through the Bsp recursively, calling FilterFunc
// for all chunks that fall into leaves.  FCoplanarInfo is used to
// handle the tricky case of double-recursion for polys that must be
// filtered through a node's front, then filtered through the node's back,
// in order to handle coplanar CSG properly.
fn filter_ed_poly(filter_func: BspFilterFunc, model: &mut UModel, mut node_index: usize, ed_poly: &mut FPoly, mut coplanar_info: FCoplanarInfo, mut outside: bool) {
    // FilterLoop:
    if ed_poly.vertices.len() > FPOLY_VERTEX_THRESHOLD {
		// Split EdPoly in half to prevent vertices from overflowing.
        match ed_poly.split_in_half() {
            Some(mut temp) => {
                filter_ed_poly(filter_func, model, node_index, &mut temp, coplanar_info, outside);
            },
            None => {}
        }
    }

    // Split em.
    let plane_base = model.points[model.vertices[model.nodes[node_index].vertex_pool_index].vertex_index];
    let plane_normal = model.vectors[model.surfaces[model.nodes[node_index].surface_index].normal_index];

    let mut our_front: Option<usize> =  None;
    let mut our_back: Option<usize> = None;
    let mut new_front_outside = false;
    let mut new_back_outside = false;

    // Process split results.
    match ed_poly.split_with_plane(plane_base, plane_normal, false) {
        ESplitType::Front => {
            // Front:
            let node = &mut model.nodes[node_index];
            outside = outside || node.is_csg(EBspNodeFlags::empty());

            match node.front_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, outside, ENodePlace::Front);
                }
                Some(front_node_index) => {
                    node_index = front_node_index;
                    // goto FilterLoop;
                }
            }
        }
        ESplitType::Back => {
            let node = &model.nodes[node_index];
            outside = outside && !node.is_csg(EBspNodeFlags::empty());

            match node.back_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, outside, ENodePlace::Back);
                }
                Some(back_node_index) => {
                    node_index = back_node_index;
                    // goto FilterLoop;
                }
            }
        }
        ESplitType::Coplanar => {
            if coplanar_info.original_node.is_some() {
                // This will happen once in a blue moon when a polygon is barely outside the
                // coplanar threshold and is split up into a new polygon that is
                // is barely inside the coplanar threshold.  To handle this, just classify
                // it as front and it will be handled propery.
                
                println!("FilterEdPoly: Encountered out-of-place coplanar");

                // goto Front;
            }

            coplanar_info.original_node = Some(node_index);
            coplanar_info.back_node_index = None;
            coplanar_info.is_processing_back = false;
            coplanar_info.is_back_node_outside = outside;
            
            new_front_outside = outside;

            // See whether Node's iFront or iBack points to the side of the tree on the front
            // of this polygon (will be as expected if this polygon is facing the same
            // way as first coplanar in link, otherwise opposite).
            let node = &model.nodes[node_index];
            if node.plane.normal().dot(ed_poly.normal) >= 0.0 {
                our_front = node.front_node_index;
                our_back = node.back_node_index;

                if node.is_csg(EBspNodeFlags::empty()) {
                    coplanar_info.is_back_node_outside = false;
                    new_front_outside = true;
                }
            } else {
                our_front = node.back_node_index;
                our_back = node.front_node_index;

                if node.is_csg(EBspNodeFlags::empty()) {
                    coplanar_info.is_back_node_outside = true;
                    new_front_outside = false;
                }
            }

		    // Process front and back.
            match (our_front, our_back) {
                (None, None) => {
                    // No front or back.``
                    coplanar_info.is_processing_back = true;
                    coplanar_info.front_leaf_outside = new_front_outside;
                    filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, coplanar_info.is_back_node_outside, ENodePlace::Plane);
                }
                (None, Some(back_node_index)) => {
			        // Back but no front.
                    coplanar_info.is_processing_back = true;
                    coplanar_info.back_node_index = our_back;
                    coplanar_info.front_leaf_outside = new_front_outside;
                    node_index = back_node_index;
                    outside = coplanar_info.is_back_node_outside;
                    // goto FilterLoop;
                }
                (Some(front_node_index), _) => {
			        // Has a front and maybe a back.
                    
                    // Set iOurBack up to process back on next call to FilterLeaf, and loop
                    // to process front.  Next call to FilterLeaf will set FrontLeafOutside.
                    coplanar_info.is_processing_back = false;

                    // May be a node or may be INDEX_NONE.
                    coplanar_info.back_node_index = our_back;

                    node_index = front_node_index;
                    outside = new_front_outside;
                    // goto FilterLoop;
                }
            }
        }
        ESplitType::Split(mut front_poly, mut back_poly) => {
            // Front half of split.
            if model.nodes[node_index].is_csg(EBspNodeFlags::empty()) {
                new_front_outside = true;
                new_back_outside = false;
            } else {
                new_front_outside = outside;
                new_back_outside = outside;
            }

            match model.nodes[node_index].front_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, &mut front_poly, coplanar_info, new_front_outside, ENodePlace::Front);
                }
                Some(front_node_index) => {
                    filter_ed_poly(filter_func, model, front_node_index, &mut front_poly, coplanar_info, new_front_outside);
                }
            }

		    // Back half of split.
            match model.nodes[node_index].back_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, &mut back_poly, coplanar_info, new_back_outside, ENodePlace::Back);
                }
                Some(back_node_index) => {
                    filter_ed_poly(filter_func, model, back_node_index, &mut back_poly, coplanar_info, new_back_outside);
                }
            }
        }
    }
}

fn add_brush_to_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside => {
            bsp_add_node(model, node_index, node_place, EBspNodeFlags::IsNew, ed_poly);
        },
        EPolyNodeFilter::CospatialFacingOut => {
            if !ed_poly.poly_flags.contains(EPolyFlags::Semisolid) {
                bsp_add_node(model, node_index, node_place, EBspNodeFlags::IsNew, ed_poly);
            }
        },
        _ => {}
    }
}

fn subtract_brush_from_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    match filter {
        EPolyNodeFilter::CoplanarInside | EPolyNodeFilter::Inside => {
            ed_poly.reverse();
            bsp_add_node(model, node_index, node_place, EBspNodeFlags::IsNew, ed_poly); // Add to Bsp back
            ed_poly.reverse();
        },
        _ => {}
    }
}

fn intersect_brush_with_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    !todo!();
}

fn deintersect_brush_with_world_func(model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, filter: EPolyNodeFilter, node_place: ENodePlace) {
    !todo!();
}

/// Merge all coplanar EdPolys in a model.  Not transactional.
/// Preserves (though reorders) iLinks.
fn bsp_merge_coplanars(model: &mut UModel, should_remap_links: bool, should_merge_disparate_textures: bool) {
    let original_num = model.polys.len();

    // Mark all polys as unprocessed.
    model.polys.iter_mut().for_each(|poly| poly.poly_flags.remove(EPolyFlags::EdProcessed));

    // Find matching coplanars and merge them.
    let mut poly_list: Vec<usize> = Vec::new();
    let mut n = 0;

    for i in 0..model.polys.len() {
        {
            let ed_poly = &mut model.polys[i];
            if ed_poly.vertices.is_empty() || ed_poly.poly_flags.contains(EPolyFlags::EdProcessed) {
                continue;
            }
            ed_poly.poly_flags |= EPolyFlags::EdProcessed;
        }

        let mut poly_count = 0;
        poly_list[poly_count] = i;
        poly_count += 1;

        for j in i + 1..model.polys.len() {
            let [ed_poly, other_poly] = model.polys.get_many_mut([i, j]).unwrap();
            if other_poly.link != ed_poly.link {
                continue;
            }
            let distance = (other_poly.vertices[0] - ed_poly.vertices[0]).dot(ed_poly.normal);
            // TODO: make this easier to understand what it's doing.
            let a = distance > -0.001 && distance < 0.001 && other_poly.normal.dot(ed_poly.normal) > 0.999;
            let b = should_merge_disparate_textures || (
                points_are_near(&other_poly.texture_u, &ed_poly.texture_u, THRESH_VECTORS_ARE_NEAR) &&
                points_are_near(&other_poly.texture_v, &ed_poly.texture_v, THRESH_VECTORS_ARE_NEAR)
            );
            if a && b {
                other_poly.poly_flags |= EPolyFlags::EdProcessed;
                poly_list[poly_count] = j;
                poly_count += 1;
            }
        }

        if poly_count > 1 {
            merge_coplanars(&mut model.polys, poly_list.as_slice());
            n += 1;
        }
    }

    println!("Found {} coplanar sets in {}", n, model.polys.len());

    // Get rid of empty EdPolys while remapping iLinks.
    let mut remap = vec![0usize; model.polys.len()];
    let mut j = 0;
    for i in 0..model.polys.len() {
        if !model.polys[i].vertices.is_empty() {
            remap.push(j);
            model.polys[j] = model.polys[i].clone();
            j += 1;
        }
    }
    model.polys.truncate(j);

    if should_remap_links {
        for poly in model.polys.iter_mut() {
            if poly.link.is_some() {
                poly.link = Some(remap[poly.link.unwrap()]);
            }
        }
    }

    println!("BspMergeCoplanars reduced {}->{}", original_num, model.polys.len());
}

/// Perform any CSG operation between the brush and the world.
pub fn bsp_brush_csg(actor_id: usize, model: &mut UModel, poly_flags: EPolyFlags, csg_operation: ECsgOper, should_build_bounds: bool, should_merge_polys: bool) {
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
    }

    if !model.nodes.is_empty() && (poly_flags & (EPolyFlags::NotSolid | EPolyFlags::Semisolid)).is_empty() {
		// Quickly build a Bsp for the brush, tending to minimize splits rather than balance
		// the tree.  We only need the cutting planes, though the entire Bsp struct (polys and
		// all) is built.
        bsp_build(&mut temp_model, EBspOptimization::Lame, 0, 70, true, 0);

        //GModel = Brush;
        temp_model.build_bound();

        let bounding_sphere = temp_model.bounding_sphere.clone();
        filter_world_through_brush(model, &mut temp_model, csg_operation, 0, &bounding_sphere);
    }

    match csg_operation {
        ECsgOper::Intersect | ECsgOper::Deintersect => {
            // Link polys obtained from the original brush.
            for i in num_polys_from_brush-1..0 {    // TODO: this may not be right
                let mut j = 0;
                while j < i {
                    let [dest_ed_poly, other] = brush.polys.get_many_mut([i, j]).unwrap();
                    if dest_ed_poly.link == other.link {
                        dest_ed_poly.link = Some(j);
                        break
                    }
                    j += 1;
                }
                if j >= i {
                    brush.polys[i].link = Some(i);
                }
            }

            // Link polys obtained from the world.
            for i in model.polys.len()-1..num_polys_from_brush {    // TODO: this may not be right
                let mut j = num_polys_from_brush;
                while j < i {
                    let [dest_ed_poly, other] = brush.polys.get_many_mut([i, j]).unwrap();
                    if dest_ed_poly.link == other.link {
                        dest_ed_poly.link = Some(j);
                        break
                    
                    }
                    j += 1;
                }
                if j >= i {
                    brush.polys[i].link = Some(i);
                }
            }

            brush.linked = true;

            // Detransform the obtained brush back into its original coordinate system.
            for (i, dest_ed_poly) in brush.polys.iter_mut().enumerate() {
                dest_ed_poly.transform(&uncoords, &location, &pre_pivot, orientation);
                dest_ed_poly.fix();
                dest_ed_poly.actor = None;
                dest_ed_poly.brush_poly_index = Some(i);
            }
        },
        ECsgOper::Add | ECsgOper::Subtract => {
            // Clean up nodes, reset node flags.
            bsp_cleanup(model);

            // Rebuild bounding volumes.
            if should_build_bounds {
                bsp_build_bounds(model);
            }
        }
    }

    // Release TempModel.
    temp_model.empty_model(true, true);

    // Merge coplanars if needed.
    match csg_operation {
        ECsgOper::Intersect | ECsgOper::Deintersect => {
            if should_merge_polys {
                bsp_merge_coplanars(brush, true, false);
            }
        },
        _ => {}
    }
}

fn bsp_cleanup(model: &mut UModel) {
    !todo!()
}

fn build_zone_masks(model: &mut UModel, node_index: usize) {
    !todo!()
}

const WORLD_MAX: f32 = 524288.0;     	/* Maximum size of the world */
const HALF_WORLD_MAX: f32 = 262144.0;	/* Half the maximum size of the world */
const HALF_WORLD_MAX1: f32 = 262143.0;	/* Half the maximum size of the world - 1*/

fn filter_bound(model: &mut UModel, parent_bound: Option<&mut FBox>, node_index: usize, polys: &[FPoly], is_outside: bool) {
    let node = &model.nodes[node_index];
    let surface = &model.surfaces[node.surface_index];
    let base = surface.plane.normal() * surface.plane.w;
    let normal = model.vectors[surface.normal_index];
    let bound = FBox::new_from_min_max(
        FVector::new(WORLD_MAX, WORLD_MAX, WORLD_MAX),
        FVector::new(-WORLD_MAX, -WORLD_MAX, -WORLD_MAX),
    );

    // let mut front_poly_indices = Vec::with_capacity(polys.len() * 2 + 16);
    // let mut back_poly_indices = Vec::with_capacity(polys.len() * 2 + 16);

    // !todo();

    // for (poly_index, poly) in polys.iter().enumerate() {
    //     match poly.split_with_plane(base, normal false) {
    //         ESplitType::Coplanar => {
    //             front_poly_indices.push()
    //         }
    //     }
    // }
}

/// Build bounding volumes for all Bsp nodes.  The bounding volume of the node
/// completely encloses the "outside" space occupied by the nodes.  Note that 
/// this is not the same as representing the bounding volume of all of the 
/// polygons within the node.
///
/// We start with a practically-infinite cube and filter it down the Bsp,
/// whittling it away until all of its convex volume fragments land in leaves.
fn bsp_build_bounds(model: &mut UModel) {
    if model.nodes.is_empty() {
        return;
    }

    build_zone_masks(model, 0);

    // TODO: extract this to a function once we determine it works.
    let polys = vec![
        FPoly::from_vertices(&[
            FVector::new(-WORLD_MAX, -WORLD_MAX, WORLD_MAX),
            FVector::new( WORLD_MAX, -WORLD_MAX, WORLD_MAX),
            FVector::new( WORLD_MAX,  WORLD_MAX, WORLD_MAX),
            FVector::new(-WORLD_MAX,  WORLD_MAX, WORLD_MAX),
        ]),
        FPoly::from_vertices(&[
            FVector::new(-WORLD_MAX,  WORLD_MAX, -WORLD_MAX),
            FVector::new( WORLD_MAX,  WORLD_MAX, -WORLD_MAX),
            FVector::new( WORLD_MAX, -WORLD_MAX, -WORLD_MAX),
            FVector::new(-WORLD_MAX, -WORLD_MAX, -WORLD_MAX),
        ]),
        FPoly::from_vertices(&[
            FVector::new(-WORLD_MAX,  WORLD_MAX, -WORLD_MAX),
            FVector::new(-WORLD_MAX,  WORLD_MAX,  WORLD_MAX),
            FVector::new( WORLD_MAX,  WORLD_MAX,  WORLD_MAX),
            FVector::new( WORLD_MAX,  WORLD_MAX, -WORLD_MAX),
        ]),
        FPoly::from_vertices(&[
            FVector::new( WORLD_MAX, -WORLD_MAX, -WORLD_MAX),
            FVector::new( WORLD_MAX, -WORLD_MAX,  WORLD_MAX),
            FVector::new(-WORLD_MAX, -WORLD_MAX,  WORLD_MAX),
            FVector::new(-WORLD_MAX, -WORLD_MAX, -WORLD_MAX),
        ]),
        FPoly::from_vertices(&[
            FVector::new( WORLD_MAX,  WORLD_MAX, -WORLD_MAX),
            FVector::new( WORLD_MAX,  WORLD_MAX,  WORLD_MAX),
            FVector::new( WORLD_MAX, -WORLD_MAX,  WORLD_MAX),
            FVector::new( WORLD_MAX, -WORLD_MAX, -WORLD_MAX),
        ]),
        FPoly::from_vertices(&[
            FVector::new(-WORLD_MAX, -WORLD_MAX, -WORLD_MAX),
            FVector::new(-WORLD_MAX, -WORLD_MAX,  WORLD_MAX),
            FVector::new(-WORLD_MAX,  WORLD_MAX,  WORLD_MAX),
            FVector::new(-WORLD_MAX,  WORLD_MAX, -WORLD_MAX),
        ]),
    ];

    model.bounds.clear();
    model.leaf_hulls.clear();

    for node in model.nodes.iter_mut() {
        node.render_bound = None;
        node.collision_bound = None;
    }

    filter_bound(model, None, 0, &polys, model.is_root_outside);

    model.bounds.shrink_to_fit();
}

/// Build Bsp from the editor polygon set (EdPolys) of a model.
///
/// Opt     = Bsp optimization, BSP_Lame (fast), BSP_Good (medium), BSP_Optimal (slow)
/// Balance = 0-100, 0=only worry about minimizing splits, 100=only balance tree.
fn bsp_build(model: &mut UModel, optimization: EBspOptimization, balance: u8, portal_bias: u8, rebuild_simple_polys: bool, node_index: usize) {
    let original_polys = model.polys.len();

	// Empty the model's tables.
    if rebuild_simple_polys {
		// Empty everything but polys.
        model.empty_model(true, false);
    } else {
		// Empty node vertices.
        for node in model.nodes.iter_mut() {
            node.vertex_count = 0;
        }

        // Refresh the Bsp.
        bsp_refresh(model, true);

        // Empty nodes.
        model.empty_model(false, false);
    }

    if !model.polys.is_empty() {
        let polys = &mut model.polys;
        let mut poly_indices = Vec::new();
        for (i, poly) in polys.iter().enumerate() {
            if !poly.vertices.is_empty() {
                poly_indices.push(i);
            }
        }
        split_poly_list(model, None, ENodePlace::Root, &poly_indices, optimization, balance, portal_bias, rebuild_simple_polys);
        
        // Now build the bounding boxes for all nodes.
        if !rebuild_simple_polys {
            // Remove unreferenced things.
            bsp_refresh(model, true);

            // Rebuild all bounding boxes.
            bsp_build_bounds(model);
        }
    }

    print!("bspBuild built {} convex polys into {} nodes", original_polys, model.nodes.len());
}

/// Convert a Bsp node to an EdPoly.  Returns number of vertices in Bsp
/// node (0 or 3-MAX_NODE_VERTICES).
fn bsp_node_to_fpoly(model: &UModel, node_index: usize) -> Option<FPoly> {
    let node = &model.nodes[node_index];
    let poly = model.surfaces[node.surface_index];
    let vert_pool = &model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];

    if vert_pool.len() < 3 {
        return None;
    }

    let mut ed_poly = FPoly::new();

    ed_poly.base = model.points[poly.base_point_index];
    ed_poly.normal = model.vectors[poly.normal_index];
    ed_poly.poly_flags = poly.poly_flags & !(EPolyFlags::EdCut | EPolyFlags::EdProcessed | EPolyFlags::Selected | EPolyFlags::Memorized);
    ed_poly.link = Some(node.surface_index);
    // ed_poly.material = poly.material;
    // ed_poly.actor = Some(poly.actor.clone());
    ed_poly.brush_poly_index = poly.brush_polygon_index;

    // ed_poly.item_name = if let Some(master_ed_poly) = poly_find_master(model, node.surface_index) {
    //     master_ed_poly.item_name.clone()
    // } else {
    //     None
    // };

    ed_poly.texture_u = model.vectors[poly.texture_u_index];
    ed_poly.texture_v = model.vectors[poly.texture_v_index];
    ed_poly.light_map_scale = poly.light_map_scale;

    ed_poly.vertices = vert_pool.iter().map(|vertex| model.points[vertex.vertex_index]).collect();

    ed_poly.remove_colinears();

    Some(ed_poly)
}

/// Filter all relevant world polys through the brush.
fn filter_world_through_brush(model: &mut UModel, brush: &mut UModel, csg_operation: ECsgOper, mut node_index: usize, brush_sphere: &FSphere) {
    loop {
        let node = &model.nodes[node_index];
        
        // Get the surface.
        let surface_index = node.surface_index;

        // Skip new nodes and their children, which are guaranteed new.
        if node.node_flags.contains(EBspNodeFlags::IsNew) {
            return;
        }

        // Sphere reject.
        let distance = node.plane.plane_dot(brush_sphere.origin);
        let (do_front, do_back) = {
            let distance = node.plane.plane_dot(brush_sphere.origin);
            (distance >= -brush_sphere.radius, distance <= brush_sphere.radius)
        };

		// Process only polys that aren't empty.
        if do_front && do_back {
            if let Some(mut temp_ed_poly) = bsp_node_to_fpoly(model, node_index) {
                let surface = &model.surfaces[surface_index];
                // temp_ed_poly.actor = surface.actor;
                temp_ed_poly.brush_poly_index = surface.brush_polygon_index;

                let filter_func = match csg_operation {
                    ECsgOper::Add => add_world_to_brush_func,
                    ECsgOper::Subtract => subtract_world_to_brush_func,
                    ECsgOper::Intersect => intersect_world_with_brush_func,
                    ECsgOper::Deintersect => deintersect_world_with_brush_func,
                };

                match csg_operation {
                    ECsgOper::Add | ECsgOper::Subtract => {
				        // Add and subtract work the same in this step.
                        // GNode       = iNode;
                        // GModel  	= Model;
                        // GDiscarded  = 0;
                        // GNumNodes	= Model->Nodes.Num();

                        // // Find last coplanar in chain.
                        // GLastCoplanar = iNode;
                        // while( Model->Nodes(GLastCoplanar).iPlane != INDEX_NONE )
                        //     GLastCoplanar = Model->Nodes(GLastCoplanar).iPlane;

				        // Do the filter operation.
                        bsp_filter_fpoly(filter_func, brush, &mut temp_ed_poly);

                        // if( GDiscarded == 0 )
                        // {
                        //     // Get rid of all the fragments we added.
                        //     Model->Nodes(GLastCoplanar).iPlane = INDEX_NONE;
                        //     Model->Nodes.Remove( GNumNodes, Model->Nodes.Num()-GNumNodes );
                        // }
                        // else
                        // {
                        //     // Tag original world poly for deletion; has been deleted or replaced by partial fragments.
                        //     if( GModel->Nodes(GNode).NumVertices )
                        //     {
                        //         GModel->Nodes.ModifyItem( GNode );
                        //         GModel->Nodes(GNode).NumVertices = 0;
                        //     }
                        // }
                    }
                    ECsgOper::Intersect | ECsgOper::Deintersect => {
                        bsp_filter_fpoly(filter_func, brush, &mut temp_ed_poly)
                    }
                }
            
            }
        }
    }
}


/// Pick a splitter poly then split a pool of polygons into front and back polygons and
/// recurse.
//
/// iParent = Parent Bsp node, or INDEX_NONE if this is the root node.
/// IsFront = 1 if this is the front node of iParent, 0 of back (undefined if iParent==INDEX_NONE)
pub fn split_poly_list(
    model: &mut UModel, 
    parent_node_index: Option<usize>, 
    node_place: ENodePlace, 
    poly_indices: &[usize],
    optimization: EBspOptimization, 
    balance: u8, 
    portal_bias: u8, 
    rebuild_simple_polys: bool
) {
	// To account for big EdPolys split up.
    let num_polys_to_alloc = poly_indices.len() + 8 + poly_indices.len() / 4;

    let mut front_poly_indices = Vec::with_capacity(num_polys_to_alloc);
    let mut back_poly_indices = Vec::with_capacity(num_polys_to_alloc);
    let split_poly_index = find_best_split_recursive(&model.polys, poly_indices, optimization, balance, portal_bias);

	// Add the splitter poly to the Bsp with either a new BspSurf or an existing one.
    if rebuild_simple_polys {
        if let Some(split_poly_index) = split_poly_index {
            let split_poly = &mut model.polys[split_poly_index];
            split_poly.link = None
        }
    }

    let split_poly = model.polys[split_poly_index.unwrap()].clone(); // TODO: if the spliy poly function is always assumed to return a valid index, just don't wrap it in an optional.
    let our_node_index = bsp_add_node(model, parent_node_index.unwrap(), node_place, EBspNodeFlags::empty(), &split_poly);
    let mut plane_node_index = our_node_index;

	// Now divide all polygons in the pool into (A) polygons that are
	// in front of Poly, and (B) polygons that are in back of Poly.
	// Coplanar polys are inserted immediately, before recursing.

	// If any polygons are split by Poly, we ignrore the original poly,
	// split it into two polys, and add two new polys to the pool.
    for poly_index in poly_indices {
        if *poly_index == split_poly_index.unwrap() {
            continue;
        }

        let [ed_poly, split_poly] = model.polys.get_many_mut([*poly_index, split_poly_index.unwrap()]).unwrap();
        let split_result = ed_poly.split_with_plane(split_poly.vertices[0], split_poly.normal, false);
        let ed_poly_copy = ed_poly.clone();

        // To dodge the borrow-checker, for the coplanar and split cases, we need to defer the
        // addition of the node and polys until we can drop the borrow of the ed and split polys.
        match split_result {
            ESplitType::Coplanar => {
                if rebuild_simple_polys {
                    ed_poly.link = Some(model.surfaces.len() - 1);
                    plane_node_index = bsp_add_node(model, plane_node_index, ENodePlace::Plane, EBspNodeFlags::empty(), &ed_poly_copy);
                }
            }
            ESplitType::Front => {
                front_poly_indices.push(*poly_index);
            }
            ESplitType::Back => {
                back_poly_indices.push(*poly_index);
            }
            ESplitType::Split(mut front_poly, mut back_poly) => {
				// Create front & back nodes.
				// If newly-split polygons have too many vertices, break them up in half.
                if front_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
                    let split_poly = front_poly.split_in_half().unwrap();
                    front_poly_indices.extend([front_poly_indices.len(), front_poly_indices.len() + 1]);
                    model.polys.extend([front_poly, split_poly]);
                } else {
                    front_poly_indices.push(front_poly_indices.len());
                    model.polys.push(front_poly);
                }

                if back_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
                    let split_poly = back_poly.split_in_half().unwrap();
                    back_poly_indices.extend([back_poly_indices.len(), back_poly_indices.len() + 1]);
                    model.polys.extend([back_poly, split_poly]);
                } else {
                    back_poly_indices.push(back_poly_indices.len());
                    model.polys.push(back_poly);
                }
            }
        }
    }

    if !front_poly_indices.is_empty() {
        split_poly_list(model, Some(our_node_index), ENodePlace::Front, &front_poly_indices, optimization, balance, portal_bias, rebuild_simple_polys);
    }

    if !back_poly_indices.is_empty() {
        split_poly_list(model, Some(our_node_index), ENodePlace::Back, &back_poly_indices, optimization, balance, portal_bias, rebuild_simple_polys);
    }

}

pub fn find_best_split(polys: &[FPoly], optimization: EBspOptimization, balance: u8, portal_bias: u8) -> Option<usize> {
    let poly_indices = (0..polys.len()).collect::<Vec<usize>>();
    find_best_split_recursive(polys, &poly_indices, optimization, balance, portal_bias)
}

fn find_best_split_recursive(polys: &[FPoly], poly_indices: &[usize], optimization: EBspOptimization, balance: u8, portal_bias: u8) -> Option<usize> {
    let portal_bias = portal_bias as f32 / 100.0;

	// No need to test if only one poly.
    if poly_indices.len() == 1 {
        return Some(0usize)
    }

    let step = match optimization {
        EBspOptimization::Optimal => 1,
        EBspOptimization::Lame => 1.max(polys.len() / 20),
        EBspOptimization::Good => 1.max(polys.len() / 4),
    };

    // See if there are any non-semisolid polygons here.
    let all_semi_solids = polys.iter().all(|poly| poly.poly_flags.contains(EPolyFlags::NoAddToBSP));

	// Search through all polygons in the pool and find:
	// A. The number of splits each poly would make.
	// B. The number of front and back nodes the polygon would create.
	// C. Number of coplanars.
    let mut best_poly_index: Option<usize> = None;
    let mut best_score = 0.0f32;

    // Iterate over the polys with the given step.
    for i in (0..poly_indices.len()).step_by(step) {
        let mut poly_index = poly_indices[i];

        if !all_semi_solids {
            loop {
                if poly_index >= (i + step) || 
                   poly_index >= polys.len() || 
                   !polys[poly_index].poly_flags.contains(EPolyFlags::NoAddToBSP) ||
                   polys[poly_index].poly_flags.contains(EPolyFlags::Portal)
                {
                    break
                }
                
                poly_index += 1;
            }
        }

        if poly_index >= (i + step) || poly_index >= polys.len() {
            continue;
        }

        let poly = &polys[poly_index];
        let mut splits = 0;
        let mut front = 0;
        let mut back = 0;
        let mut coplanars = 0;

        for j in 0..poly_indices.len() {
            let other_poly = &polys[poly_indices[j]];
            if i == j {
                continue;
            }
            match other_poly.split_with_plane_fast(&poly.vertices[0], &poly.normal) {
                ESplitType::Coplanar => {
                    coplanars += 1;
                }
                ESplitType::Front => {
                    front += 1;
                },
                ESplitType::Back => {
                    back += 1;
                },
                ESplitType::Split(_, _) => {
					// Disfavor splitting polys that are zone portals.
                    if !other_poly.poly_flags.contains(EPolyFlags::Portal) {
                        splits += 1;
                    } else {
                        splits += 16;
                    }
                },
            }

            // Score optimization: minimize cuts vs. balance tree (as specified in BSP Rebuilder dialog)
            let score = {
                let mut score = (100.0 - balance as f32) * splits as f32 + (balance as f32) * ((front - back) as f32).abs();
                if poly.poly_flags.contains(EPolyFlags::Portal) {
                    // PortalBias enables level designers to control the effect of Portals on the BSP.
                    // This effect can range from 0.0 (ignore portals), to 1.0 (portals cut everything).
                    //
                    // In builds prior to this (since the 221 build dating back to 1/31/1999) the bias
                    // has been 1.0 causing the portals to cut the BSP in ways that will potentially
                    // degrade level performance, and increase the BSP complexity.
                    // 
                    // By setting the bias to a value between 0.3 and 0.7 the positive effects of 
                    // the portals are preserved without giving them unreasonable priority in the BSP.
                    //
                    // Portals should be weighted high enough in the BSP to separate major parts of the
                    // level from each other (pushing entire rooms down the branches of the BSP), but
                    // should not be so high that portals cut through adjacent geometry in a way that
                    // increases complexity of the room being (typically, accidentally) cut.
                    score -= (100.0 - balance as f32) * splits as f32 * portal_bias;
                }
                score
            };

            println!("score: {}", score);

            if score < best_score || best_poly_index.is_none() {
                best_poly_index = Some(poly_index);
                best_score = score;
            }
        }
    }

    best_poly_index
}

/// If the Bsp's point and vector tables are nearly full, reorder them and delete
/// unused ones:
pub fn bsp_refresh(model: &mut UModel, no_remap_surfs: bool) {
    // Removed unreferenced Bsp surfs.

    let mut node_ref = vec![None; model.nodes.len()];
    let mut poly_ref = vec![None; model.polys.len()];

    if !model.nodes.is_empty() {
        tag_referenced_nodes(model, &mut node_ref, &mut poly_ref);
    }

    if no_remap_surfs {
        poly_ref.fill(Some(0));
    }

    // Remap Bsp nodes and surfs.
    let mut n = 0;
    for i in 0..model.surfaces.len() {
        if poly_ref[i].is_some() {
            model.surfaces.swap(n, i);
            poly_ref[i] = Some(n);
            n += 1;
        }
    }
    model.surfaces.truncate(n);

    let mut n = 0;
    for i in 0..model.nodes.len() {
        model.nodes.swap(n, i);
        node_ref[i] = Some(n);
        n += 1;
    }
    model.nodes.truncate(n);

    // Update Bsp nodes.
    for node in model.nodes.iter_mut() {
        node.surface_index = poly_ref[node.surface_index].unwrap();
        if let Some(front_node_index) = node.front_node_index {
            node.front_node_index = node_ref[front_node_index];
        }
        if let Some(back_node_index) = node.back_node_index {
            node.back_node_index = node_ref[back_node_index];
        }
        if let Some(plane_node_index) = node.plane_index {
            node.plane_index = node_ref[plane_node_index];
        }
    }

	// Remove unreferenced points and vectors.
    let mut vector_ref = vec![None; model.vectors.len()];
    let mut point_ref = vec![None; model.points.len()];

    // Check Bsp surfs.
    for surf in model.surfaces.iter_mut() {
        vector_ref[surf.normal_index] = Some(0);
        vector_ref[surf.texture_u_index] = Some(0);
        vector_ref[surf.texture_v_index] = Some(0);
        point_ref[surf.base_point_index] = Some(0);
    }

	// Check Bsp nodes.
    for node in model.nodes.iter_mut() {
        let vert_pool = &model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];
        for vert in vert_pool {
            point_ref[vert.vertex_index] = Some(0);
        }
    }
    
	// Remap points.
    let mut n = 0;
    for i in 0..model.points.len() {
        if point_ref[i].is_some() {
            model.points.swap(n, i);
            point_ref[i] = Some(n);
            n += 1;
        }
    }
    model.points.truncate(n);

    // Remap vectors.
    let mut n = 0;
    for i in 0..model.vectors.len() {
        if vector_ref[i].is_some() {
            model.vectors.swap(n, i);
            vector_ref[i] = Some(n);
            n += 1;
        }
    }
    model.vectors.truncate(n);

    // Update Bsp surfs.
    for surf in model.surfaces.iter_mut() {
        surf.normal_index = vector_ref[surf.normal_index].unwrap();
        surf.texture_u_index = vector_ref[surf.texture_u_index].unwrap();
        surf.texture_v_index = vector_ref[surf.texture_v_index].unwrap();
        surf.base_point_index = point_ref[surf.base_point_index].unwrap();
    }

    // Update Bsp nodes.
    for node in model.nodes.iter_mut() {
        let vert_pool = &mut model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];
        for vert in vert_pool {
            vert.vertex_index = point_ref[vert.vertex_index].unwrap();
        }
    }

    model.shrink_model()
}

fn tag_referenced_nodes(model: &UModel, node_ref: &mut [Option<usize>], poly_ref: &mut [Option<usize>]) {
    let mut node_index_stack: Vec<usize> = vec![0usize];

    while let Some(node_index) = node_index_stack.pop() {
        let node = &model.nodes[node_index];
        node_ref[node_index] = Some(0);
        poly_ref[node.surface_index] = Some(0);

        if let Some(front_node_index) = node.front_node_index {
            node_index_stack.push(front_node_index);
        }
        if let Some(back_node_index) = node.back_node_index {
            node_index_stack.push(back_node_index);
        }
        if let Some(plane_node_index) = node.plane_index {
            node_index_stack.push(plane_node_index);
        }
    }
}