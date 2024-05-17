use cgmath::{dot, InnerSpace};

use crate::box_::FBox;
use crate::math::{point_plane_distance, points_are_near, points_are_same, FPlane, FVector, THRESH_VECTORS_ARE_NEAR};
use crate::fpoly::{EPolyFlags, ESplitType, FPoly, RemoveColinearsResult, FPOLY_MAX_VERTICES, FPOLY_VERTEX_THRESHOLD};
use crate::model::{EBspNodeFlags, FBspNode, FBspSurf, FVert, UModel, BSP_NODE_MAX_NODE_VERTICES};
use crate::brush::ABrush;
use crate::sphere::FSphere;
use arrayvec::ArrayVec;
use std::borrow::Borrow;
use std::iter;


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
pub fn bsp_add_node(model: &mut UModel, mut parent_node_index: Option<usize>, node_place: ENodePlace, mut node_flags: EBspNodeFlags, ed_poly: &FPoly) -> usize {
    if node_place == ENodePlace::Plane {
		// Make sure coplanars are added at the end of the coplanar list so that 
		// we don't insert NF_IsNew nodes with non NF_IsNew coplanar children.
        while let Some(plane_index) = model.nodes[parent_node_index.unwrap()].plane_index {
            parent_node_index = Some(plane_index)
        }
    }
    
    // TODO: this is actually the NONE case, we set the link to None when we want it inserted at the end.
    let (surface, surface_index) = if ed_poly.link == Some(model.surfaces.len()) {
        let mut surf = FBspSurf::default();

        // This node has a new polygon being added by bspBrushCSG; must set its properties here.
        surf.base_point_index = model.bsp_add_point(ed_poly.base, true);
        surf.normal_index = model.bsp_add_vector(ed_poly.normal, true);
        surf.texture_u_index = model.bsp_add_vector(ed_poly.texture_u, false);
        surf.texture_v_index = model.bsp_add_vector(ed_poly.texture_v, false);
        surf.material_index = ed_poly.material_index;
        surf.brush_id = ed_poly.brush_id.unwrap();
        surf.poly_flags = ed_poly.poly_flags & !EPolyFlags::NoAddToBSP;
        surf.light_map_scale = ed_poly.light_map_scale;
        surf.brush_polygon_index = ed_poly.brush_poly_index;
        surf.plane = FPlane::new_from_origin_and_normal(ed_poly.vertices.first().unwrap(), &ed_poly.normal);
        
        let surface_index = model.surfaces.len();
        model.surfaces.push(surf);

        (&model.surfaces[surface_index], surface_index)
    } else {
        assert!(ed_poly.link.is_some());
        let link  = ed_poly.link.unwrap();
        assert!(link < model.surfaces.len());
        (&model.surfaces[link], link)
    };

	// Set NodeFlags.
    if surface.poly_flags.intersects(EPolyFlags::NotSolid) {
        node_flags |= EBspNodeFlags::NotCsg;
    }
    if surface.poly_flags.intersects(EPolyFlags::Invisible | EPolyFlags::Portal) {
        node_flags |= EBspNodeFlags::NotVisBlocking;
    }
    if surface.poly_flags.intersects(EPolyFlags::Masked) {
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
        bsp_add_node(model, Some(node_index), ENodePlace::Plane, node_flags, &mut ed_poly2);

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
                    let [node, parent_node] = model.nodes.get_many_mut([node_index, parent_node_index.unwrap()]).unwrap();

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

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ECsgOper {
    Add,
    Subtract,
}

type BspFilterFunc = fn(model: &mut UModel, node_index: usize, ed_poly: &FPoly, filter: EPolyNodeFilter, node_place: ENodePlace, filter_context: &mut FilterContext);

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
pub fn bsp_filter_fpoly(filter_func: BspFilterFunc, model: &mut UModel, ed_poly: &mut FPoly, filter_context: &mut FilterContext) {
     if model.nodes.is_empty() {
        // If Bsp is empty, process at root.
        let filter = match model.is_root_outside {
            true => EPolyNodeFilter::Outside,
            false => EPolyNodeFilter::Inside,
        };
        filter_func(model, 0, ed_poly, filter, ENodePlace::Root, filter_context);
     } else {
        let starting_coplanar_info = FCoplanarInfo::default();
        // Filter through BSP.
        filter_ed_poly(filter_func, model, 0, ed_poly, starting_coplanar_info, model.is_root_outside, filter_context)
     }
}

/// Handle a piece of a polygon that was filtered to a leaf.
fn filter_leaf(filter_func: BspFilterFunc, model: &mut UModel, node_index: usize, ed_poly: &mut FPoly, mut coplanar_info: FCoplanarInfo, mut leaf_outside: bool, node_place: ENodePlace, filter_context: &mut FilterContext) {
    let mut done_filtering_back = false;

    if coplanar_info.original_node.is_none() {
		// Processing regular, non-coplanar polygons.
        let filter_type = if leaf_outside { EPolyNodeFilter::Outside } else { EPolyNodeFilter::Inside };

        filter_func(model, node_index, ed_poly, filter_type, node_place, filter_context);
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

                filter_ed_poly(filter_func, model, back_node_index, ed_poly, coplanar_info, coplanar_info.is_back_node_outside, filter_context);
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
        filter_func(model, coplanar_info.original_node.unwrap(), ed_poly, filter_type, ENodePlace::Plane, filter_context);
    }

}

// Filter an EdPoly through the Bsp recursively, calling FilterFunc
// for all chunks that fall into leaves.  FCoplanarInfo is used to
// handle the tricky case of double-recursion for polys that must be
// filtered through a node's front, then filtered through the node's back,
// in order to handle coplanar CSG properly.
fn filter_ed_poly(filter_func: BspFilterFunc, model: &mut UModel, mut node_index: usize, ed_poly: &mut FPoly, mut coplanar_info: FCoplanarInfo, mut outside: bool, filter_context: &mut FilterContext) {
    loop {
        if ed_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
            // Split EdPoly in half to prevent vertices from overflowing.
            if let Some(mut temp) = ed_poly.split_in_half() {
                filter_ed_poly(filter_func, model, node_index, &mut temp, coplanar_info, outside, filter_context);
            }
        }
    
        // Split em.
        let plane_base = model.points[model.vertices[model.nodes[node_index].vertex_pool_index].vertex_index];
        let plane_normal = model.vectors[model.surfaces[model.nodes[node_index].surface_index].normal_index];
        let mut do_front = false;
    
        // Process split results.
        let split_result = ed_poly.split_with_plane(plane_base, plane_normal, false);

        match split_result {
            ESplitType::Front => {
                do_front = true;
            }
            ESplitType::Back => {
                let node = &model.nodes[node_index];
                outside = outside && !node.is_csg(EBspNodeFlags::empty());
    
                match node.back_node_index {
                    None => {
                        filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, outside, ENodePlace::Back, filter_context);
                        break;
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
                    do_front = true;
                } else {
                    coplanar_info.original_node = Some(node_index);
                    coplanar_info.back_node_index = None;
                    coplanar_info.is_processing_back = false;
                    coplanar_info.is_back_node_outside = outside;
                    
                    let mut new_front_outside = outside;
        
                    // See whether Node's iFront or iBack points to the side of the tree on the front
                    // of this polygon (will be as expected if this polygon is facing the same
                    // way as first coplanar in link, otherwise opposite).
                    let node = &model.nodes[node_index];
        
                    let (our_front, our_back) = if node.plane.normal().dot(ed_poly.normal) >= 0.0 {
                        if node.is_csg(EBspNodeFlags::empty()) {
                            coplanar_info.is_back_node_outside = false;
                            new_front_outside = true;
                        }
                        (node.front_node_index, node.back_node_index)
                    } else {
                        if node.is_csg(EBspNodeFlags::empty()) {
                            coplanar_info.is_back_node_outside = true;
                            new_front_outside = false;
                        }
                        (node.back_node_index, node.front_node_index)
                    };

                    // Process front and back.
                    match (our_front, our_back) {
                        (None, None) => {
                            // No front or back.
                            coplanar_info.is_processing_back = true;
                            coplanar_info.front_leaf_outside = new_front_outside;
                            filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, coplanar_info.is_back_node_outside, ENodePlace::Plane, filter_context);
                            break;
                        }
                        (None, Some(our_back)) => {
                            // Back but no front.
                            coplanar_info.is_processing_back = true;
                            coplanar_info.back_node_index = Some(our_back);
                            coplanar_info.front_leaf_outside = new_front_outside;
                            node_index = our_back;
                            outside = coplanar_info.is_back_node_outside;
                        }
                        (Some(our_front), _) => {
                            // Has a front and maybe a back.
                            
                            // Set iOurBack up to process back on next call to FilterLeaf, and loop
                            // to process front.  Next call to FilterLeaf will set FrontLeafOutside.
                            coplanar_info.is_processing_back = false;
        
                            // May be a node or may be INDEX_NONE.
                            coplanar_info.back_node_index = our_back;
        
                            node_index = our_front;
                            outside = new_front_outside;
                        }
                    }
                }                
            }
            ESplitType::Split(mut front_poly, mut back_poly) => {
		        // Front half of split.
                let (new_front_outside, new_back_outside) = {
                    if model.nodes[node_index].is_csg(EBspNodeFlags::empty()) {
                        (true, false)
                    } else {
                        (outside, outside)
                    }
                };
    
                match model.nodes[node_index].front_node_index {
                    None => {
                        filter_leaf(filter_func, model, node_index, &mut front_poly, coplanar_info, new_front_outside, ENodePlace::Front, filter_context);
                    }
                    Some(front_node_index) => {
                        filter_ed_poly(filter_func, model, front_node_index, &mut front_poly, coplanar_info, new_front_outside, filter_context);
                    }
                }
    
                // Back half of split.
                match model.nodes[node_index].back_node_index {
                    None => {
                        filter_leaf(filter_func, model, node_index, &mut back_poly, coplanar_info, new_back_outside, ENodePlace::Back, filter_context);
                    }
                    Some(back_node_index) => {
                        filter_ed_poly(filter_func, model, back_node_index, &mut back_poly, coplanar_info, new_back_outside, filter_context);
                    }
                }

                break;
            }
        }

        if do_front {
            let node = &mut model.nodes[node_index];
            outside = outside || node.is_csg(EBspNodeFlags::empty());

            match node.front_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, outside, ENodePlace::Front, filter_context);
                    break;
                }
                Some(front_node_index) => {
                    node_index = front_node_index;
                }
            }
        }
    }
}

fn add_brush_to_world_func(model: &mut UModel, node_index: usize, ed_poly: &FPoly, filter: EPolyNodeFilter, node_place: ENodePlace, _: &mut FilterContext) {
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside => {
            bsp_add_node(model, Some(node_index), node_place, EBspNodeFlags::IsNew, ed_poly);
        },
        EPolyNodeFilter::CospatialFacingOut => {
            if !ed_poly.poly_flags.intersects(EPolyFlags::Semisolid) {
                bsp_add_node(model, Some(node_index), node_place, EBspNodeFlags::IsNew, ed_poly);
            }
        },
        _ => {}
    }
}

fn subtract_brush_from_world_func(model: &mut UModel, node_index: usize, ed_poly: &FPoly, filter: EPolyNodeFilter, node_place: ENodePlace, _: &mut FilterContext) {
    match filter {
        EPolyNodeFilter::CoplanarInside | EPolyNodeFilter::Inside => {
            bsp_add_node(model, Some(node_index), node_place, EBspNodeFlags::IsNew, &ed_poly.reversed()); // Add to Bsp back
        },
        _ => {}
    }
}

/// Merge all coplanar EdPolys in a model.  Not transactional.
/// Preserves (though reorders) iLinks.
pub fn bsp_merge_coplanars(model: &mut UModel, should_remap_links: bool, should_merge_disparate_textures: bool) {
    let original_num = model.polys.len();

    // Mark all polys as unprocessed.
    model.polys.iter_mut().for_each(|poly| poly.poly_flags.remove(EPolyFlags::EdProcessed));

    // Find matching coplanars and merge them.
    let mut poly_list: Vec<usize> = vec![];
    let mut n = 0;

    for i in 0..model.polys.len() {
        {
            let ed_poly = &mut model.polys[i];
            if ed_poly.vertices.is_empty() || ed_poly.poly_flags.contains(EPolyFlags::EdProcessed) {
                continue;
            }
            ed_poly.poly_flags |= EPolyFlags::EdProcessed;
        }

        poly_list.clear();
        poly_list.push(i);

        for j in i + 1..model.polys.len() {
            let [ed_poly, other_poly] = model.polys.get_many_mut([i, j]).unwrap();
            if other_poly.link != ed_poly.link {
                continue;
            }

            let distance = (other_poly.vertices[0] - ed_poly.vertices[0]).dot(ed_poly.normal);
            // TODO: make this easier to understand what it's doing.
            let a = distance > -0.001 && distance < 0.001 && other_poly.normal.dot(ed_poly.normal) > 0.9999;
            let b = should_merge_disparate_textures || (
                points_are_near(&other_poly.texture_u, &ed_poly.texture_u, THRESH_VECTORS_ARE_NEAR) &&
                points_are_near(&other_poly.texture_v, &ed_poly.texture_v, THRESH_VECTORS_ARE_NEAR)
            );
            if a && b {
                other_poly.poly_flags |= EPolyFlags::EdProcessed;
                poly_list.push(j);
            }
        }

        if poly_list.len() > 1 {
            merge_coplanars(&mut model.polys, poly_list.as_slice());
            n += 1;
        }
    }

    println!("Found {} coplanar sets in {}", n, model.polys.len());

    // Get rid of empty EdPolys while remapping iLinks.
    let mut j = 0;
    let mut remap = vec![0usize; model.polys.len()];
    for i in 0..model.polys.len() {
        if !model.polys[i].vertices.is_empty() {
            remap[i] = j;
            model.polys[j] = model.polys[i].clone();
            j += 1;
        }
    }
    model.polys.truncate(j);

    if should_remap_links {
        for poly in model.polys.iter_mut() {
            if let Some(poly_link) = poly.link {
                poly.link = Some(remap[poly_link]);
            }
        }
    }

    println!("BspMergeCoplanars reduced {}->{}", original_num, model.polys.len());
}


/// Validate a brush, and set iLinks on all EdPolys to index of the
/// first identical EdPoly in the list, or its index if it's the first.
/// Not transactional.
pub fn bsp_validate_brush(brush: &mut UModel, force_validate: bool) {
    if force_validate || !brush.linked {
        brush.linked = true;

        for (i, poly) in brush.polys.iter_mut().enumerate() {
            poly.link = Some(i);
        }

        let mut n = 0;
        for i in 0..brush.polys.len() {
            if brush.polys[i].link == Some(i) {
                // use get_many_mut
                for j in i + 1..brush.polys.len() {
                    let [ed_poly, other_poly] = brush.polys.get_many_mut([i, j]).unwrap();

                    if other_poly.link == Some(j) &&
                        other_poly.material_index == ed_poly.material_index &&
                        other_poly.texture_u == ed_poly.texture_u &&
                        other_poly.texture_v == ed_poly.texture_v &&
                        other_poly.poly_flags == ed_poly.poly_flags &&
                        other_poly.normal.dot(ed_poly.normal) > 0.999
                    {
                        let distance = point_plane_distance(&other_poly.vertices[0], &ed_poly.vertices[0], &ed_poly.normal);
                        if distance > -0.001 && distance < 0.001 {
                            other_poly.link = Some(i);
                            n += 1;
                        }
                    }
                }
            }
        }
    }

    brush.build_bound();
}

fn bsp_node_to_fpoly(model: &UModel, node_index: usize) -> Option<FPoly> {
    let node = &model.nodes[node_index];
    let poly = &model.surfaces[node.surface_index];
    let vert_pool = &model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];

    // BDK: Early out if the poly is degenerate. We do another check after removing colinear vertices at the end.
    if node.vertex_count < 3 {
        return None
    }

    let mut ed_poly = FPoly::new();

    ed_poly.base = model.points[poly.base_point_index];
    ed_poly.normal = model.vectors[poly.normal_index];
    ed_poly.poly_flags &= !(EPolyFlags::EdCut | EPolyFlags::EdProcessed | EPolyFlags::Selected | EPolyFlags::Memorized);
    ed_poly.link = Some(node.surface_index);
    ed_poly.material_index = poly.material_index;
    ed_poly.brush_id = Some(poly.brush_id);
    ed_poly.brush_poly_index = poly.brush_polygon_index;
    ed_poly.texture_u = model.vectors[poly.texture_u_index];
    ed_poly.texture_v = model.vectors[poly.texture_v_index];
    ed_poly.light_map_scale = poly.light_map_scale;

    for vert in vert_pool {
        ed_poly.vertices.push(model.points[vert.vertex_index]);
    }

    ed_poly.remove_colinears();
    
    if ed_poly.vertices.len() < 3 {
        return None
    }

    Some(ed_poly)
}

fn add_world_to_brush_func(_: &mut UModel, _: usize, ed_poly: &FPoly, filter: EPolyNodeFilter, _: ENodePlace, filter_context: &mut FilterContext) {
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside => {
            // Only affect the world poly if it has been cut.
            if ed_poly.poly_flags.contains(EPolyFlags::EdCut) {
                bsp_add_node(filter_context.model, Some(filter_context.last_coplanar_node_index), ENodePlace::Plane, EBspNodeFlags::IsNew, ed_poly);
            }
        }
        EPolyNodeFilter::Inside | EPolyNodeFilter::CoplanarInside | EPolyNodeFilter::CospatialFacingIn | EPolyNodeFilter::CospatialFacingOut => {
            // Discard original poly.
            filter_context.discarded += 1;
            filter_context.model.nodes[filter_context.node_index].vertex_count = 0;
        }
    }
}

fn subtract_world_to_brush_func(_: &mut UModel, _: usize, ed_poly: &FPoly, filter: EPolyNodeFilter, _: ENodePlace, filter_context: &mut FilterContext) {
    match filter {
        EPolyNodeFilter::Outside | EPolyNodeFilter::CoplanarOutside | EPolyNodeFilter::CospatialFacingIn => {
			// Only affect the world poly if it has been cut.
            if ed_poly.poly_flags.contains(EPolyFlags::EdCut) {
                bsp_add_node(filter_context.model, Some(filter_context.last_coplanar_node_index), ENodePlace::Plane, EBspNodeFlags::IsNew, ed_poly);
            }
        }
        EPolyNodeFilter::Inside | EPolyNodeFilter::CoplanarInside | EPolyNodeFilter::CospatialFacingOut => {
            // Discard original poly.
            filter_context.discarded += 1;
            filter_context.model.nodes[filter_context.node_index].vertex_count = 0;
        }
    }
}

pub struct FilterContext<'a> {
    discarded: usize,
    node_index: usize,
    last_coplanar_node_index: usize,
    model: &'a mut UModel,
}

/// Filter all relevant world polys through the brush.
fn filter_world_through_brush(model: &mut UModel, brush: &mut UModel, csg_operation: ECsgOper, mut node_index: usize, brush_sphere: FSphere) {
    // Loop through all coplanars.
    loop {
        // Get surface.
        let surface_index = model.nodes[node_index].surface_index;

        // Skip new nodes and their children, which are guaranteeed new.
        if model.nodes[node_index].node_flags.contains(EBspNodeFlags::IsNew) {
            return
        }

        // Sphere reject.
        let (do_front, do_back) = {
            let distance = model.nodes[node_index].plane.plane_dot(brush_sphere.origin);
            (distance >= -brush_sphere.radius, distance <= brush_sphere.radius)
        };

        // Process only polys that aren't empty.
        if do_front && do_back {
            if let Some(mut temp_ed_poly) = bsp_node_to_fpoly(model, node_index) {
                temp_ed_poly.brush_id = Some(model.surfaces[surface_index].brush_id);
                temp_ed_poly.brush_poly_index = model.surfaces[surface_index].brush_polygon_index;

                // Find last coplanar in chain.
                let mut last_coplanar_node_index = node_index;
                while let Some(plane_index) = model.nodes[last_coplanar_node_index].plane_index {
                    last_coplanar_node_index = plane_index;
                }

                // Add and subtract work the same in this step.
                let node_count = model.nodes.len();
                let mut filter_context = FilterContext {
                    discarded: 0,
                    node_index,
                    last_coplanar_node_index,
                    model,
                };

                let filter_func = match csg_operation {
                    ECsgOper::Add => add_world_to_brush_func,
                    ECsgOper::Subtract => subtract_world_to_brush_func,
                };

                // Do the filter operation.
                bsp_filter_fpoly(filter_func, brush, &mut temp_ed_poly, &mut filter_context);

                if filter_context.discarded == 0 {
                    // Get rid of all the fragments we added.
                    // BDK: `filter_context` and `model` 
                    filter_context.model.nodes[filter_context.last_coplanar_node_index].plane_index = None;
                    filter_context.model.nodes.truncate(node_count);
                } else {
					// Tag original world poly for deletion; has been deleted or replaced by partial fragments.
                    filter_context.model.nodes[filter_context.node_index].vertex_count = 0;
                }
            }
        }

        // Now recurse to filter all of the world's children nodes.
        if do_front {
            if let Some(front_node_index) = model.nodes[node_index].front_node_index {
                filter_world_through_brush(model, brush, csg_operation, front_node_index, brush_sphere);
            }
        }

        if do_back {
            if let Some(back_node_index) = model.nodes[node_index].back_node_index {
                filter_world_through_brush(model, brush, csg_operation, back_node_index, brush_sphere);
            }
        }

        match model.nodes[node_index].plane_index {
            Some(plane_index) => node_index = plane_index,
            None => return,
        }
    }
}

/// Perform any CSG operation between the brush and the world.
pub fn bsp_brush_csg(
    actor: &ABrush, 
    model: &mut UModel, 
    poly_flags: EPolyFlags, 
    csg_operation: ECsgOper, 
    should_build_bounds: bool
) {
    // Non-solid and semisolid stuff can only be added.
    let not_poly_flags = match csg_operation {
        ECsgOper::Add => EPolyFlags::Semisolid | EPolyFlags::NotSolid,
        _ => EPolyFlags::empty()
    };

    // TODO: we're making a whole new model here, but the original code uses a global and clears it.
    let mut temp_model: UModel = UModel::new(true);

    // Build the brush's coordinate system and find orientation of scale
	// transform (if negative, edpolyTransform will reverse the clockness
	// of the EdPoly points and invert the normal).
    let (coords, uncoords, orientation) = ABrush::build_coords();

	// Transform original brush poly into same coordinate system as world
	// so Bsp filtering operations make sense.
    let pre_pivot = actor.pre_pivot;
    let location = actor.location;
    let brush = &actor.model;

    for (poly_index, poly) in brush.polys.iter().enumerate() {
		// Set texture the first time.
        // SKIPPED

        // Get the brush poly.
        let mut dest_ed_poly = poly.clone();
        assert!(brush.polys[poly_index].link.is_none() || brush.polys[poly_index].link.unwrap() < brush.polys.len());

        // Set its backward brush link.
        dest_ed_poly.brush_id = Some(actor.id);
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
    };

    // Pass the brush polys through the world Bsp.
    for i in 0..temp_model.polys.len() {
        let mut ed_poly = temp_model.polys[i].clone();

        // Mark the polygon as non-cut so that it won't be harmed unless it must
        // be split, and set iLink so that BspAddNode will know to add its information
        // if a node is added based on this poly.
        ed_poly.poly_flags &= !EPolyFlags::EdCut;

        if ed_poly.link == Some(i) {
            let a = model.surfaces.len();
            temp_model.polys[i].link = Some(a);
            ed_poly.link = Some(a);
        } else {
            ed_poly.link = temp_model.polys[ed_poly.link.unwrap()].link;
        }

        // HACK: create a filter context here, it won't actually be used in this case but it's necessary for the call.
        let mut dummy_model = UModel::new(true);
        let mut filter_context = FilterContext {
            discarded: 0,
            node_index: 0,
            last_coplanar_node_index: 0,
            model: &mut dummy_model,
        };

        // Filter brush through the world.
        bsp_filter_fpoly(filter_func, model, &mut ed_poly, &mut filter_context);
    }

    if !model.nodes.is_empty() && !poly_flags.intersects(EPolyFlags::NotSolid | EPolyFlags::Semisolid) {

        // BDK:
        // This is (I think) the part of the code that splits and removes faces from the existing world model
        // that were cut or obscured by the brush. It also (presumably) removes any faces on the brush that
        // fall outside the BSP tree.
        //
        // The bspCleanup call is the part that removes the junk nodes that were created during the CSG operation.

		// Quickly build a Bsp for the brush, tending to minimize splits rather than balance
		// the tree.  We only need the cutting planes, though the entire Bsp struct (polys and
		// all) is built.

        // NOTE: TempModel is the brush model.

        bsp_build(&mut temp_model, EBspOptimization::Lame, 0, 70, true);

        temp_model.build_bound();
        
        // Does the sphere get modified here?
        let brush_sphere = temp_model.bounding_sphere.clone();

        filter_world_through_brush(model, &mut temp_model, csg_operation, 0, brush_sphere);
    }
    
    // Clean up nodes, reset node flags.
    bsp_cleanup(model);

    // Rebuild bounding volumes.
    if should_build_bounds {
        bsp_build_bounds(model);
    }

    // Release TempModel.
    temp_model.empty_model(true, true);
}

/// Clean up all nodes after a CSG operation.  Resets temporary bit flags and unlinks
/// empty leaves.  Removes zero-vertex nodes which have nonzero-vertex coplanars.
fn bsp_cleanup(model: &mut UModel) {
    if !model.nodes.is_empty() {
        cleanup_nodes(model, 0, None)
    }
}

/// Recursive worker function called by BspCleanup.
fn cleanup_nodes(model: &mut UModel, node_index: usize, parent_node_index: Option<usize>) {
    {
        let node = &mut model.nodes[node_index];

        // Transactionally empty vertices of tag-for-empty nodes.
        node.node_flags &= !(EBspNodeFlags::IsNew | EBspNodeFlags::IsFront | EBspNodeFlags::IsBack);
    }

    let (node_front_node_index, node_back_node_index, node_plane_index) = {
        let node = &model.nodes[node_index];
        (node.front_node_index, node.back_node_index, node.plane_index)
    };

    // Recursively clean up front, back, and plane nodes.
    if let Some(front_node_index) = node_front_node_index {
        cleanup_nodes(model, front_node_index, Some(node_index));
    }
    if let Some(back_node_index) = node_back_node_index {
        cleanup_nodes(model, back_node_index, Some(node_index));
    }
    if let Some(plane_node_index) = node_plane_index {
        cleanup_nodes(model, plane_node_index, Some(node_index));
    }

    // Reload Node since the recursive call aliases it.
    if model.nodes[node_index].vertex_count > 0 {
        return
    }

    let (node_front_node_index, node_back_node_index, node_plane_index) = {
        let node = &model.nodes[node_index];
        (node.front_node_index, node.back_node_index, node.plane_index)
    };

    if let Some(plane_index) = node_plane_index {
        {
            let [node, plane_node] = model.nodes.get_many_mut([node_index, plane_index]).unwrap();
    
            // Stick our front, back, and parent nodes on the coplanar.
            if node.plane.normal().dot(plane_node.plane.normal()) >= 0.0 {
                plane_node.front_node_index = node_front_node_index;
                plane_node.back_node_index = node_back_node_index;
            } else {
                plane_node.front_node_index = node_back_node_index;
                plane_node.back_node_index = node_front_node_index;
            }
        }

        match parent_node_index {
            None => {
                // This node is the root.
                let [node, plane_node] = model.nodes.get_many_mut([node_index, plane_index]).unwrap();
                *node = plane_node.clone();     // Replace root.
                plane_node.vertex_count = 0;    // Mark as unused.
            }
            Some(parent_node_index) => {
                // This is a child node.
                let parent_node = &mut model.nodes[parent_node_index];

                if parent_node.front_node_index == Some(node_index) {
                    parent_node.front_node_index = node_plane_index;
                } else if parent_node.back_node_index == Some(node_index) {
                    parent_node.back_node_index = node_plane_index;
                } else if parent_node.plane_index == Some(node_index) {
                    parent_node.plane_index = node_plane_index;
                }
            }
        }
    } else if node_front_node_index.is_none() || node_back_node_index.is_none() {
		// Delete empty nodes with no fronts or backs.
		// Replace empty nodes with only fronts.
		// Replace empty nodes with only backs.
        let replacement_node_index = if node_front_node_index.is_some() {
            node_front_node_index
        } else if node_back_node_index.is_some() {
            node_back_node_index
        } else {
            None
        };

        match parent_node_index {
            None => {
                // Root.
                match replacement_node_index {
                    None => {
                        model.nodes.clear();
                    }
                    Some(replacement_node_index) => {
                        let [node, replacement_node] = model.nodes.get_many_mut([node_index, replacement_node_index]).unwrap();
                        *node = replacement_node.clone();
                    }
                }
            },
            Some(parent_node_index) => {
			    // Regular node.
                let parent_node = &mut model.nodes[parent_node_index];

                if parent_node.front_node_index == Some(node_index) {
                    parent_node.front_node_index = replacement_node_index;
                } else if parent_node.back_node_index == Some(node_index) {
                    parent_node.back_node_index = replacement_node_index;
                } else if parent_node.plane_index == Some(node_index) {
                    parent_node.plane_index = replacement_node_index;
                }
            }
        }
    }
}

// Build a 64-bit zone mask for each node, with a bit set for every
// zone that's referenced by the node and its children.  This is used
// during rendering to reject entire sections of the tree when it's known
// that none of the zones in that section are active.
fn build_zone_masks(model: &mut UModel, node_index: usize) -> u64 {
    let mut zone_mask = 0u64;

    {
        let node = &model.nodes[node_index];
        if node.zone[0] != 0 {
            zone_mask |= 1u64 << node.zone[0];
        }
        if node.zone[1] != 0 {
            zone_mask |= 1u64 << node.zone[1];
        }
    }

    let (node_front_node_index, node_back_node_index, node_plane_node_index) = {
        let node = &model.nodes[node_index];
        (node.front_node_index, node.back_node_index, node.plane_index)
    };

    if let Some(front_node_index) = node_front_node_index {
        zone_mask |= build_zone_masks(model, front_node_index);
    }
    if let Some(back_node_index) = node_back_node_index {
        zone_mask |= build_zone_masks(model, back_node_index);
    }
    if let Some(plane_node_index) = node_plane_node_index {
        zone_mask |= build_zone_masks(model, plane_node_index);
    }

    model.nodes[node_index].zone_mask = zone_mask;

    zone_mask
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
pub fn bsp_build_bounds(model: &mut UModel) {
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
fn bsp_build(model: &mut UModel, optimization: EBspOptimization, balance: u8, portal_bias: u8, rebuild_simple_polys: bool) {
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

    let split_poly_index = find_best_split_with_indices(&model.polys, poly_indices, optimization, balance, portal_bias);

	// Add the splitter poly to the Bsp with either a new BspSurf or an existing one.
    if rebuild_simple_polys {
        if let Some(split_poly_index) = split_poly_index {
            let split_poly = &mut model.polys[split_poly_index];
            split_poly.link = Some(model.surfaces.len());
        }
    }

    let split_poly = model.polys[split_poly_index.unwrap()].clone(); // TODO: if the split poly function is always assumed to return a valid index, just don't wrap it in an optional.
    let our_node_index = bsp_add_node(model, parent_node_index, node_place, EBspNodeFlags::empty(), &split_poly);
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
                    // TODO: this will fail if the surfaces are empty.
                    ed_poly.link = Some(model.surfaces.len() - 1);
                }
                plane_node_index = bsp_add_node(model, Some(plane_node_index), ENodePlace::Plane, EBspNodeFlags::empty(), &ed_poly_copy);
            }
            ESplitType::Front => {
                front_poly_indices.push(*poly_index);
            }
            ESplitType::Back => {
                back_poly_indices.push(*poly_index);
            }
            ESplitType::Split(front_poly, back_poly) => {
				// Create front & back nodes.
                {
                    front_poly_indices.push(model.polys.len());
                    model.polys.push(front_poly);
    
                    back_poly_indices.push(model.polys.len());
                    model.polys.push(back_poly);
                }

                // BDK: Get mutable references to the front and back polys that we just added.
                let mut split_polies_to_add = vec![];
                {
                    let mut poly_count = model.polys.len();
                    let [front_poly, back_poly] = model.polys.get_many_mut([*front_poly_indices.last().unwrap(), *back_poly_indices.last().unwrap()]).unwrap();
    
                    // If newly-split polygons have too many vertices, break them up in half.
                    if front_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
                        let split_poly = front_poly.split_in_half().unwrap();
                        front_poly_indices.push(poly_count);
                        poly_count += 1;
                        split_polies_to_add.push(split_poly);
                    }
    
                    if back_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
                        let split_poly = back_poly.split_in_half().unwrap();
                        back_poly_indices.push(poly_count);
                        split_polies_to_add.push(split_poly);
                    }
                }

                // Add the split polys.
                for split_poly in split_polies_to_add {
                    model.polys.push(split_poly);
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

    find_best_split_with_indices(polys, &poly_indices, optimization, balance, portal_bias)
}

fn find_best_split_with_indices(polys: &[FPoly], poly_indices: &[usize], optimization: EBspOptimization, balance: u8, portal_bias: u8) -> Option<usize> {
    let portal_bias = portal_bias as f32 / 100.0;

	// No need to test if only one poly.
    if poly_indices.len() == 1 {
        return Some(poly_indices[0]);
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

    // BDK: how is it possible to get a None out of this if there is at least one poly?

    // Iterate over the polys with the given step.
    for i in (0..poly_indices.len()).step_by(step) {
        // TODO: it's a little dicey here because we split the poly and indices up.
        let mut index = i;

        if !all_semi_solids {
            // BDK: Find the next non-semi-solid poly.
            loop {
                if index >= (i + step) || 
                    index >= polys.len() || 
                   !polys[poly_indices[index]].poly_flags.contains(EPolyFlags::AddLast) ||
                   polys[poly_indices[index]].poly_flags.contains(EPolyFlags::Portal)
                {
                    break;
                }
                
                index += 1;
            }
        }

        if index >= (i + step) || index >= poly_indices.len() {
            continue;
        }

        let poly = &polys[poly_indices[index]];
        let mut splits = 0;
        let mut front = 0;
        let mut back = 0;

        for j in 0..poly_indices.len() {
            if i == j {
                continue;
            }
            let other_poly = &polys[poly_indices[j]];
            match other_poly.split_with_plane_fast(&poly.vertices[0], &poly.normal) {
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
                ESplitType::Coplanar => { }
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

            if score < best_score || best_poly_index.is_none() {
                best_poly_index = Some(poly_indices[index]);
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

fn make_ed_polys(model: &mut UModel, node_index: usize) {
    let mut node_index_stack = vec![node_index];
    while let Some(node_index) = node_index_stack.pop() {
        if let Some(fpoly) = bsp_node_to_fpoly(model, node_index) {
            model.polys.push(fpoly);
        }
        let node = &model.nodes[node_index];

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

/// Build EdPoly list from a model's Bsp. Not transactional.
pub fn bsp_build_fpolys(model: &mut UModel, surf_links: bool, node_index: usize) {
    model.polys.clear();

    if !model.nodes.is_empty() {
        make_ed_polys(model, node_index);
    }

    if !surf_links {
        for (poly_index, poly) in model.polys.iter_mut().enumerate() {
            poly.link = Some(poly_index);
        }
    }
}

pub fn bsp_repartition(model: &mut UModel, node_index: usize, rebuild_simple_polys: bool) {
    bsp_build_fpolys(model, true, node_index);
    bsp_merge_coplanars(model, false, false);
    bsp_build(model, EBspOptimization::Good, 12, 70, rebuild_simple_polys);
    bsp_refresh(model, true);
}

// Add a point to a Bsp node before a specified vertex (between it and the previous one).
// VertexNumber can be from 0 (before first) to Node->NumVertices (after last).
//
// Splits node into two coplanar polys if necessary. If the polygon is split, the
// vertices will be distributed among this node and it's newly-linked iPlane node
// in an arbitrary way, that preserves the clockwise orientation of the vertices.
//
// Maintains node-vertex list, if not NULL.
fn add_point_to_node(model: &mut UModel, point_verts: &mut FPointVertList, node_index: usize, vertex_number: usize, vertex_index: usize) {
    {
        let node = &mut model.nodes[node_index];

        if node.vertex_count + 1 >= BSP_NODE_MAX_NODE_VERTICES {
            // Just refuse to add point: This is a non-fatal problem.
            return;
        }
    }

	// Remove node from vertex list, since vertex numbers will be reordered.
    point_verts.remove_node(model, node_index);

    let node = &mut model.nodes[node_index];
    let old_vert = node.vertex_pool_index;

    // BDK: Add a bunch of vertices to the vertex pool.
    node.vertex_pool_index = model.vertices.len();
    model.vertices.extend(iter::repeat(FVert::new()).take(node.vertex_count + 1));

	// Make sure this node doesn't already contain the vertex.
    for i in 0..node.vertex_count {
        debug_assert!(model.vertices[old_vert + i].vertex_index != vertex_index);
    }

    // Copy the old vertex pool to the new one.
    for i in 0..vertex_number {
        model.vertices[node.vertex_pool_index + i] = model.vertices[old_vert + i];
    }

    for i in vertex_number..node.vertex_count {
        model.vertices[node.vertex_pool_index + i + 1] = model.vertices[old_vert + i];
    }

	// Add the new point to the new vertex pool.
    {
        let new_point = &mut model.vertices[node.vertex_pool_index + vertex_number];
        new_point.vertex_index = vertex_index;
        new_point.side_index = None;
    }

    // Increment number of node vertices.
    node.vertex_count += 1;

    // Update the point-vertex list.
    point_verts.add_node(model, node_index);
}

/// Add a point to all sides of polygons in which the side intersects with
/// this point but doesn't contain it, and has the correct (clockwise) orientation
/// as this side.  pVertex is the index of the point to handle, and
/// ReferenceVertex defines the direction of this side.
pub fn distribute_point<'a>(model: &'a mut UModel, point_verts: &'a mut FPointVertList, node_index: usize, vertex_index: usize) -> usize {
    let mut count = 0usize;

    // Handle front, back, and plane.
    let distance = model.nodes[node_index].plane.plane_dot(model.points[vertex_index]);

    const THRESH_OPTGEOM_COPLANAR: f32 = 0.25f32;		/* Threshold for Bsp geometry optimization */
    const THRESH_OPTGEOM_COSIDAL: f32 = 0.25f32;		/* Threshold for Bsp geometry optimization */

    if distance < THRESH_OPTGEOM_COPLANAR {
        // Back.
        if let Some(back_node_index) = model.nodes[node_index].back_node_index {
            count += distribute_point(model, point_verts, back_node_index, vertex_index);
        }
    }

    if distance > -THRESH_OPTGEOM_COPLANAR {
        // Front.
        if let Some(front_node_index) = model.nodes[node_index].front_node_index {
            count += distribute_point(model, point_verts, front_node_index, vertex_index);
        }
    }

    if distance > -THRESH_OPTGEOM_COSIDAL && distance < THRESH_OPTGEOM_COPLANAR {
		// This point is coplanar with this node, so check point for intersection with
		// this node's sides, then loop with its coplanars.
        loop {
            let node = &model.nodes[node_index];
            let vert_pool = &model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];

			// Skip this node if it already contains the point in question.
            if vert_pool.iter().find(|v| v.vertex_index == vertex_index).is_none() {
                continue;
            }

			// Loop through all sides and see if (A) side is colinear with point, and
			// (B) point falls within inside of this side.
            let mut found_side = None;
            let mut skipped_colinear = false;
            let mut skipped_inside = false;
            let mut is_point_outside_polygon = false;   // BDK: New variable to track if the point is outside the polygon. Previous code was using the iterator to determine this.

            for i in 0..vert_pool.len() {
                let j = if i > 0 { i - 1 } else {vert_pool.len() - 1 };

				// Create cutting plane perpendicular to both this side and the polygon's normal.
                let side = model.points[vert_pool[i].vertex_index] - model.points[vert_pool[j].vertex_index];
                let side_plane_normal = side.cross(node.plane.normal());
                let size_squared = side_plane_normal.magnitude2();

                if size_squared > (0.001 * 0.001) {
                    // Points aren't coincident.
                    let dist = (model.points[vertex_index] - model.points[vert_pool[i].vertex_index]).dot(side_plane_normal) / size_squared.sqrt();

                    if dist >= THRESH_OPTGEOM_COSIDAL {
						// Point is outside polygon, can't possibly fall on a side.
                        is_point_outside_polygon = true;
						break;
                    } else if dist > -THRESH_OPTGEOM_COSIDAL {
						// The point we're adding falls on this line.
						//
						// Verify that it falls within this side; though it's colinear
						// it may be out of the bounds of the line's endpoints if this side
						// is colinear with an adjacent side.
						//
						// Do this by checking distance from point to side's midpoint and
						// comparing with the side's half-length.
                        let mid_point = (model.points[vert_pool[i].vertex_index] + model.points[vert_pool[j].vertex_index]) * 0.5f32;
                        let mid_dist_vect = model.points[vertex_index] - mid_point;

                        if mid_dist_vect.magnitude2() <= (0.501 * 0.501) * side.magnitude2() {
                            found_side = Some(i);
                        } else {
                            skipped_colinear = true;
                        }
                    } else {
						// Point is inside polygon, so continue checking.
                        skipped_inside = true;
                    }
                } else {
                    // TODO: increment error count??
                }
            }

            // TODO: i needs to be defined outside of the loop.
            if !is_point_outside_polygon && found_side.is_some() {
				// AddPointToNode will reorder the vertices in this node.  This is okay
				// because it's called outside of the vertex loop.
                // TODO: redundant, point_verts contains a reference to the model.
                add_point_to_node(model, point_verts, node_index, found_side.unwrap(), vertex_index);
                count += 1;
            } else if skipped_colinear {
				// This happens occasionally because of the fuzzy Dist comparison.  It is
				// not a sign of a problem when the vertex being distributed is colinear
				// with one of this polygon's sides, but slightly outside of this polygon.
                // TODO: increment error count
            } else if skipped_inside {
				// Point is on interior of polygon.
                // TODO: increment error count
            }
        }
    }

    count
}

/// Optimize a level's Bsp, eliminating T-joints where possible, and building side
/// links.  This does not always do a 100% perfect job, mainly due to imperfect 
/// levels, however it should never fail or return incorrect results.
pub fn bsp_opt_geom(model: &mut UModel) {
    merge_near_points(model, 0.25f32);
    bsp_refresh(model, false);

    // First four entries are reserved for view-clipped sides.
    model.num_shared_sides = 4;

	// Mark all sides as unlinked.
    for vert in model.vertices.iter_mut() {
        vert.side_index = None;
    }

    let mut tees_found = 0;

    let mut point_verts = FPointVertList::new(model.points.len());
    point_verts.add_all_nodes(model);
    
	// Eliminate T-joints on each node by finding all vertices that aren't attached to
	// two shared sides, then filtering them down through the BSP and adding them to
	// the sides they belong on.
    for node_index in 0..model.nodes.len() {
		// Loop through all sides (side := line from PrevVert to ThisVert)	
        let node_vertex_count = model.nodes[node_index].vertex_count;
        let node_vertex_pool_index = model.nodes[node_index].vertex_pool_index;
        for this_vert in 0..node_vertex_count {
            let prev_vert = if this_vert > 0 { this_vert - 1 } else { node_vertex_count - 1 };

			// Count number of nodes sharing this side, i.e. number of nodes for
			// which two adjacent vertices are identical to this side's two vertices.
            let skip_it = point_verts.iter(model, model.nodes[node_index].vertex_pool_index + this_vert).any(|point_vert_1| {
                point_verts.iter(model, model.nodes[node_index].vertex_pool_index + prev_vert).any(|point_vert_2| {
                    point_vert_1.node_index == point_vert_2.node_index && point_vert_1.node_index != node_index
                })
            });

            if !skip_it {
                // Didn't find another node that shares our two vertices; must add each
                // vertex to all polygons where the vertex lies on the polygon's side.
                // DistributePoint will not affect the current node but may change others
                // and may increase the number of nodes in the Bsp.
                tees_found += 1;
                distribute_point(model, &mut point_verts, 0, model.vertices[node_vertex_pool_index + this_vert].vertex_index);
                distribute_point(model, &mut point_verts, 0, model.vertices[node_vertex_pool_index + prev_vert].vertex_index);
            }
        }
    }

	// Build side links
	// Definition of side: Side (i) links node vertex (i) to vertex ((i+1)%n)
    let mut point_verts = FPointVertList::new(model.points.len());
    point_verts.add_all_nodes(model);

    for node_index in 0..model.nodes.len() {
        let node_vertex_count = model.nodes[node_index].vertex_count;
        let node_vertex_pool_index = model.nodes[node_index].vertex_pool_index;
        for this_vert in 0..node_vertex_count {
            // BDK: This used a goto statement in the original code. It has been replaced with a closure that returns when the side is linked.
            let mut link_side = || {
                if model.vertices[node_vertex_pool_index + this_vert].side_index.is_none() {
                    // See if this node links to another one.
                    let prev_vert = if this_vert > 0 { this_vert - 1 } else { node_vertex_count - 1 };
                    let a = model.vertices[node_vertex_pool_index + this_vert].vertex_index;
                    let b = model.vertices[node_vertex_pool_index + prev_vert].vertex_index;

                    for point_vert_1 in point_verts.iter(model, a) {
                        for point_vert_2 in point_verts.iter(model, b) {
                            if point_vert_1.node_index == point_vert_2.node_index && point_vert_1.node_index != node_index {
                                // Make sure that the other node's two vertices are adjacent and
                                // ordered opposite this node's vertices.
                                let other_node_index = point_vert_2.node_index;
                                let other_node = &model.nodes[other_node_index];
                                let delta = (other_node.vertex_count + point_vert_2.vertex_index - point_vert_1.vertex_index) % other_node.vertex_count;

                                if delta == 1 {
                                    // Side is properly linked!
                                    let other_vert = point_vert_2.vertex_index;
                                    let side_index = model.vertices[other_node.vertex_pool_index + other_vert].side_index.unwrap_or_else(|| {
                                        model.num_shared_sides += 1;
                                        model.num_shared_sides
                                    });

                                    // Link both sides to the shared side.
                                    model.vertices[node_vertex_pool_index + this_vert].side_index = Some(side_index);
                                    model.vertices[other_node.vertex_pool_index + other_vert].side_index = Some(side_index);

                                    // goto SkipSide;
                                    return ();
                                }
                            }
                        }
                    }

                    // This node doesn't have correct side linking
                    //GErrors++;
                }
            };
            link_side();

            // Go to next side.
            // SkipSide:
        }
    }

    // Gather stats.
    let mut i = 0;
    let mut j = 0;

    for node_index in 0..model.nodes.len() {
        let node = &model.nodes[node_index];
        let vert_pool = &model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];

        i += node.vertex_count;
        j += (0..node.vertex_count)
            .filter(|&this_vert| vert_pool[this_vert].side_index.is_some())
            .count();
    }

    // Done.
    println!("BspOptGeom end");
    println!("Processed {} T-points, linked: {}/{} sides", tees_found, j, i);

	// Remove unused vertices from the vertex streams.
	// This is necessary to ensure the vertices added to eliminate T junctions
	// don't overflow the 65536 vertex/stream limit.
    bsp_refresh(model, false);
}

/// A node and vertex number corresponding to a point, used in generating Bsp side links.
struct FPointVert {
    pub node_index: usize,
    pub vertex_index: usize,
    pub next_index: Option<usize>,
}

/// A list of point/vertex links, used in generating Bsp side links.
struct FPointVertList {
    // pub model: &'a mut UModel,
    pub indices: Vec<Option<FPointVert>>,
}

struct PointVertIter<'a> {
    point_verts: &'a [Option<FPointVert>],
    current_index: Option<usize>,
}

impl<'a> PointVertIter<'a> {
    fn new(start_index: Option<usize>, point_verts: &'a [Option<FPointVert>]) -> Self {
        Self {
            point_verts,
            current_index: start_index,
        }
    }
}

impl FPointVertList {
    fn iter(&self, model: &mut UModel, vert_idx: usize) -> PointVertIter {
        let start_index = model.vertices[vert_idx].vertex_index;
        PointVertIter::new(Some(start_index), &self.indices)
    }
}

impl<'a> Iterator for PointVertIter<'a> {
    type Item = &'a FPointVert;

    fn next(&mut self) -> Option<Self::Item> {
        self.current_index.and_then(|index| {
            if let Some(ref point_vert) = self.point_verts[index] {
                self.current_index = point_vert.next_index;
                Some(point_vert)
            } else {
                None
            }
        })
    }
}

impl<'a> FPointVertList {
    pub fn new(capacity: usize) -> Self {
        Self {
            indices: Vec::with_capacity(capacity),  // TODO: should this actually be just filled with None?
        }
    }

    /// Add all of a node's vertices to a node-vertex list.
    pub fn add_node(&mut self, model: &UModel, node_index: usize) {
        let node = &model.nodes[node_index];
        let vert_pool = &model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];

        for (i, vert) in vert_pool.iter().enumerate() {
            let vertex_index = vert.vertex_index;

			// Add new point/vertex pair to array, and insert new array entry
			// between index and first entry.
            self.indices[vertex_index] = Some(FPointVert {
                node_index,
                vertex_index: i,
                next_index: Some(vertex_index),
            });
        }
    }

    /// Add all nodes' vertices in the model to a node-vertex list.
    pub fn add_all_nodes(&mut self, model: &UModel) {
        for node_index in 0..model.nodes.len() {
            self.add_node(model, node_index);
        }
    }

    // TODO: definitely not tested and probably broken.
    /// Remove all of a node's vertices from a node-vertex list.
    pub fn remove_node(&mut self, model: &mut UModel, node_index: usize) {
        let node = &model.nodes[node_index];
        let vert_pool = &mut model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];

		// Loop through all of the node's vertices and search through the
		// corresponding point's node-vert list, and delink this node.
        let mut count = 0;
        for vert in vert_pool.iter_mut() {
            let vertex_index = vert.vertex_index;
            let mut prev_link_index_optional = Some(vertex_index);
            while let Some(prev_link_index) = prev_link_index_optional {
                if let Some(prev_link) = &mut self.indices[prev_link_index] {
                    if prev_link.node_index == node_index {
                        // Delink this entry from the list.
                        prev_link_index_optional = prev_link.next_index;
                        count += 1;
                    }
                } else {
                    break;
                }
            }

			// Node's vertex wasn't found, there's a bug.
            assert!(count >= 1);
        }
    }
}
