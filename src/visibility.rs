extern crate bit_vec;

use arrayvec::ArrayVec;
use bit_vec::BitVec;
use cgmath::InnerSpace;

use crate::bsp::{bsp_add_node, bsp_build_bounds, bsp_cleanup, bsp_node_to_fpoly, bsp_refresh, build_zone_masks, WORLD_MAX};
use crate::fpoly::{EPolyFlags, ESplitType, FPoly, FPOLY_VERTEX_THRESHOLD};
use crate::math::FVector;
use crate::model::{EBspNodeFlags, FLeaf, UModel};
use crate::Poly;

/// An nxn symmetric bit array.
pub struct UBitMatrix {  // bitset??
    size: usize,
    data: BitVec,
}

// BDK:
// The original code makes accomodations for the fact that the diagonals will all be 1s and skips storing those.
// However, this means it has to massage the incoming indices on each read/write call.
// I think the miniscule memory savings at the expense of the speed is not worth it.
// Even if you had 65536 leaf nodes (an obscene amount), you'd be saving a grand total of 8KB of memory.
impl UBitMatrix {
    pub fn new(size: usize) -> UBitMatrix {
        // Set all the diagonal values to 1.
        let mut data = BitVec::from_elem(size, false);
        for i in (0..(size * size)).step_by(size + 1) {
            data.set(i, true);
        }
        UBitMatrix { size, data }
    }

    pub fn get(&self, i: usize, j: usize) -> bool {
        self.data.get(i + j * self.size).unwrap()
    }

    pub fn set(&mut self, i: usize, j: usize, value: bool) {
        self.data.set(i + j * self.size, value)
    }
}

pub struct FPortal {
    pub poly: FPoly,
    pub front_leaf_index: usize,
    pub back_leaf_index: usize,
    pub node_index: usize,
    //FPortal *GlobalNext, *FrontLeafNext, *BackLeafNext, *NodeNext;
    pub is_testing: bool,
    pub should_test: bool,
    pub fragment_count: usize,
    pub zone_portal_surface_index: Option<usize>,
}

impl FPortal {

	/// Get the leaf on the opposite side of the specified leaf.
    pub fn get_neighbor_leaf_of(&self, leaf_index: usize) -> usize {
        assert!(leaf_index == self.front_leaf_index || leaf_index == self.back_leaf_index);
        if self.front_leaf_index == leaf_index {
            self.back_leaf_index
        } else {
            self.front_leaf_index
        }
    }

    pub fn area(&self) -> f32 {
        let mut cross = FVector::new(0.0f32, 0.0f32, 0.0f32);
        let vertices = &self.poly.vertices;
        for i in 2..vertices.len() {
            cross += (vertices[i - 1] - vertices[0]).cross(vertices[i] - vertices[0]);
        }
        cross.magnitude()
    }
    
	// FPortal* Next( INT iLeaf )
	// {
	// 	check( iLeaf==iFrontLeaf || iLeaf==iBackLeaf );
	// 	if     ( iFrontLeaf == iLeaf )	return FrontLeafNext;
	// 	else							return BackLeafNext;
	// }

    
	/// Return this portal polygon, facing outward from leaf iLeaf.
    /// BDK: This doesn't actually return anything, the original comment is wrong.
    pub fn get_poly_facing_out_of(&self, leaf_index: usize, poly: &mut FPoly) {
        assert!(leaf_index == self.front_leaf_index || leaf_index == self.back_leaf_index);
        poly.clone_from(&self.poly);
        if leaf_index == self.front_leaf_index {
            poly.reverse();
        }
    }

	// Return this portal polygon, facing inward to leaf iLeaf.
    /// BDK: This doesn't actually return anything, the original comment is wrong.
    pub fn get_poly_facing_into(&self, leaf_index: usize, poly: &mut FPoly) {
        assert!(leaf_index == self.front_leaf_index || leaf_index == self.back_leaf_index);
        poly.clone_from(&self.poly);
        if leaf_index == self.back_leaf_index {
            poly.reverse();
        }
    }


}

const MAX_CLIPS: usize = 16384;
const CLIP_BACK_FLAG: usize = 0x40000000;

pub struct FEditorVisibility<'a>
{
    model: &'a mut UModel,
    portal_count: usize,
    clip_count: usize,
    clips: ArrayVec<usize, MAX_CLIPS>,
    clip_test_count: usize,
    passed_clip_count: usize,
    unclipped_count: usize,
    bsp_portal_count: usize,
    fragments_max: usize,
    zone_portal_count: usize,
    zone_fragment_count: usize,
    extra: usize,    // flags?
    //first_portal: &mut FPortal, // use an index instead?
    node_portals: Vec<FPortal>,
    leaf_portals: Vec<FPortal>,
    //leaf_lights: Vec<FActorLink>,   
    zone_portal_surface_index: Option<usize>,
}

type PortalFunc = fn(&mut FEditorVisibility, &FPoly, Option<usize>, Option<usize>, usize, usize);

impl FEditorVisibility<'_> {
    //
    // Filter a portal through a front or back subtree.
    //
    fn filter_through_subtree(
        &mut self,
        pass: usize,
        generating_node_index: usize,
        generating_base_index: usize,
        mut parent_leaf_index: Option<usize>,
        node_index: Option<usize>,
        mut poly: FPoly,
        func: PortalFunc,
        back_leaf_index: Option<usize>,
    ) {
        let mut outer_node_index = node_index;
        while let Some(node_index) = outer_node_index {
            // If overflow.
            if poly.vertices.len() > FPOLY_VERTEX_THRESHOLD {
                let poly_half = poly.split_in_half().unwrap();
                self.filter_through_subtree(
                    pass, generating_node_index, generating_base_index,
                    parent_leaf_index, Some(node_index), poly_half, func, back_leaf_index,
                );
            }

            // Test split.
            let split_type = poly.split_with_node(self.model, node_index, true);

            match split_type {
                ESplitType::Split(front, back) => {
                    self.filter_through_subtree(
                        pass, generating_node_index, generating_base_index,
                        self.model.nodes[node_index].leaf_indices[1],
                        self.model.nodes[node_index].front_node_index,
                        front, func, back_leaf_index,
                    );
                    poly = back;
                },
                ESplitType::Front => {
                    // BDK: Same as above, but using the original poly, not the front split poly.
                    self.filter_through_subtree(
                        pass, generating_node_index, generating_base_index,
                        self.model.nodes[node_index].leaf_indices[1], self.model.nodes[node_index].front_node_index,
                        poly, func, back_leaf_index,
                    );
                    return;
                },
                ESplitType::Back => {
                    
                }
                _ => { return }
            }

            parent_leaf_index = self.model.nodes[node_index].leaf_indices[0];
            outer_node_index = self.model.nodes[node_index].back_node_index;
        }

        if pass == 0 {
            self.filter_through_subtree(
                1,
                generating_node_index,
                generating_base_index,
                self.model.nodes[generating_base_index].leaf_indices[1],
                self.model.nodes[generating_base_index].front_node_index,
                poly,
                func,
                parent_leaf_index,
            )
        } else {
            func(self, &poly, parent_leaf_index, back_leaf_index, generating_node_index, generating_base_index)
        }
    }
    
    /// Assign contiguous unique numbers to all front and back leaves in the BSP.
    /// Stores the leaf numbers in FBspNode::iLeaf[2].
    fn assign_leaves(&mut self, node_index: usize, is_outside: bool) {
        let child_indices = {
            let node = &self.model.nodes[node_index];
            [node.back_node_index, node.front_node_index]
        };
        for (child_index, child_node_index) in child_indices.into_iter().enumerate() {
            if let Some(child_node_index) = child_node_index {
                let is_outside = self.model.nodes[node_index].is_child_outside(child_index, is_outside, EBspNodeFlags::NotVisBlocking);
                self.assign_leaves(child_node_index, is_outside);
            } else if self.model.nodes[node_index].is_child_outside(child_index, is_outside, EBspNodeFlags::NotVisBlocking) {
                let leaf_index = self.model.leaves.len();
                // BDK: why is the zone index being passed in as a leaf index?
                let leaf = FLeaf::new(leaf_index);
                self.model.nodes[node_index].leaf_indices[0] = Some(leaf_index);
                self.model.leaves.push(leaf);
            }
        }
    }

    /// Clip a portal by all parent nodes above it.
    fn make_portals_clip(&mut self, node_index: usize, poly: FPoly, clip: usize, portal_func: PortalFunc) {
        
    }

    /// Tag a zone portal fragment.
    fn tag_zone_portal_fragment(visibility: &mut FEditorVisibility, poly: &FPoly, front_leaf_index: Option<usize>, back_leaf_index: Option<usize>, generating_node_index: usize, generating_base_index: usize) {
	    // Add this node to the bsp as a coplanar to its generator.
        let new_node_index = bsp_add_node(
            visibility.model,
            Some(generating_node_index), 
            crate::bsp::ENodePlace::Plane,
            visibility.model.nodes[generating_node_index].node_flags | EBspNodeFlags::IsNew, poly,
        );
        // Set the node's zones.
        let backward = poly.normal.dot(visibility.model.nodes[generating_base_index].plane.normal()) < 0.0;
        let new_node = &mut visibility.model.nodes[new_node_index];
        new_node.zone[backward as usize] = match back_leaf_index {
            Some(back_leaf_index) => visibility.model.leaves[back_leaf_index].zone_index,
            None => 0,
        } as u8;
        new_node.zone[!backward as usize] = match front_leaf_index {
            Some(front_leaf_index) => visibility.model.leaves[front_leaf_index].zone_index,
            None => 0,
        } as u8;
    }

    /// Go through the Bsp and assign zone numbers to all nodes.  Prior to this
    /// function call, only leaves have zone numbers.  The zone numbers for the entire
    /// Bsp can be determined from leaf zone numbers.
    fn assign_all_zones(&mut self, node_index: usize, is_outside: bool) {
        let original_node_index = node_index;

        // Recursively assign zone numbers to children.
        if let Some(front_node_index) = self.model.nodes[node_index].front_node_index {
            self.assign_all_zones(front_node_index, is_outside);
        }
        if let Some(back_node_index) = self.model.nodes[node_index].back_node_index {
            self.assign_all_zones(back_node_index, is_outside);
        }
        
        let mut outer_node_index = Some(node_index);
        // Make sure this node's polygon resides in a single zone.  In other words,
        // find all of the zones belonging to outside Bsp leaves and make sure their
        // zone number is the same, and assign that zone number to this node.
        while let Some(node_index) = outer_node_index {
            let is_new = self.model.nodes[node_index].node_flags.contains(EBspNodeFlags::IsNew);
            if !is_new {
                if let Some(poly) = bsp_node_to_fpoly(&self.model, node_index) {
			        // Make sure this node is added to the BSP properly.
                    let original_node_count = self.model.nodes.len();
                    let original_node = &self.model.nodes[original_node_index];
                    self.filter_through_subtree(
                        0,
                        node_index,
                        original_node_index,
                        original_node.leaf_indices[0],
                        original_node.front_node_index,  // TODO: maybe this is BACK actually.
                        poly,
                        Self::tag_zone_portal_fragment,
                        None
                    );
                }
            }
            outer_node_index = self.model.nodes[node_index].plane_index;
        }
    }

    // void AddPortal( FPoly &Poly, INT iFrontLeaf, INT iBackLeaf, INT iGeneratingNode, INT iGeneratingBase );

    /// Make all portals.
    fn make_portals(&mut self, node_index: usize) {
        let original_node_index = node_index;

	    // Make an infinite edpoly for this node.
        let poly = build_infinite_poly(&self.model, node_index);

	    // Filter the portal through this subtree.
        self.make_portals_clip(node_index, poly, 0, Self::add_portal);

	    // Make portals for front.
        if let Some(front_node_index) = self.model.nodes[node_index].front_node_index {
            self.clips[self.clip_count] = node_index;
            self.clip_count += 1;
            self.make_portals(front_node_index);
            self.clip_count -= 1;
        }

    	// Make portals for back.
        if let Some(back_node_index) = self.model.nodes[node_index].back_node_index {
            self.clips[self.clip_count] = node_index | CLIP_BACK_FLAG;
            self.clip_count += 1;
            self.make_portals(back_node_index);
            self.clip_count -= 1;
        }

	    // For all zone portals at this node, mark the matching FPortals as blocked.
        !todo!("not done!");
        let mut outer_node_index = Some(node_index);
        while let Some(node_index) = outer_node_index {
            let node = &self.model.nodes[node_index];
            let surface = &self.model.surfaces[node.surface_index];
            let original_node = &self.model.nodes[original_node_index];

            if surface.poly_flags.contains(EPolyFlags::Portal) {
                if let Some(poly) = bsp_node_to_fpoly(self.model, node_index) {
                    self.zone_portal_count += 1;
                    self.zone_portal_surface_index = Some(node.surface_index);
                    self.filter_through_subtree(
                        0,
                        node_index,
                        original_node_index,
                        original_node.leaf_indices[0],
                        original_node.back_node_index,
                        poly,
                        Self::block_portal,
                        None,
                    );
                }
            }
            outer_node_index = node.plane_index;
        }
    }

    fn test_visibility(&mut self) {
        // Init Bsp info.
        for node in &mut self.model.nodes {
            node.leaf_indices.iter_mut().for_each(|f| { *f = None });
            node.zone.iter_mut().for_each(|z| { *z = 0 });
        }

	    // Invalidate actor render data which may reference specific leaf indices.
        // BDK: Skipped, not relevant right now.
        
	    // Allocate objects.
        self.model.leaves.clear();
        //self.model.lights.clear();

        // Assign leaf numbers to convex outisde volumes.
        self.assign_leaves(0, self.model.is_root_outside);

        // Allocate leaf info.
        self.leaf_portals.reserve_exact(self.model.leaves.len());
        // self.leaf_lights
        self.node_portals.reserve_exact(self.model.nodes.len() * 2 + 256);  // Allow for 2X expansion from zone portal fragments!!

        
	    // Build all portals, with references to their front and back leaves.
        self.make_portals(0);

        // Form zones.
        self.form_zones_from_leaves();
        self.assign_all_zones(0, self.model.is_root_outside);

	    // Cleanup the bsp.
	    // !!unsafe: screws up the node portals required for visibility checking.
	    // !!but necessary for proper rendering.
        bsp_cleanup(&mut self.model);
        bsp_refresh(&mut self.model, true);
        bsp_build_bounds(self.model);

    	// Build zone interconnectivity info.   
        build_zone_masks(&mut self.model, 0);
        self.build_connectivity();
        self.build_zone_info();

        
        println!("Portalized: {} portals, {} zone portals ({} fragments), {} leaves, {} nodes", self.portal_count, self.zone_portal_count, self.zone_fragment_count, self.model.leaves.len(), self.model.nodes.len());
        
	    // Get the rebuild options.

        // TODO: keep going.

    }

    fn bsp_visibility(&self, node_index: usize) {

    }

    fn bsp_cross_visibility(&self, front_portal_leaf_index: usize, back_portal_leaf_index: usize, front_leaf_index: usize, back_leaf_index: usize, front_poly: &FPoly, clip_poly: &FPoly, back_poly: &FPoly, valid_polys: usize, pass: usize) {

    }

    fn form_zones_from_leaves(&self) {

    }

    // Attach ZoneInfo actors to the zones that they belong in.
    // ZoneInfo actors are a class of actor which level designers may
    // place in UnrealEd in order to specify the properties of the zone they
    // reside in, such as water effects, zone name, etc.
    fn build_zone_info(&mut self) {
    }

    // Build 64x64 zone connectivity matrix.  Entry(i,j) is set if node i is connected
    // to node j.  Entry(i,i) is always set by definition.  This structure is built by
    // analyzing all portals in the world and tagging the two zones they connect.
    //
    // Called by: TestVisibility.
    fn build_connectivity(&mut self) {
    }

    fn add_portal(visibility: &mut FEditorVisibility, poly: &FPoly, front_leaf_index: Option<usize>, back_leaf_index: Option<usize>, generating_node_index: usize, generating_base: usize) {
        if let Some(front_leaf_index) = front_leaf_index { 
            if let Some(back_leaf_index) = back_leaf_index {
		    // Add to linked list of all portals.
            }
        }
    }

    /// Mark a portal as blocked.
    fn block_portal(visibility: &mut FEditorVisibility, poly: &FPoly, front_leaf_index: Option<usize>, back_leaf_index: Option<usize>, generating_node_index: usize, generating_base: usize) {
        if let Some(front_leaf_index) = front_leaf_index {
            if let Some(back_leaf_index) = back_leaf_index {
                !todo!("this iterates through a linked list of portals.")
            }
        }
    }
}


// Build an FPoly representing an "infinite" plane (which exceeds the maximum
// dimensions of the world in all directions) for a particular Bsp node.
fn build_infinite_poly(model: &UModel, node_index: usize) -> FPoly {
    let node = &model.nodes[node_index];
    let poly = &model.surfaces[node.surface_index];
    let normal = poly.plane.normal();
    let base = normal * poly.plane.w;
    
    // Find two non-problematic axis vectors.
    let (axis1, axis2) = find_best_axis_vectors(normal);

    // Set up the FPoly.
    FPoly::from_vertices_and_base(&[
        base + (axis1 * WORLD_MAX) + (axis2 * WORLD_MAX),
        base - (axis1 * WORLD_MAX) + (axis2 * WORLD_MAX),
        base - (axis1 * WORLD_MAX) - (axis2 * WORLD_MAX),
        base + (axis1 * WORLD_MAX) - (axis2 * WORLD_MAX),
    ], &base)
}


/// Find good arbitrary axis vectors to represent U and V axes of a plane
/// given just the normal.
fn find_best_axis_vectors(v: FVector) -> (FVector, FVector) {
    let nx = v.x.abs();
    let ny = v.y.abs();
    let nz = v.z.abs();

    // Find best basis vectors.
    let mut axis1 = if nz > nx && nz > ny {
        FVector::new(1.0, 0.0, 0.0)
    } else {
        FVector::new(0.0, 0.0, 1.0)
    };

    axis1 = (axis1 - v * (axis1.dot(v))).normalize();
    let axis2 = axis1.cross(v);

    (axis1, axis2)
}