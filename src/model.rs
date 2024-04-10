use std::result;

use bitflags::bitflags;
use cgmath::MetricSpace;
use crate::fpoly::{EPolyFlags, FPoly};
use crate::math::{FPlane, FVector};
use crate::sphere::FSphere;
use crate::box_::FBox;
use crate::math::{THRESH_POINTS_ARE_SAME, THRESH_POINTS_ARE_NEAR, THRESH_NORMALS_ARE_SAME, THRESH_VECTORS_ARE_NEAR};

pub struct FBspVertex {
    pub position: FVector,
    pub normal: FVector,
    pub u: f32,
    pub v: f32,
    pub u2: f32,
    pub v2: f32,
}

/// Flags associated with a Bsp node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EBspNodeFlags(u16);

bitflags! {
    impl EBspNodeFlags : u16 {
        /// Node is not a Csg splitter, i.e. is a transparent poly.
        const NotCsg                    = 0x0001;
        /// Can shoot through (for projectile solid ops).
        const ShootThrough	            = 0x0002;
        /// Node does not block visibility, i.e. is an invisible collision hull.
        const NotVisBlocking            = 0x0004;
        /// Node's poly was occluded on the previously-drawn frame.
        const PolyOccluded	            = 0x0008;
        /// Node's bounding box was occluded.
        const BoxOccluded	            = 0x0010;
        /// Temporary.
        const BrightCorners	            = 0x0010;
        /// Editor: Node was newly-added.
        const IsNew 		 	        = 0x0020;
        /// Filter operation bounding-sphere precomputed and guaranteed to be front.
        const IsFront     	            = 0x0040;
        /// Guaranteed back.
        const IsBack      	            = 0x0080;
        /// Use collision static mesh for traces against skeletal meshes.
        const UseCollisionStaticMesh    = 0x0100;
    }
}

/// One Bsp polygon.  Lists all of the properties associated with the
/// polygon's plane.  Does not include a point list; the actual points
/// are stored along with Bsp nodes, since several nodes which lie in the
/// same plane may reference the same poly.
#[derive(Debug, PartialEq)]
pub struct FBspSurf {
    //pub material: Rc<UMaterial>,
    pub poly_flags: EPolyFlags,
    pub base_point_index: usize,
    pub normal_index: usize,
    pub texture_u_index: usize,
    pub texture_v_index: usize,
    pub brush_polygon_index: Option<usize>,
    //pub actor: Rc<ABrush>,
    pub node_indices: Vec<usize>,   // TODO: what's this one??
    pub plane: FPlane,
    pub light_map_scale: f32
}

impl Default for FBspSurf {
    fn default() -> Self {
        FBspSurf {
            poly_flags: EPolyFlags::empty(),
            base_point_index: 0,
            normal_index: 0,
            texture_u_index: 0,
            texture_v_index: 0,
            brush_polygon_index: None,
            node_indices: Vec::new(),
            plane: FPlane::new(),
            light_map_scale: 0.0
        }
    }
}

/// Max vertices in a Bsp node, pre clipping.
pub const BSP_NODE_MAX_NODE_VERTICES: usize = 16;
/// Max vertices in a Bsp node, post clipping.
pub const BSP_NODE_MAX_FINAL_VERTICES: usize = 24;
/// Max zones per level.
pub const BSP_NODE_MAX_ZONES: usize = 64;

/// FBspNode defines one node in the Bsp, including the front and back
/// pointers and the polygon data itself.  A node may have 0 or 3 to (MAX_NODE_VERTICES-1)
/// vertices. If the node has zero vertices, it's only used for splitting and
/// doesn't contain a polygon (this happens in the editor).
///
/// vNormal, vTextureU, vTextureV, and others are indices into the level's
/// vector table.  iFront,iBack should be INDEX_NONE to indicate no children.
///
/// If iPlane==INDEX_NONE, a node has no coplanars.  Otherwise iPlane
/// is an index to a coplanar polygon in the Bsp.  All polygons that are iPlane
/// children can only have iPlane children themselves, not fronts or backs.
#[derive(Debug, PartialEq)]
pub struct FBspNode {
    /// Plane the node falls into (X, Y, Z, W).
    pub plane: FPlane,
    /// Bit mask for all zones at or below this node (up to 64).
    pub zone_mask: u64,
    /// Index of first vertex in vertex pool, =iTerrain if NumVertices==0 and NF_TerrainFront.
    pub vertex_pool_index: usize,
    /// Index to surface information.
    pub surface_index: usize,

    /// Index to node in front (in direction of Normal).
    pub back_node_index: Option<usize>,
    /// Index to node in back  (opposite direction as Normal).
    pub front_node_index: Option<usize>,
    /// Index to next coplanar poly in coplanar list.
    pub plane_index: Option<usize>,

    /// Bounding sphere excluding child nodes.
    pub exclusive_sphere_bound: FSphere,

    /// Collision bound.
    pub collision_bound: Option<usize>,
    /// Rendering bound.
    pub render_bound: Option<usize>,

    /// Visibility zone in 1=front, 0=back.
    pub zone: [u8;2],
    /// Number of vertices in node.
    pub vertex_count: usize,
    /// Node flags.
    pub node_flags: EBspNodeFlags,
    /// Leaf in back and front, INDEX_NONE = NOT A LEAF.
    pub leaf_indices: [Option<usize>;2],

    pub section_index: Option<usize>,
    pub first_vertex_index: usize,
    pub light_map_index: Option<usize>,
}

impl FBspNode {

    pub fn new() -> FBspNode {
        FBspNode {
            plane: FPlane::new(),
            zone_mask: 0,
            vertex_pool_index: 0,
            surface_index: 0,
            back_node_index: None,
            front_node_index: None,
            plane_index: None,
            exclusive_sphere_bound: FSphere::default(),
            collision_bound: None,
            render_bound: None,
            zone: [0, 0],
            vertex_count: 0,
            node_flags: EBspNodeFlags::empty(),
            leaf_indices: [None, None],
            section_index: None,
            first_vertex_index: 0,
            light_map_index: None,
        }
    }

    pub fn is_csg(&self, extra_flags: EBspNodeFlags) -> bool {
        self.vertex_count > 0 && self.node_flags.contains(EBspNodeFlags::IsNew | EBspNodeFlags::NotCsg | extra_flags)
    }

    pub fn is_child_outside(&self, child_index: usize, outside: bool, extra_flags: EBspNodeFlags) -> bool {
        if child_index != 0 {
            outside || self.is_csg(extra_flags)
        } else {
            outside && !self.is_csg(extra_flags)
        }
    }
}

/// One vertex associated with a Bsp node's polygon.  Contains a vertex index
/// into the level's FPoints table, and a unique number which is common to all
/// other sides in the level which are cospatial with this side.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FVert {
    pub vertex_index: usize,
    pub side_index: Option<usize>,
}

impl FVert {
    pub fn new() -> FVert {
        FVert {
            vertex_index: 0,
            side_index: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
/// Information about a convex volume.
pub struct FLeaf {
    /// The zone this convex volume is in.
    zone_index: usize,
    /// Lights permeating this volume considering shadowing.
    permeating: usize,
    /// Volumetric lights hitting this region, no shadowing.
    volumentic: usize,
    /// Bit mask of visible zones from this convex volume.
    visible_zone_bits: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FZoneProperties {
    // zone_actor: ??
    connectivity_bits: u64,
    visibility_bits: u64,
}

#[derive(Debug, PartialEq)]
pub struct UModel {
    pub vertices: Vec<FVert>,
    pub points: Vec<FVector>,
    pub vectors: Vec<FVector>,
    pub nodes: Vec<FBspNode>,
    pub surfaces: Vec<FBspSurf>,
    pub polys: Vec<FPoly>,
    pub bounds: Vec<FBox>,
    pub leaf_hulls: Vec<usize>,
    pub leaves: Vec<FLeaf>,
    pub zones: Vec<FZoneProperties>,
    pub bounding_sphere: FSphere,
    pub bounding_box: FBox,

    pub linked: bool,
    pub is_root_outside: bool,
}

impl UModel {
    pub fn new() -> UModel {
        UModel {
            vertices: Vec::new(),
            points: Vec::new(),
            vectors: Vec::new(),
            nodes: Vec::new(),
            surfaces: Vec::new(),
            polys: Vec::new(),
            bounds: Vec::new(),
            leaf_hulls: Vec::new(),
            leaves: Vec::new(),
            zones: Vec::new(),
            bounding_sphere: FSphere::default(),
            bounding_box: FBox::default(),
            linked: false,
            is_root_outside: true
        }
    }

    pub fn shrink_model(&mut self) {
        self.vectors.shrink_to_fit();
        self.points.shrink_to_fit();
        self.vertices.shrink_to_fit();
        self.nodes.shrink_to_fit();
        self.surfaces.shrink_to_fit();
        self.polys.shrink_to_fit();
        self.bounds.shrink_to_fit();
        self.leaf_hulls.shrink_to_fit();
    }

    /// Empty the contents of a model.
    pub fn empty_model(&mut self, empty_surface_info: bool, empty_polys: bool) {
        // Ensure all projectors are destroyed.

        /*
        // Ensure all projectors are destroyed
        for( INT i=0; i<Nodes.Num(); i++ )
        {
            FBspNode& Node = Nodes(i);
            INT j;
            while( (j=Node.Projectors.Num()) > 0)
            {
                Node.Projectors(j-1)->RenderInfo->RemoveReference();
                delete Node.Projectors(j-1);
                Node.Projectors.Remove(j-1);		
            }
        }
        */

        self.nodes.clear();
        self.bounds.clear();
        self.leaf_hulls.clear();
        self.leaves.clear();
        self.vertices.clear();
        // self.lights.clear();
        // self.light_maps.clear();
        // self.dynamic_light_maps.clear();
        // self.light_map_textures.clear();
        // self.sections.clear();

        if empty_surface_info {
            self.vectors.clear();
            self.points.clear();
            self.surfaces.clear();
        }

        if empty_polys {
            self.polys.clear();
        }

        /*
        NumZones = 0;
        for( INT i=0; i<FBspNode::MAX_ZONES; i++ )
        {
            Zones[i].ZoneActor    = NULL;
            Zones[i].Connectivity = ((QWORD)1)<<i;
            Zones[i].Visibility   = ~(QWORD)0;
        }	
        */
    }

    /// Build the model's bounds (min and max).
    pub fn build_bound(&mut self) {
        self.bounding_box = FBox::default();
        for poly in &self.polys {
            self.bounding_box.add_points(&poly.vertices);
        }
        self.bounding_sphere = FSphere::from(&self.bounding_box);
    }

    // Find Bsp node vertex nearest to a point (within a certain radius) and
    // set the location.  Returns distance, or -1.f if no point was found.
    pub fn find_nearest_vertex(&self, source_point: FVector, dest_point: &mut FVector, min_radius: f32, vertex_index: &mut usize) -> f32 {
        if self.nodes.is_empty() {
            return -1.0;
        }
        self.find_nearest_vertex_recursive(source_point, dest_point, min_radius, Some(0), vertex_index)
    }

    fn find_nearest_vertex_recursive(&self, source_point: FVector, dest_point: &mut FVector, mut min_radius: f32, node_index: Option<usize>, vertex_index: &mut usize) -> f32 {
        let mut result_radius = -1.0f32;

        let mut next_node_index = node_index;
        while let Some(node_index) = next_node_index {
            let node = &self.nodes[node_index];
            let back_index = node.back_node_index;
            let plane_distance = node.plane.plane_dot(source_point);

            if plane_distance >= -min_radius {
                if let Some(front_node_index) = node.front_node_index {
                    // Check front.
                    let temp_radius = self.find_nearest_vertex_recursive(source_point, dest_point, min_radius, Some(front_node_index), vertex_index);
                    if temp_radius >= 0.0 {
                        result_radius = temp_radius;
                        min_radius = temp_radius;
                    }
                }
            }

            if plane_distance > -min_radius && plane_distance <= min_radius {
                // Check this node's poly's vertices.
                next_node_index = back_index;
                while let Some(node_index) = next_node_index {
                    // Loop through all coplanars.
                    let node = &self.nodes[node_index];
                    let surf = &self.surfaces[node.surface_index];
                    let base = &self.points[surf.base_point_index];
                    let temp_radius_squared = source_point.distance2(*base);

                    if temp_radius_squared < (min_radius * min_radius) {
                        *vertex_index = surf.base_point_index;
                        min_radius = temp_radius_squared.sqrt();
                        result_radius = min_radius;
                        *dest_point = *base;
                    }

                    let vert_pool = &self.vertices[node.vertex_pool_index..(node.vertex_pool_index + node.vertex_count as usize)];
                    for vert in vert_pool {
                        let vertex = &self.points[vert.vertex_index];
                        let temp_radius_squared = source_point.distance2(*vertex);
                        if temp_radius_squared < min_radius * min_radius {
                            *vertex_index = vert.vertex_index;
                            min_radius = temp_radius_squared.sqrt();
                            result_radius = min_radius;
                            *dest_point = *vertex;
                        }
                    }

                    next_node_index = node.plane_index;
                }
            }

            if plane_distance > min_radius {
                break;
            }

            next_node_index = back_index;
        }

        result_radius
    }

    /// Add a new point to the model, merging near-duplicates,  and return its index.
    pub fn bsp_add_point(&mut self, v: FVector, exact: bool) -> usize {
        let thresh = if exact { THRESH_POINTS_ARE_SAME } else { THRESH_POINTS_ARE_NEAR };
        
        // Try to find a match quickly from the Bsp. This finds all potential matches
        // except for any dissociated from nodes/surfaces during a rebuild.
        let mut temp = FVector::new(0.0, 0.0, 0.0);
        let mut vertex_index = 0usize;
        let nearest_distance = self.find_nearest_vertex(v, &mut temp, thresh, &mut vertex_index);

        if nearest_distance >= 0.0 && nearest_distance <= thresh {
            // Found an existing point.
            vertex_index
        } else {
            let fast_rebuild = false;
            add_thing(&mut self.points, v, thresh, fast_rebuild)
        }
    }

    /// Add a new vector to the model, merging near-duplicates,  and return its index.
    pub fn bsp_add_vector(&mut self, v: FVector, is_normal: bool) -> usize {
        add_thing(&mut self.vectors, v, 
            if is_normal { THRESH_NORMALS_ARE_SAME } else { THRESH_VECTORS_ARE_NEAR }, 
            false)
    }
}


/// Add a new point to the model (preventing duplicates) and return its
/// index.
fn add_thing(vectors: &mut Vec<FVector>, v: FVector, threshold: f32, check: bool) -> usize {
    if check {
        for (i, table_vector) in vectors.iter().enumerate() {
            let temp = v.x - table_vector.x;
            if temp > -threshold && temp < threshold {
                let temp = v.y - table_vector.y;
                if temp > -threshold && temp < threshold {
                    let temp = v.z - table_vector.z;
                    if temp > -threshold && temp < threshold {
                        // Found nearly-matching vector.
                        return i
                    }
                }
            }
        }
    }
    vectors.push(v);
    vectors.len() - 1
}