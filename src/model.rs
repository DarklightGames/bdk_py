use bitflags::bitflags;
use crate::fpoly::{EPolyFlags, FPoly};
use crate::math::{FPlane, FVector};
use crate::sphere::FSphere;
use crate::box_::FBox;

pub struct FBspVertex {
    pub position: FVector,
    pub normal: FVector,
    pub u: f32,
    pub v: f32,
    pub u2: f32,
    pub v2: f32,
}

/// Flags associated with a Bsp node.
#[derive(Clone, Debug, PartialEq)]
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
    pub node_indices: Vec<usize>,
    pub plane: FPlane,
    pub light_map_scale: f32
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
    pub collision_bound: usize,
    /// Rendering bound.
    pub render_bound: usize,

    /// Visibility zone in 1=front, 0=back.
    pub zone: [u8;2],
    /// Number of vertices in node.
    pub vertex_count: u8,
    /// Node flags.
    pub node_flags: EBspNodeFlags,
    /// Leaf in back and front, INDEX_NONE = NOT A LEAF.
    pub leaf_indices: [Option<usize>;2],

    pub section_index: usize,
    pub first_vertex_index: usize,
    pub light_map_index: usize,
}

impl FBspNode {

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
        }
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
}