use arrayvec::ArrayVec;
use bitflags::bitflags;
use cgmath::{InnerSpace, Vector3, Zero};
use std::{cmp, rc::Rc};

//
// Lengths of normalized vectors (These are half their maximum values
// to assure that dot products with normalized vectors don't overflow).
//
const FLOAT_NORMAL_THRESH: f32 = 0.0001;

//
// Magic numbers for numerical precision.
//
const THRESH_POINT_ON_PLANE: f32 = 0.10;		/* Thickness of plane for front/back/inside test */
const THRESH_POINT_ON_SIDE: f32 = 0.20;		/* Thickness of polygon side's side-plane for point-inside/outside/on side test */
const THRESH_POINTS_ARE_SAME: f32 = 0.002;	/* Two points are same if within this distance */
const THRESH_POINTS_ARE_NEAR: f32 = 0.015;	/* Two points are near if within this distance and can be combined if imprecise math is ok */
const THRESH_NORMALS_ARE_SAME: f32 = 0.00002;	/* Two normal points are same if within this distance */
												/* Making this too large results in incorrect CSG classification and disaster */
const THRESH_VECTORS_ARE_NEAR: f32 = 0.0004;	/* Two vectors are near if within this distance and can be combined if imprecise math is ok */
												/* Making this too large results in lighting problems due to inaccurate texture coordinates */
const THRESH_SPLIT_POLY_WITH_PLANE: f32 = 0.25;		/* A plane splits a polygon in half */
const THRESH_SPLIT_POLY_PRECISELY: f32 = 0.01;		/* A plane exactly splits a polygon */
const THRESH_ZERO_NORM_SQUARED: f32 = 0.0001;	/* Size of a unit normal that is considered "zero", squared */
const THRESH_VECTORS_ARE_PARALLEL: f32 = 0.02;		/* Vectors are parallel if dot product varies less than this */

pub enum EBspOptimization {
    Lame,
    Good,
    Optimal,
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct EPolyFlags: u32 {
        const None          = 0;

        const Invisible		= 0x00000001;	// Poly is invisible.
        const Masked		= 0x00000002;	// Poly should be drawn masked.
        const Translucent	= 0x00000004;	// Poly is transparent.
        const NotSolid		= 0x00000008;	// Poly is not solid, doesn't block.
        const Environment   = 0x00000010;	// Poly should be drawn environment mapped.
        const SemiSolid	= 0x00000020;	// Poly is semi-solid = collision solid, Csg nonsolid.
        const Modulated 	= 0x00000040;	// Modulation transparency.
        const FakeBackdrop	= 0x00000080;	// Poly looks exactly like backdrop.
        const TwoSided     = 0x00000100;	// Poly is visible from both sides.
        const NoSmooth		= 0x00000800;	// Don't smooth textures.
        const AlphaTexture = 0x00001000;	// Honor texture alpha (reuse BigWavy and SpecialPoly flags)
        const Flat			= 0x00004000;	// Flat surface.
        const NoMerge		= 0x00010000;	// Don't merge poly's nodes before lighting when rendering.
        const NoZTest		= 0x00020000;	// Don't test Z buffer
        const Additive		= 0x00040000;	// sjs - additive blending, (Aliases DirtyShadows).
        const SpecialLit	= 0x00100000;	// Only speciallit lights apply to this poly.
        const Wireframe		= 0x00200000;	// Render as wireframe
        const Unlit			= 0x00400000;	// Unlit.
        const Portal		= 0x04000000;	// Portal between iZones.
        const AntiPortal   = 0x08000000;	// Antiportal
        const Mirrored      = 0x20000000;   // Mirrored BSP surface.

        // Editor flags.
        const Memorized     = 0x01000000;	// Editor: Poly is remembered.
        const Selected      = 0x02000000;	// Editor: Poly is selected.

        // Internal.
        const EdProcessed 	= 0x40000000;	// FPoly was already processed in editorBuildFPolys.
        const EdCut       	= 0x80000000;	// FPoly has been split by SplitPolyWithPlane.

        // Combinations of flags.
        //const NoOcclude		= EPolyFlags::Masked | EPolyFlags::Translucent | EPolyFlags::Invisible | EPolyFlags::Modulated | EPolyFlags::AlphaTexture;
        //const NoEdit			= EPolyFlags.Memorized | EPolyFlags.Selected | EPolyFlags.EdProcessed | EPolyFlags.NoMerge | EPolyFlags.EdCut,
        //const NoImport		= EPolyFlags.NoEdit | EPolyFlags.NoMerge | EPolyFlags.Memorized | EPolyFlags.Selected | EPolyFlags.EdProcessed | EPolyFlags.EdCut,
        //const AddLast			= EPolyFlags.Semisolid | EPolyFlags.NotSolid,
        //const NoAddToBSP		= EPolyFlags.EdCut | EPolyFlags.EdProcessed | EPolyFlags.Selected | EPolyFlags.Memorized,
        //const NoShadows		= EPolyFlags.Unlit | EPolyFlags.Invisible | EPolyFlags.Environment | EPolyFlags.FakeBackdrop
    }
}

pub struct UMaterial {}

pub struct ABrush {
    brush: Option<Box<UModel>>,
}

pub struct FPlane {
    pub normal: Vector3<f32>,
    pub distance: f32,
}

impl FPlane {

    pub fn new(base: Vector3<f32>, normal: Vector3<f32>) -> FPlane {
        let distance = base.dot(normal);
        FPlane {
            normal,
            distance,
        }
    }

    pub fn plane_dot(&self, p: Vector3<f32>) -> f32 {
        self.normal.dot(p) - self.distance
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ESplitType {
    Coplanar = 0, // Poly wasn't split, but is coplanar with plane
    Front = 1,    // Poly wasn't split, but is entirely in front of plane
    Back = 2,     // Poly wasn't split, but is entirely in back of plane
    Split = 3,    // Poly was split into two new editor polygons
}

pub struct FBspSurf {
    material: Option<Rc<UMaterial>>,
    poly_flags: u32,
    base: usize,
    normal_index: Option<usize>,
    texture_u_vector_index: Option<usize>,
    texture_v_vector_index: Option<usize>,
    brush_polygon_index: Option<usize>,
    brush: Rc<ABrush>,
    node_indices: Vec<u32>,
    plane: FPlane,
    light_map_scale: f32,
}

pub fn line_plane_intersection(point1: &Vector3<f32>, point2: &Vector3<f32>, plane: &FPlane) -> Vector3<f32> {
    point1 + (point2 - point1) * ((plane.distance - (point1.dot(plane.normal))) / ((point2 - point1).dot(plane.normal)))
}

pub fn line_plane_intersection_with_base_and_normal(point1: Vector3<f32>, point2: Vector3<f32>, plane_base: Vector3<f32>, plane_normal: Vector3<f32>) -> Vector3<f32> {
    point1 + (point2 - point1) * (((plane_base - point1).dot(plane_normal)) / ((point2 - point1).dot(plane_normal)))
}

/// Compare two points and see if they're the same, using a threshold.
/// Uses fast distance approximation.
pub fn points_are_same(p: &Vector3<f32>, q: &Vector3<f32>) -> bool {
    let mut temp = p.x - q.x;
    if temp > -THRESH_POINTS_ARE_SAME && temp < THRESH_POINTS_ARE_SAME {
        temp = p.y - q.y;
        if temp > -THRESH_POINTS_ARE_SAME && temp < THRESH_POINTS_ARE_SAME {
            temp = p.z - q.z;
            if temp > -THRESH_POINTS_ARE_SAME && temp < THRESH_POINTS_ARE_SAME {
                return true;
            }
        }
    }
    false
}

pub fn point_plane_distance(point: Vector3<f32>, plane_base: Vector3<f32>, plane_normal: Vector3<f32>) -> f32 {
    (point - plane_base).dot(plane_normal)
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum EStatus {
    Front,
    Back,
    Either,
}

const FPOLY_MAX_VERTICES: usize = 16;
const FPOLY_VERTEX_THRESHOLD: usize = FPOLY_MAX_VERTICES - 2;

#[derive(Clone)]
pub struct FPoly {
    pub base: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub texture_u: Vector3<f32>,
    pub texture_v: Vector3<f32>,
    pub vertices: ArrayVec<Vector3<f32>, FPOLY_MAX_VERTICES>,
    pub poly_flags: EPolyFlags,
    pub actor: Option<Rc<ABrush>>,
    pub material: Option<Rc<UMaterial>>,
    pub item_name: String,
    // pub vertex_count: usize,
    pub link: Option<usize>, // iBspSurf, or brush fpoly index of first identical polygon, or MAXWORD.
    pub brush_polygon_index: usize, // Index of editor solid's polygon this originated from.
    pub uv: [Vector3<f32>; 3], // UV coordinates corresponding to the first 3 vertices in the list.  Used by texture alignment tools. (X = U, Y = V, Z = N/A)
    pub smoothing_mask: u32,
    pub light_map_scale: f32,
}

impl FPoly {
    pub fn new() -> FPoly {
        FPoly {
            base: Vector3 { x: 0.0, y: 0.0, z: 0.0, },
            normal: Vector3 { x: 0.0, y: 0.0, z: 0.0, },
            texture_u: Vector3 { x: 0.0, y: 0.0, z: 0.0, },
            texture_v: Vector3 { x: 0.0, y: 0.0, z: 0.0, },
            vertices: ArrayVec::new(),
            poly_flags: EPolyFlags::EdCut,
            actor: None,
            material: None,
            item_name: String::new(),
            link: None,
            brush_polygon_index: 0,
            uv: [Vector3::new(0.0, 0.0, 0.0); 3],
            smoothing_mask: 0,
            light_map_scale: 32.0,
        }
    }

    // Compute normal of an FPoly.  Works even if FPoly has 180-degree-angled sides (which
    // are often created during T-joint elimination).  Returns nonzero result (plus sets
    // normal vector to zero) if a problem occurs.
    pub fn calc_normal(&mut self, silent: bool) -> i32 {
        let v = &mut self.vertices;
        self.normal.set_zero();
        for i in 2..v.len() {
            self.normal += (v[i - 1] - v[0]).cross(v[i] - v[0]);
        }

        if self.normal.magnitude2() < THRESH_ZERO_NORM_SQUARED {
            if !silent {
                eprintln!("FPoly::CalcNormal: Zero area polygon");
            }
            return 1;
        }
        self.normal = self.normal.normalize();
        0
    }

    pub fn new_from_vertices(vertices: &[Vector3<f32>]) -> FPoly {
        let mut poly = FPoly::new();
        let _ = poly.vertices.try_extend_from_slice(vertices);
        // TODO: not sure about the winding order here, this normal calculation may need to be reversed
        poly.normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalize();
        poly
    }

    pub fn split_with_plane(
        &mut self, 
        plane_base: Vector3<f32>,
        plane_normal: Vector3<f32>,
        front_poly: Option<&mut FPoly>,
        back_poly: Option<&mut FPoly>,
        very_precise: bool
    ) -> ESplitType {
        let threshold = if very_precise {
            THRESH_SPLIT_POLY_PRECISELY
        } else {
            THRESH_SPLIT_POLY_WITH_PLANE
        };

        assert!(self.vertices.len() >= 3);
        assert!(self.vertices.len() <= FPOLY_MAX_VERTICES);

        let mut distance = 0.0;
        let mut max_distance = 0.0;
        let mut min_distance = 0.0;
        for i in 0..self.vertices.len() {
            let distance = point_plane_distance(self.vertices[i], plane_base, plane_normal);
            if i == 0 || distance > max_distance {
                max_distance = distance;
            }
            if i == 0 || distance < min_distance {
                min_distance = distance;
            }
        }

        if max_distance < threshold && min_distance > -threshold {
            return ESplitType::Coplanar;
        } else if max_distance < threshold {
            return ESplitType::Back;
        } else if min_distance > -threshold {
            return ESplitType::Front;
        } else {
            // Split.
            if let Some(front_poly) = front_poly {
                if let Some(back_poly) = back_poly {
                    if self.vertices.len() > FPOLY_MAX_VERTICES { // TODO: this isn't possible
                        eprintln!("FPoly::SplitWithPlane: Vertex overflow");
                    }

                    *front_poly = self.clone();
                    front_poly.poly_flags |= EPolyFlags::EdCut;
                    front_poly.vertices.clear();

                    *back_poly = self.clone();
                    back_poly.poly_flags |= EPolyFlags::EdCut;
                    back_poly.vertices.clear();

                    let mut j = self.vertices.len() - 1; // Previous vertex, have PrevStatus already.
                    let mut previous_status = EStatus::Either;
                    let mut previous_distance: f32;
                    for i in 0..self.vertices.len() {
                        previous_distance = distance;
                        distance = point_plane_distance(self.vertices[i], plane_base, plane_normal);

                        let status = if distance > threshold {
                            EStatus::Front
                        } else if distance < -threshold {
                            EStatus::Back
                        } else {
                            previous_status
                        };

                        if status != previous_status {
                            // Crossing. Either front-to-back or back-to-front.
                            // Intersection point is naturally on both front and back polys.
                            if distance >= -threshold && distance < threshold {
                                // This point lies on plane.
                                // TODO; these two cases are literally the same, simplify later after tests have been written
                                if previous_status == EStatus::Front {
                                    front_poly.vertices.push(self.vertices[i]);
                                    back_poly.vertices.push(self.vertices[i]);
                                } else {
                                    back_poly.vertices.push(self.vertices[i]);
                                    front_poly.vertices.push(self.vertices[i]);
                                }
                            } else if previous_distance >= -threshold && previous_distance < threshold {
                                // Previous point lies on plane.
                                if status == EStatus::Front {
                                    front_poly.vertices.push(self.vertices[j]);
                                    front_poly.vertices.push(self.vertices[i]);
                                } else {
                                    back_poly.vertices.push(self.vertices[j]);
                                    back_poly.vertices.push(self.vertices[i]);
                                }
                            } else {
                                let intersection = line_plane_intersection_with_base_and_normal(self.vertices[j], self.vertices[i], plane_base, plane_normal);

                                if previous_status == EStatus::Front {
                                    front_poly.vertices.push(intersection);
                                    back_poly.vertices.push(intersection);
                                    back_poly.vertices.push(self.vertices[i]);
                                } else {
                                    back_poly.vertices.push(intersection);
                                    front_poly.vertices.push(intersection);
                                    front_poly.vertices.push(self.vertices[i]);
                                }
                            }
                        } else {
                            if status == EStatus::Front {
                                front_poly.vertices.push(self.vertices[i]);
                            } else {
                                back_poly.vertices.push(self.vertices[i]);
                            }
                        }
                        j = i;
                        previous_status = status;
                    }

                    // Handle possibility of sliver polys due to precision errors.
                    if front_poly.fix() < 3 {
                        return ESplitType::Back;
                    } else if back_poly.fix() < 3 {
                        return ESplitType::Front;
                    } else {
                        return ESplitType::Split;
                    }
                } else {
                    // TODO: ugly.
                    // Caller only wanted status.
                    return ESplitType::Split;
                }
            } else {
                // Caller only wanted status.
                return ESplitType::Split;
            }
        }
    }

    pub fn reverse(&mut self) {
        self.normal = -self.normal;
        self.vertices.reverse();
    }

    /// Fix up an editor poly by deleting vertices that are identical.
    /// Sets vertex count to zero if it collapses. Returns number of vertices, 0 or >= 3.
    pub fn fix(&mut self) -> usize {
        let mut j = 0;
        let mut prev = self.vertices.len() - 1;

        for i in 0..self.vertices.len() {
            if !points_are_same(&self.vertices[i], &self.vertices[prev]) {
                if j != i {
                    self.vertices[j] = self.vertices[i];
                }
                prev = j;
                j += 1;
            }
        }
        if j >= 3 {
            self.vertices.truncate(j);
        } else {
            self.vertices.clear()
        }
        self.vertices.len()
    }

    pub fn split_in_half(&mut self) -> Self {
        let m = self.vertices.len() / 2;

        if self.vertices.len() <= 3 || self.vertices.len() > FPOLY_MAX_VERTICES {
            eprintln!("Cannot split a polygon with {} vertices", self.vertices.len())
        }

        // bitwise copy via Copy trait
        let mut other_half: FPoly = self.clone();

        // TODO: this whole thing is rather confusing and can probably be simplified.
        other_half.vertices.truncate(self.vertices.len() - m + 1);

        for i in 0..other_half.vertices.len() - 1 { // TODO: are we sure this is -1?
            other_half.vertices[i] = self.vertices[i + m]
        }

        let k = other_half.vertices.len();  // avoids compile error, might be a nicer way to do this (set last somehow)
        other_half.vertices[k - 1] = self.vertices[0];

        self.vertices.truncate(m + 1);

        self.poly_flags |= EPolyFlags::EdCut;
        other_half.poly_flags |= EPolyFlags::EdCut;

        other_half
    }

    pub fn split_with_plane_fast(
        &self,
        plane: FPlane,
        front_poly: Option<&mut FPoly>,
        back_poly: Option<&mut FPoly>,
    ) -> ESplitType {
        let mut vertex_statuses = [EStatus::Front; FPOLY_MAX_VERTICES];
        let mut front = false;
        let mut back = false;

        for i in 0..self.vertices.len() {
            let distance = plane.plane_dot(self.vertices[i]);
            if distance >= 0.0 {
                vertex_statuses[i] = EStatus::Front;
                if distance > THRESH_SPLIT_POLY_WITH_PLANE {
                    front = true
                }
            } else {
                vertex_statuses[i] = EStatus::Back;
                if distance < -THRESH_SPLIT_POLY_WITH_PLANE {
                    back = true;
                }
            }
        }

        if !front {
            return if back {
                ESplitType::Back
            } else {
                ESplitType::Coplanar
            };
        }

        if !back {
            return ESplitType::Front;
        }

        if let Some(front_poly) = front_poly {
            if let Some(back_poly) = back_poly {
                let mut wi = self.vertices.len() - 1;
                let mut previous_status = vertex_statuses[wi];
                for i in 0..self.vertices.len() {
                    let status = vertex_statuses[i];
                    if status != previous_status {
                        // Crossing.
                        let intersection = line_plane_intersection(&self.vertices[wi], &self.vertices[i], &plane);
                        front_poly.vertices.push(intersection);
                        back_poly.vertices.push(intersection);

                        if previous_status == EStatus::Front {
                            back_poly.vertices.push(self.vertices[i]);
                        } else {
                            front_poly.vertices.push(self.vertices[i]);
                        }
                    } else if status == EStatus::Front {
                        front_poly.vertices.push(self.vertices[i]);
                    } else {
                        back_poly.vertices.push(self.vertices[i]);
                    }
                    previous_status = status;
                    wi = i;
                }

                front_poly.base = self.base;
                front_poly.normal = self.normal;
                front_poly.poly_flags = self.poly_flags;
                
                back_poly.base = self.base;
                back_poly.normal = self.normal;
                back_poly.poly_flags = self.poly_flags;
            }
        }
        return ESplitType::Split;
    }
}

const MAX_NODE_VERTICES: usize = 16;
const MAX_FINAL_VERTICES: usize = 24;
const MAX_ZONES: usize = 64;

pub struct FSphere {
    origin: Vector3<f32>,
    radius: f32,
}

pub struct FBspNode {
    pub plane: FPlane,
    pub zone_mask: i64,
    pub vertex_pool_index: usize,
    pub surface_index: usize,
    pub back_node_index: Option<usize>,   // Index to the node in front (in the direction of the normal) (union with child_index)??
    pub front_node_index: Option<usize>,  // Index to the node in back (opposite direction of the normal)
    pub plane_index: Option<usize>,   // Index to the next coplanar poly in the coplanar list
    pub exclusive_sphere_bound: FSphere,
    pub collision_bound_index: usize,
    pub render_bound_index: usize,
    pub zone_indices: [u8; 2],
    pub vertex_count: u8,
    pub node_flags: EBspNodeFlags,
    pub leaf_indices: [Option<i32>; 2],
    pub section_index: i32,
    pub section_vertex_index: i32,
    pub light_map_index: i32
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct EBspNodeFlags : u8 {
        const None = 0;
        const NotCSG = 0x0001; // Node is not a Csg splitter, i.e. is a transparent poly.
        const ShootThrough = 0x0002; // Can shoot through (for projectile solid ops).
        const NotVisBlocking = 0x0004; // Node does not block visibility, i.e. is an invisible collision hull.
        const PolyOccluded = 0x0008; // Node's poly was occluded on the previously-drawn frame.
        const BoxOccluded = 0x0010; // Node's bounding box was occluded.
        const BrightCorners = 0x0010; // Temporary.
        const IsNew = 0x0020; // Editor: Node was newly-added.
        const IsFront = 0x0040; // Filter operation bounding-sphere precomputed and guaranteed to be front.
        const IsBack = 0x0080; // Guaranteed back.
        //const USE_COLLISION_STATIC_MESH = 0x0100; // Use collision static mesh for traces against skeletal meshes. (commented out because the engine doesn't use this for the CSG process)
    }
}

impl FBspNode {

    pub fn is_csg(&self, extra_flags: EBspNodeFlags) -> bool {
        self.vertex_count > 0 && (self.node_flags & (EBspNodeFlags::IsNew | EBspNodeFlags::NotCSG | extra_flags)) == EBspNodeFlags::None
    }

    pub fn is_child_outside(&self, child: bool, outside: bool, extra_flags: EBspNodeFlags) -> bool {
        if child {
            outside || self.is_csg(extra_flags)
        } else {
            outside && !self.is_csg(extra_flags)
        }
    }
}

pub struct FBox {
    min: Vector3<f32>,
    max: Vector3<f32>,
}

pub struct FLeaf {
    zone_index: usize,          // The zone this convex volume is in.
    permeating: usize,          // Lights permeating this volume considering shadowing.
    volumentric: usize,         // Volumetric lights hitting this region, no shadowing.
    visible_zones_mask: u64     // Bit mask of visible zones from this convex volume.
}

struct FBspVertexStream {

}

pub struct FBspSection {
    vertices: FBspVertexStream,
    node_count: usize,
    material: Rc<UMaterial>,
    poly_flags: EPolyFlags,
    light_map_texture: Option<usize>
}

pub struct FLightMapTexture {

}

pub struct FLightMap {

}

pub struct FZoneProperties {

}

// One vertex associated with a Bsp node's polygon.  Contains a vertex index
// into the level's FPoints table, and a unique number which is common to all
// other sides in the level which are cospatial with this side.
pub struct FVert {
    pub vertex_index: usize,
    pub side: Option<i32>,      // If shared, index of unique side. Otherwise INDEX_NONE.
}

pub struct UModel {
    polys: Vec<FPoly>,
    nodes: Vec<FBspNode>,
    verts: Vec<FVert>,
    vectors: Vec<Vector3<f32>>,
    points: Vec<Vector3<f32>>,
    surfs: Vec<FBspSurf>,
    bounds: Vec<FBox>,
    leaf_hulls: Vec<i32>,
    leaves: Vec<FLeaf>,
    sections: Vec<FBspSection>,

    // some lightmap related stuff is also in here, but we don't need it for now

    root_outside: bool,
    linked: bool,
    shared_side_count: i32,
    zones: ArrayVec<FZoneProperties, MAX_ZONES>
}

impl UModel {

    pub fn shrink_model(&mut self) {
        self.vectors.shrink_to_fit();
        self.points.shrink_to_fit();
        self.verts.shrink_to_fit();
        self.nodes.shrink_to_fit();
        self.surfs.shrink_to_fit();
        self.polys.shrink_to_fit();
        self.bounds.shrink_to_fit();
        self.leaf_hulls.shrink_to_fit();
    }

    // Empty the contents of a model.
    pub fn empty_model(&mut self, empty_surface_info: bool, empty_polygons: bool) {
        self.nodes.clear();
        self.bounds.clear();
        self.leaf_hulls.clear();
        self.leaves.clear();
        self.verts.clear();
        //self.lights.clear();
        //self.light_maps.clear();
        //self.dynamic_light_maps.clear();
        //self.light_map_textures.clear();
        self.sections.clear();

        if empty_surface_info {
            self.vectors.clear();
            self.points.clear();
            self.surfs.clear();
        }

        if empty_polygons {
            self.polys.clear();
        }

        self.zones.clear();
        self.shared_side_count = 0;
    }
}


#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ECsgOper {
    Add,
    Subtract
}

pub enum EPolyNodeFilter {
    Outside				= 0, // Leaf is an exterior leaf (visible to viewers).
    Inside              = 1, // Leaf is an interior leaf (non-visible, hidden behind backface).
    CoplanarOutside 	= 2, // Poly is coplanar and in the exterior (visible to viewers).
    CoplanarInside		= 3, // Poly is coplanar and inside (invisible to viewers).
    CospatialFacingIn	= 4, // Poly is coplanar, cospatial, and facing in.
    CospatialFacingOut  = 5, // Poly is coplanar, cospatial, and facing out.
}

pub enum ENodePlace {
	Back		= 0, // Node is in back of parent              -> Bsp[iParent].iBack.
	Front		= 1, // Node is in front of parent             -> Bsp[iParent].iFront.
	Plane		= 2, // Node is coplanar with parent           -> Bsp[iParent].iPlane.
	Root		= 3, // Node is the Bsp root and has no parent -> Bsp[0].
}

#[derive(Clone)]
pub struct FCoplanarInfo {
    original_node_index: Option<usize>,
	back_node_index: Option<usize>,
	back_node_outside: bool,   // TODO; these might be bools
	front_leaf_outside: bool,
	processing_back: bool
}

impl FCoplanarInfo {
    pub fn new() -> FCoplanarInfo {
        FCoplanarInfo {
            original_node_index: None,
            back_node_index: None,
            back_node_outside: false,
            front_leaf_outside: false,
            processing_back: false
        }
    }
}

type BspFilterFunc = fn(model: &UModel, node_index: usize, ed_poly: &FPoly, leaf: EPolyNodeFilter, node_place: ENodePlace);

pub fn add_brush_to_world_func(model: &UModel, node_index: usize, ed_poly: &FPoly, leaf: EPolyNodeFilter, node_place: ENodePlace) {

}

pub fn sutract_brush_from_world_func(model: &UModel, node_index: usize, ed_poly: &FPoly, leaf: EPolyNodeFilter, node_place: ENodePlace) {

}

pub fn filter_leaf(filter_func: BspFilterFunc, model: &UModel, node_index: usize, ed_poly: &FPoly, mut coplanar_info: FCoplanarInfo, mut leaf_outside: bool, place: ENodePlace) {
    
    if coplanar_info.original_node_index.is_none() {
        // Processing regular, non-coplanar polygons.
        let filter_type = if leaf_outside {
            EPolyNodeFilter::Outside
        } else {
            EPolyNodeFilter::Inside
        };
        filter_func(model, node_index, ed_poly, filter_type, place);
    } else if coplanar_info.processing_back {
		// Finished filtering polygon through tree in back of parent coplanar.
        //DoneFilteringBack:
        let filter_type = if !leaf_outside && !coplanar_info.front_leaf_outside {
            EPolyNodeFilter::CoplanarInside
        } else if leaf_outside && coplanar_info.front_leaf_outside {
            EPolyNodeFilter::CoplanarOutside
        } else if !leaf_outside && coplanar_info.front_leaf_outside {
            EPolyNodeFilter::CospatialFacingOut
        } else  {
            EPolyNodeFilter::CospatialFacingIn
        };
        filter_func(model, coplanar_info.original_node_index.unwrap(), ed_poly, filter_type, ENodePlace::Plane);
    } else {
        coplanar_info.front_leaf_outside = leaf_outside;

        match coplanar_info.back_node_index {
            None => {
                // Back tree is empty.
                leaf_outside = coplanar_info.back_node_outside;
                //goto DoneFilteringBlock;
            }
            Some(back_node_index) => {
                // Call FilterEdPoly to filter through the back.  This will result in
                // another call to FilterLeaf with iNode = leaf this falls into in the
                // back tree and EdPoly = the final EdPoly to insert.
                coplanar_info.processing_back = true;
                filter_ed_poly(filter_func, model, back_node_index, ed_poly, coplanar_info, coplanar_info.back_node_outside);
            }
        }
    }
}

// Filter an EdPoly through the Bsp recursively, calling FilterFunc
// for all chunks that fall into leaves.  FCoplanarInfo is used to
// handle the tricky case of double-recursion for polys that must be
// filtered through a node's front, then filtered through the node's back,
// in order to handle coplanar CSG properly.
pub fn filter_ed_poly(filter_func: BspFilterFunc, model: &UModel, mut node_index: usize, ed_poly: &FPoly, mut coplanar_info: FCoplanarInfo, mut outside: bool) {
    let mut split_result = 0; // TODO: not sure what this is actually.
    let mut our_front_node_index: Option<usize> = None;
    let mut our_back_node_index: Option<usize> = None;
    let mut new_front_outside = false;
    let mut new_back_outside = false;

    //FilterLoop:
    if ed_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
        // Split EdPoly in half to prevent vertices from overflowing.
        let temp = ed_poly.split_in_half();

        // Filter other half.
        filter_ed_poly(filter_func, model, node_index, &temp, coplanar_info, outside);
    }

    // Split em.
    let plane_base = model.points[model.verts[model.nodes[node_index].vertex_pool_index].vertex_index];
    let plane_normal = model.vectors[model.surfs[model.nodes[node_index].surface_index].normal_index.unwrap()];
    let mut temp_front_ed_poly = FPoly::new();
    let mut temp_back_ed_poly = FPoly::new();
    let split_result = ed_poly.split_with_plane(plane_base, plane_normal, Some(&mut temp_front_ed_poly), Some(&mut temp_back_ed_poly), false);

    match split_result {
        ESplitType::Front => {
            //Front:
            let node = &model.nodes[node_index];
            let outside = outside || node.is_csg(EBspNodeFlags::None);

            match node.front_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, outside, ENodePlace::Front);
                }
                Some(front_node_index) => {
                    node_index = front_node_index
                },
            }
        }
        ESplitType::Back => {
            let node = &model.nodes[node_index];
            let outside = outside && !node.is_csg(EBspNodeFlags::None);

            match node.back_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info, outside, ENodePlace::Back)
                }
                Some(back_node_index) => {
                    node_index = back_node_index
                    // goto FilterLoop;
                }
            }
        }
        ESplitType::Coplanar => {
            if coplanar_info.original_node_index.is_some() {
                // This will happen once in a blue moon when a polygon is barely outside the
                // coplanar threshold and is split up into a new polygon that is
                // is barely inside the coplanar threshold.  To handle this, just classify
                // it as front and it will be handled propery.
                println!("FilterEdPoly: Encountered out-of-place coplanar");
                // GOTO Front;
            }

            coplanar_info.original_node_index = Some(node_index);
            coplanar_info.back_node_index = None;
            coplanar_info.processing_back = false;
            coplanar_info.back_node_outside = outside;
            new_front_outside = outside;

            // See whether Node's iFront or iBack points to the side of the tree on the front
            // of this polygon (will be as expected if this polygon is facing the same
            // way as first coplanar in link, otherwise opposite).
            let node = model.nodes[node_index];
            if node.plane.normal.dot(ed_poly.normal) >= 0.0 {
                our_front_node_index = node.front_node_index;
                our_back_node_index = node.back_node_index;

                if node.is_csg(EBspNodeFlags::None) {
                    coplanar_info.back_node_outside = false;
                    new_front_outside = true;
                }
            } else {
                our_front_node_index = node.back_node_index;
                our_back_node_index =  node.front_node_index;

                if node.is_csg(EBspNodeFlags::None) {
                    coplanar_info.back_node_outside = true;
                    new_front_outside = false;
                }
            }

            // Process front and back.
            if our_front_node_index.is_none() && our_back_node_index.is_none() {
                coplanar_info.processing_back = true;
                coplanar_info.front_leaf_outside = new_front_outside;

                filter_leaf(
                    filter_func, model, node_index, ed_poly, coplanar_info, coplanar_info.back_node_outside, ENodePlace::Plane
                );
            } else if our_front_node_index.is_none() && our_back_node_index.is_some() {
                // Back but no front.
                coplanar_info.processing_back = true;
                coplanar_info.back_node_index = our_back_node_index;
                coplanar_info.front_leaf_outside = new_front_outside;

                node_index = our_back_node_index.unwrap();
                outside = coplanar_info.back_node_outside;
                // GOTO FilterLoop;
            } else {
                // Has a front and maybe a back.

                // Set iOurBack up to process back on next call to FilterLeaf, and loop
                // to process front.  Next call to FilterLeaf will set FrontLeafOutside.
                coplanar_info.processing_back = false;

                // May be a node or may be INDEX_NONE.
                coplanar_info.back_node_index = our_back_node_index;

                node_index = our_front_node_index.unwrap();
                outside = new_front_outside;
                // GOTO FilterLoop;
            }
        }
        ESplitType::Split => {
            let node = &model.nodes[node_index];
            if node.is_csg(EBspNodeFlags::None) {
                new_front_outside = true;
                new_back_outside = false;
            } else {
                new_front_outside = outside;
                new_back_outside = outside;
            }

            // Front half of split.
            match node.front_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, &temp_front_ed_poly, coplanar_info, new_front_outside, ENodePlace::Front);
                }
                Some(front_node_index) => {
                    filter_ed_poly(filter_func, model, front_node_index, &temp_front_ed_poly, coplanar_info, new_front_outside)
                }
            }

            // Back half of split.
            match node.back_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, &temp_back_ed_poly, coplanar_info, new_back_outside, ENodePlace::Back);
                }
                Some(back_node_index) => {
                    filter_ed_poly(filter_func, model, back_node_index, &temp_back_ed_poly, coplanar_info, new_back_outside);
                }
            }
        }
    }
}

// Regular entry into FilterEdPoly (so higher-level callers don't have to
// deal with unnecessary info). Filters starting at root.
pub fn bsp_filter_fpoly(filter_func: BspFilterFunc, model: &UModel, ed_poly: &FPoly) {
    let starting_coplanar_info = FCoplanarInfo::new();

    if model.nodes.is_empty() {
        // If BSP is empty, process at root.
        let filter_type = if model.root_outside {
            EPolyNodeFilter::Outside
        } else {
            EPolyNodeFilter::Inside
        };
        filter_func(model, 0, ed_poly, filter_type, ENodePlace::Root);
    } else {
        // Filter through Bsp.
        filter_ed_poly(filter_func, model, 0, ed_poly, starting_coplanar_info, model.root_outside);
    }
}

struct BspBuilder {
    temp_model: UModel,
}

const INDEX_NONE: i32 = -1; // TODO: remove this eventually

/*---------------------------------------------------------------------------------------
   Bsp point/vector refreshing.
---------------------------------------------------------------------------------------*/

pub fn tag_referenced_nodes(model: &mut UModel, node_ref: &mut Vec<Option<usize>>, poly_ref: &mut Vec<Option<usize>>, node_index: usize) {
    let mut node = model.nodes[node_index];

    node_ref[node_index] = Some(0);
    poly_ref[node.surface_index] = Some(0);

	if let Some(front_node_index) = node.front_node_index {
        tag_referenced_nodes(model, node_ref, poly_ref, front_node_index);
    }
	if let Some(back_node_index) = node.back_node_index {
        tag_referenced_nodes(model, node_ref, poly_ref, back_node_index);
    }
    if let Some(plane_index) = node.plane_index {
        tag_referenced_nodes(model, node_ref, poly_ref, plane_index);
    }
}

// If the Bsp's point and vector tables are nearly full, reorder them and delete
// unused ones:
pub fn bsp_refresh(model: &mut UModel, no_remap_surfs: bool) {
    let mut node_ref: Vec<Option<usize>> = vec![None; model.nodes.len()];
    let mut poly_ref: Vec<Option<usize>> = vec![None; model.polys.len()];

    // Remove unreferenced Bsp surfs.
    if !model.nodes.is_empty() {
        tag_referenced_nodes(model, &mut node_ref, &mut poly_ref, 0);
    }

    if no_remap_surfs {
        poly_ref.fill(Some(0));
    }

    // Remap Bsp nodes and surfs.
    let mut n = 0;
    for i in 0..model.surfs.len() {
        if poly_ref[i].is_some() {
            model.surfs[n] = model.surfs[i];
            poly_ref[i] = Some(n);
            n += 1;
        }
    }
    model.surfs.truncate(n);

    n = 0;
    for i in 0..model.nodes.len() {
        if node_ref[i].is_some() {
            model.nodes[n] = model.nodes[i];
            node_ref[i] = Some(n);
            n += 1;
        }
    }
    model.nodes.truncate(n);

    // Update Bsp nodes.
    for i in 0..model.nodes.len() {
        let node = &mut model.nodes[i];
        // There's some sketchy shit going on here. The poly_ref can potentially be None, but it's being unwrapped unconditionally.
        node.surface_index = poly_ref[node.surface_index].unwrap();  // TODO: this can potentially be None
        if let Some(front_node_index) = node.front_node_index {
            node.front_node_index = node_ref[front_node_index];
        }
        if let Some(back_node_index) = node.back_node_index {
            node.back_node_index = node_ref[back_node_index];
        }
        if let Some(plane_index) = node.plane_index {
            node.plane_index = node_ref[plane_index];
        }
    }

    // Remove unreferenced points and vectors.
    let mut vector_ref: Vec<Option<usize>> = vec![None; model.vectors.len()]; // TODO: this is actually NEGATIVE one (it fills with ones, bitwise, which ends up being -1)
    let mut point_ref: Vec<Option<usize>> = vec![None; model.points.len()];

    // Check Bsp surfs.
    for surf in &model.surfs {
        // TODO: hail-mary unwraps
        vector_ref[surf.normal_index.unwrap()] = Some(0);
        vector_ref[surf.texture_u_vector_index.unwrap()] = Some(0);
        vector_ref[surf.texture_v_vector_index.unwrap()] = Some(0);
        point_ref[surf.base] = Some(0);
    }

    // Check Bsp nodes.
    for node in &model.nodes {
        // Tag all points used by nodes.
        let mut vertex_pool_index = node.vertex_pool_index;

        for b in 0..node.vertex_count {
            point_ref[model.verts[vertex_pool_index + b as usize].vertex_index] = Some(0);
        }
    }

    // Remap points.
    n = 0;
    for i in 0..model.points.len() {
        if point_ref[i].is_some() {
            model.points[n] = model.points[i];
            point_ref[i] = Some(n);
            n += 1;
        }
    }
    model.points.truncate(n);

    // Remap vectors.
    n = 0;
    for i in 0..model.vectors.len() {
        if vector_ref[i].is_some() {
            model.vectors[n] = model.vectors[i];
            vector_ref[i] = Some(n);
            n += 1;
        }
    }
    model.vectors.truncate(n);

    // Update Bsp surfs.
    for surf in &mut model.surfs {
        surf.normal_index = vector_ref[surf.normal_index.unwrap()];
        surf.texture_u_vector_index = vector_ref[surf.texture_u_vector_index.unwrap()];
        surf.texture_v_vector_index = vector_ref[surf.texture_v_vector_index.unwrap()];
        surf.base = point_ref[surf.base].unwrap();
    }

    // Update Bsp nodes.
    for node in model.nodes {
        let vertex_index = node.vertex_pool_index;
        for b in 0..node.vertex_count {
            let vert = &mut model.verts[vertex_index + b as usize];
            vert.vertex_index = point_ref[vert.vertex_index].unwrap();
        }
    }

    // Shrink the objects.
    model.shrink_model();

}

pub fn find_best_split<'a>(poly_list: &[&'a FPoly], optimization: EBspOptimization, balance: usize, portal_bias: f32) -> &'a FPoly {
    assert!(poly_list.len() > 0);
    if poly_list.len() == 1 {
        return poly_list[0];
    }

    let mut best = None;
    let mut score = 0.0;

    let inc = match optimization {   // original called "inc" (increment, probably)
        EBspOptimization::Optimal => 1,                                             // Test lots of nodes.
        EBspOptimization::Good => std::cmp::max(1, poly_list.len() / 20),   // Test 20 nodes.
        EBspOptimization::Lame => std::cmp::max(1, poly_list.len() / 4),    // Test 4 nodes.
    };

    // See if there are any semisolid polygons here.
    let all_semi_solids = poly_list.iter().all(|poly| !poly.poly_flags.intersects(EPolyFlags::SemiSolid | EPolyFlags::NotSolid));

	// Search through all polygons in the pool and find:
	// A. The number of splits each poly would make.
	// B. The number of front and back nodes the polygon would create.
	// C. Number of coplanars.
    let mut best_score = 0.0;
    for i in (0..poly_list.len()).step_by(inc) {
        let mut splits: usize = 0;
        let mut front: usize = 0;
        let mut back: usize = 0;
        let mut coplanar: usize = 0;

        // NOTE: this loop structure is whack as hell.
        let mut index: i32 = i as i32 - 1;

        loop {
            index += 1;
            let poly = poly_list[index as usize];

            // TODO: this is complex and is prone to translation errors
            if index >= (i + inc) as i32 || index >= poly_list.len() as i32 || 
            (!poly.poly_flags.intersects(EPolyFlags::SemiSolid | EPolyFlags::NotSolid) || poly.poly_flags.contains(EPolyFlags::Portal)) ||
            all_semi_solids {
                break;
            }
        }

        if index >= (i + inc) as i32 || index >= poly_list.len() as i32 {
            continue;
        }

        let poly = poly_list[index as usize];

        for j in (0..poly_list.len()).step_by(inc) {
            if j == index as usize {
                continue;
            }
            let other_poly = poly_list[j];
            let plane = FPlane::new(*poly.vertices.first().unwrap(), poly.normal); // TODO: Fplane from origin and normal
            match other_poly.split_with_plane_fast(plane, None, None) {
                ESplitType::Coplanar => {
                    coplanar += 1;
                }
                ESplitType::Front => {
                    front += 1;
                }
                ESplitType::Back => {
                    back += 1;
                }
                ESplitType::Split => {
					// Disfavor splitting polys that are zone portals.
                    if !other_poly.poly_flags.contains(EPolyFlags::Portal) {
                        splits += 1;
                    } else {
                        splits += 16;
                    }
                }
            }
        }

        let mut score = (100.0 - balance as f32) * splits as f32 + balance as f32 * (front as i32 - back as i32).abs() as f32;

        if poly.poly_flags.contains(EPolyFlags::Portal) {
			// PortalBias -- added by Legend on 4/12/2000
			//
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

        if score < best_score || best.is_none() {
            best = Some(poly);
            best_score = score;
        }
    }
    return best.unwrap();
}

pub fn split_poly_list(
    model: &mut UModel, 
    parent_node_index: Option<usize>, 
    node_place: ENodePlace, 
    poly_count: usize, 
    poly_list: Vec<&FPoly>, 
    optimization: EBspOptimization, 
    balance: usize, 
    portal_bias: f32, 
    rebuild_simple_polys: bool) {
    let num_polys_to_alloc = poly_count + 8 + poly_count / 4;
    let front_list: Vec<&FPoly> = Vec::with_capacity(num_polys_to_alloc);
    let back_list: Vec<&FPoly> = Vec::with_capacity(num_polys_to_alloc);

    let split_poly = find_best_split(poly_list.as_slice(), optimization, balance, portal_bias);
}


// Build Bsp from the editor polygon set (EdPolys) of a model.
//
// Opt     = Bsp optimization, BSP_Lame (fast), BSP_Good (medium), BSP_Optimal (slow)
// Balance = 0-100, 0=only worry about minimizing splits, 100=only balance tree.
pub fn bsp_build(model: &mut UModel, optimization: EBspOptimization, balance: usize, portal_bias: f32, rebuild_simple_polys: bool, node_index: usize) {
    let original_polys = model.polys.len();

	// Empty the model's tables.
    if rebuild_simple_polys {
        model.empty_model(true, false);
    } else {
        // Empty node vertices.
        for node in &mut model.nodes {
            node.vertex_count = 0;  // TODO: ??? 
        }

        // Refresh the Bsp.
        bsp_refresh(model, true);

        // Empt nodes.
        model.empty_model(false, false);
    }

    if !model.polys.is_empty() {
        // Allocate polygon pool.
        let mut poly_list: Vec<&FPoly> = Vec::with_capacity(model.polys.len());

        // Add all FPolys to active list.   // TODO: I think it's storing an array of FPoly pointers, and any polys with no vertices[??] are excluded.
        for i in 0..model.polys.len() {
            if !model.polys[i].vertices.is_empty() {
                poly_list.push(&model.polys[i]);
            }
        }

        // Now split the entire Bsp by splitting the list of all polygons.
        split_poly_list(model, None, ENodePlace::Root, model.polys.len(), poly_list, optimization, balance, portal_bias, rebuild_simple_polys);

        if !rebuild_simple_polys {
            // Remove unreferenced things.
            bsp_refresh(model, true);

            // Rebuild all bounding boxes.
            bsp_build_bounds(model);
        }
    }

    // model.clear_render_data <- irrelevant

    println!("bspBuild built {} convex polys into {} nodes", original_polys, model.nodes.len());

}

impl BspBuilder {

    pub fn bsp_brush_csg(&mut self, actor: Rc<ABrush>, model: &mut UModel, poly_flags: EPolyFlags, csg_operation: ECsgOper, build_bounds: i32, merge_polygons: bool) {
        let mut poly_flags_mask = EPolyFlags::None;
        let mut num_polys_from_brush = 0;
        let mut i = 0;
        let mut j = 0;
        let mut really_big = false;
    
        // TODO: dicey
        let mut brush = &actor.brush.as_ref().unwrap();
    
        let mut g_errors = 0;
    
        if csg_operation != ECsgOper::Add {
            poly_flags_mask |= EPolyFlags::SemiSolid | EPolyFlags::NotSolid;
        }
    
        if csg_operation == ECsgOper::Subtract {
            model.zones.clear();
        }
    
        // Prep the temporary model.
        self.temp_model.empty_model(true, true);

        // Build the brush's coordinate system and find orientation of scale
        // transform (if negative, edpolyTransform will reverse the clockness
        // of the EdPoly points and invert the normal).
        // TODO: fill this in, not really sure what it's doing atm.
        
        // Transform original brush poly into same coordinate system as world
        // so Bsp filtering operations make sense.
        for i in 0..brush.polys.len() {
            let mut poly = &brush.polys[i];

            if poly.material.is_none() {
                poly.material = current_material;   // TODO: where is this coming from?
            }

            let dest_ed_poly = poly.clone();

            // Set its backwards brush link.
            dest_ed_poly.actor = Some(actor);
            dest_ed_poly.brush_polygon_index = i;

            // Update its flags.
            dest_ed_poly.poly_flags = (dest_ed_poly.poly_flags | poly_flags) & !poly_flags_mask;

            // Set its internal link.
            if dest_ed_poly.link.is_none() {
                dest_ed_poly.link = Some(i)
            }

            // Transform it.
            // DestEdPoly.Transform( Coords, Actor->PrePivot, Actor->Location, Orientation );
            // Presumably this is just moving it all into world space. We could just make the brush
            // world space to begin with.

            // Add poly to the temporary model.
            self.temp_model.polys.push(dest_ed_poly);
        }

        for i in 0..brush.polys.len() {
            let mut ed_poly = self.temp_model.polys[i];

         	// Mark the polygon as non-cut so that it won't be harmed unless it must
         	// be split, and set iLink so that BspAddNode will know to add its information
         	// if a node is added based on this poly.
            ed_poly.poly_flags &= !EPolyFlags::EdCut;
            if let Some(ed_poly_link) = ed_poly.link {
                let surf_count = model.surfs.len();  // TODO: maybe just convert the values to usize
                if ed_poly_link == i {
                    self.temp_model.polys[i].link = Some(surf_count);
                    ed_poly.link = Some(surf_count);
                } else {
                    // TODO: unwrap is a little dicey
                    ed_poly.link = self.temp_model.polys[ed_poly_link].link;
                }

                let bsp_filter_function = match csg_operation {
                    ECsgOper::Add => add_brush_to_world_func,
                    ECsgOper::Subtract => sutract_brush_from_world_func
                };
                
                bsp_filter_fpoly(bsp_filter_function, model, &ed_poly);
            }
        }

        if !model.nodes.is_empty() && !poly_flags.intersects(EPolyFlags::NotSolid | EPolyFlags::SemiSolid) {
            // Quickly build a Bsp for the brush, tending to minimize splits rather than balance
            // the tree.  We only need the cutting planes, though the entire Bsp struct (polys and
            // all) is built.
            bsp_build(self.temp_model, EBspOptimization::Lame, 0, 70, true, false);

            gmodel = brush;

            self.temp_model.build_bound();  // TODO: this is called twice for some reason in the original code.

            filter_world_through_brush(model, &self.temp_model, csg_operation, 0, self.temp_model.bounding_shere);
        }

		// Clean up nodes, reset node flags.
        bsp_cleanup(model);

		// Rebuild bounding volumes.
        if build_bounds {
            bsp_build_bounds(model);
        }

	    // Release TempModel.
        self.temp_model.empty_model(true, true);

        return 1 + g_errors;

    }
}


