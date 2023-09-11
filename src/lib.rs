use arrayvec::ArrayVec;
use bitflags::bitflags;
use cgmath::{InnerSpace, Vector3, Zero, Bounded};
// use core::slice::SlicePattern;
use std::{cmp, rc::Rc, f32::consts::E, result};

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

#[derive(Clone)]
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

#[derive(Clone)]
pub struct FBspSurf {
    material: Option<Rc<UMaterial>>,
    poly_flags: EPolyFlags,
    base: usize,
    actor: Option<Rc<ABrush>>,
    normal_index: Option<usize>,
    texture_u_vector_index: Option<usize>,
    texture_v_vector_index: Option<usize>,
    brush_polygon_index: Option<usize>,
    brush: Rc<ABrush>,
    node_indices: Vec<u32>,
    plane: FPlane,
    light_map_scale: f32,
}

impl FBspSurf {
    pub fn new() -> FBspSurf {
        FBspSurf {
            material: None,
            poly_flags: EPolyFlags::None,
            base: 0,
            actor: None,
            normal_index: None,
            texture_u_vector_index: None,
            texture_v_vector_index: None,
            brush_polygon_index: None,
            brush: Rc::new(ABrush { brush: None }),
            node_indices: Vec::new(),
            plane: FPlane::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)),
            light_map_scale: 0.0,
        }
    }
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
    pub brush_polygon_index: Option<usize>, // Index of editor solid's polygon this originated from.
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
            item_name: String::new(),   // TODO: if this is never used just remove it
            link: None,
            brush_polygon_index: None,
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
        poly.base = vertices[0];
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

#[derive(Clone)]
pub struct FSphere {
    origin: Vector3<f32>,
    radius: f32,
}

impl FSphere {
    // Compute a bounding sphere from an array of points.
    pub fn new_from_points(points: &[Vector3<f32>]) -> FSphere {
        if points.is_empty() {
            FSphere { origin: Vector3::zero(), radius: 0.0 }
        } else {
            let box_ = FBox::new_from_points(points);
            let mut sphere = FSphere { origin: (box_.min + box_.max) * 0.5, radius: 0.0 };
            for point in points {
                let dist = (point - sphere.origin).magnitude2();
                if dist > sphere.radius {
                    sphere.radius = dist;
                }
            }
            // NOTE: The 1.001 is probably to avoid precision issues.
            sphere.radius = sphere.radius.sqrt() * 1.001;
            sphere
        }
    }
}

#[derive(Clone)]
pub struct FBspNode {
    pub plane: FPlane,
    pub zone_mask: i64,
    pub vertex_pool_index: usize,
    pub surface_index: usize,
    pub back_node_index: Option<usize>,   // Index to the node in front (in the direction of the normal) (union with child_index)??
    pub front_node_index: Option<usize>,  // Index to the node in back (opposite direction of the normal)
    pub plane_index: Option<usize>,   // Index to the next coplanar poly in the coplanar list
    pub exclusive_sphere_bound: FSphere,
    pub collision_bound_index: Option<usize>,
    pub render_bound_index: Option<usize>,
    pub zone_indices: [u8; 2],
    pub vertex_count: u8,
    pub node_flags: EBspNodeFlags,
    pub leaf_indices: [Option<i32>; 2],
    pub section_index: Option<usize>,
    pub section_vertex_index: i32,
    pub light_map_index: Option<usize>
}

impl FBspNode {
    fn new() -> FBspNode {
        FBspNode {
            plane: FPlane::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)),
            zone_mask: 0,
            vertex_pool_index: 0,
            surface_index: 0,
            back_node_index: None,
            front_node_index: None,
            plane_index: None,
            exclusive_sphere_bound: FSphere { origin: Vector3::new(0.0, 0.0, 0.0), radius: 0.0 },
            collision_bound_index: None,
            render_bound_index: None,
            zone_indices: [0; 2],
            vertex_count: 0,
            node_flags: EBspNodeFlags::None,
            leaf_indices: [None; 2],
            section_index: None,
            section_vertex_index: 0,
            light_map_index: None
        }
    }
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

impl FBox {
    pub fn new_from_points(points: &[Vector3<f32>]) -> FBox {
        let mut min = Vector3::max_value();
        let mut max = Vector3::min_value();
        for point in points {
            if point.x < min.x {
                min.x = point.x;
            }
            if point.y < min.y {
                min.y = point.y;
            }
            if point.z < min.z {
                min.z = point.z;
            }
            if point.x > max.x {
                max.x = point.x;
            }
            if point.y > max.y {
                max.y = point.y;
            }
            if point.z > max.z {
                max.z = point.z;
            }
        }
        FBox { min, max }
    }
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

impl FVert {
    pub fn new() -> FVert {
        FVert {
            vertex_index: 0,
            side: None,
        }
    }
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

// Find closest vertex to a point at or below a node in the Bsp.  If no vertices
// are closer than MinRadius, returns -1.
pub fn find_nearest_vertex(
    model: &UModel,
    source_point: Vector3<f32>,
    dest_point: &mut Vector3<f32>,
    mut min_radius: f32,
    mut node_index: Option<usize>,
    vertex_index: &mut usize
) -> f32 {
    let mut result_radius =  -1.0;
    while node_index.is_some() {
        let node = &model.nodes[node_index.unwrap()];
        let back_index = node.back_node_index;
        let plane_distance = node.plane.plane_dot(source_point);

        if plane_distance >= -min_radius && node.front_node_index.is_some() {
            // Check front.
            let temp_radius = find_nearest_vertex(model, source_point, dest_point, min_radius, node.front_node_index, vertex_index);
            if temp_radius >= 0.0 {
                result_radius = temp_radius;
                min_radius = temp_radius;
            }
        }

        if plane_distance > -min_radius && plane_distance <= min_radius {
			// Check this node's poly's vertices.
            while node_index.is_some() {
                // Loop through all coplanars.
                let node = &model.nodes[node_index.unwrap()];
                let surf = &model.surfs[node.surface_index];
                let base = &model.points[surf.base];
                let temp_radius_squared = (source_point - base).magnitude2();

                if temp_radius_squared < (min_radius * min_radius) {
                    *vertex_index = surf.base;
                    min_radius = temp_radius_squared.sqrt();
                    result_radius = min_radius;
                    *dest_point = *base;
                }

                // more here...it never ends
            }
        }
    }
    // unfinished
    0.0 
}

impl UModel {

    // Find Bsp node vertex nearest to a point (within a certain radius) and
    // set the location.  Returns distance, or -1.f if no point was found.
    pub fn find_vertest_vertex(&self, source_point: Vector3<f32>, dest_point: &mut Vector3<f32>, min_radius: f32, vertex_index: &mut usize) -> f32 {
        return if self.nodes.is_empty() {
            -1.0
        } else {
            find_nearest_vertex(self, source_point, dest_point, min_radius, 0, vertex_index)
        }
    }
    
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
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

pub fn filter_leaf(filter_func: BspFilterFunc, model: &UModel, node_index: usize, ed_poly: &mut FPoly, mut coplanar_info: FCoplanarInfo, mut leaf_outside: bool, place: ENodePlace) {
    
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
                filter_ed_poly(filter_func, model, back_node_index, ed_poly, coplanar_info.clone(), coplanar_info.back_node_outside);
            }
        }
    }
}

// Filter an EdPoly through the Bsp recursively, calling FilterFunc
// for all chunks that fall into leaves.  FCoplanarInfo is used to
// handle the tricky case of double-recursion for polys that must be
// filtered through a node's front, then filtered through the node's back,
// in order to handle coplanar CSG properly.
pub fn filter_ed_poly(filter_func: BspFilterFunc, model: &UModel, mut node_index: usize, ed_poly: &mut FPoly, mut coplanar_info: FCoplanarInfo, mut outside: bool) {
    let mut split_result = 0; // TODO: not sure what this is actually.
    let mut our_front_node_index: Option<usize> = None;
    let mut our_back_node_index: Option<usize> = None;
    let mut new_front_outside = false;
    let mut new_back_outside = false;

    //FilterLoop:
    if ed_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
        // Split EdPoly in half to prevent vertices from overflowing.
        let mut temp = ed_poly.split_in_half();

        // Filter other half.
        filter_ed_poly(filter_func, model, node_index, &mut temp, coplanar_info.clone(), outside);
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
                    filter_leaf(filter_func, model, node_index, ed_poly, coplanar_info.clone(), outside, ENodePlace::Back)
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
            let node = &model.nodes[node_index];
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
                    filter_func, model, node_index, ed_poly, coplanar_info.clone(), coplanar_info.back_node_outside, ENodePlace::Plane
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
                    filter_leaf(filter_func, model, node_index, &mut temp_front_ed_poly, coplanar_info.clone(), new_front_outside, ENodePlace::Front);
                }
                Some(front_node_index) => {
                    filter_ed_poly(filter_func, model, front_node_index, &mut temp_front_ed_poly, coplanar_info.clone(), new_front_outside)
                }
            }

            // Back half of split.
            match node.back_node_index {
                None => {
                    filter_leaf(filter_func, model, node_index, &mut temp_back_ed_poly, coplanar_info.clone(), new_back_outside, ENodePlace::Back);
                }
                Some(back_node_index) => {
                    filter_ed_poly(filter_func, model, back_node_index, &mut temp_back_ed_poly, coplanar_info.clone(), new_back_outside);
                }
            }
        }
    }
}

// Regular entry into FilterEdPoly (so higher-level callers don't have to
// deal with unnecessary info). Filters starting at root.
pub fn bsp_filter_fpoly(filter_func: BspFilterFunc, model: &UModel, ed_poly: &mut FPoly) {
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

/*---------------------------------------------------------------------------------------
   Bsp point/vector refreshing.
---------------------------------------------------------------------------------------*/

pub fn tag_referenced_nodes(model: &mut UModel, node_ref: &mut Vec<Option<usize>>, poly_ref: &mut Vec<Option<usize>>, node_index: usize) {
    let node = &mut model.nodes[node_index];

    node_ref[node_index] = Some(0);
    poly_ref[node.surface_index] = Some(0);

    let front_node_index = node.front_node_index;
    let back_node_index = node.back_node_index;
    let plane_index = node.plane_index;

	if let Some(front_node_index) = front_node_index {
        tag_referenced_nodes(model, node_ref, poly_ref, front_node_index);
    }
	if let Some(back_node_index) = back_node_index {
        tag_referenced_nodes(model, node_ref, poly_ref, back_node_index);
    }
    if let Some(plane_index) = plane_index {
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
            model.surfs[n] = model.surfs[i].clone();
            poly_ref[i] = Some(n);
            n += 1;
        }
    }
    model.surfs.truncate(n);

    n = 0;
    for i in 0..model.nodes.len() {
        if node_ref[i].is_some() {
            model.nodes[n] = model.nodes[i].clone();
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
    for node in &model.nodes {
        let vertex_index = node.vertex_pool_index;
        for b in 0..node.vertex_count {
            let vert = &mut model.verts[vertex_index + b as usize];
            vert.vertex_index = point_ref[vert.vertex_index].unwrap();
        }
    }

    // Shrink the objects.
    model.shrink_model();

}

const WORLD_MAX: f32 = 524288.0;        /* Maximum size of the world */
const HALF_WORLD_MAX: f32 = 262144.0;	/* Half the maximum size of the world */
const HALF_WORLD_MAX1: f32 = 262143.0;  /* Half the maximum size of the world - 1*/

fn build_zone_masks(model: &mut UModel, unknown1: bool) {
    // TODO: fill this in
}

fn bsp_build_bounds(model: &mut UModel) {
    if model.nodes.is_empty() {
        return;
    }

    build_zone_masks(model, false);

    let mut polys: [FPoly; 6];
    
	polys[0].vertices[0] = Vector3::new(-HALF_WORLD_MAX, -HALF_WORLD_MAX,HALF_WORLD_MAX);
	polys[0].vertices[1] = Vector3::new(HALF_WORLD_MAX, -HALF_WORLD_MAX,HALF_WORLD_MAX);
	polys[0].vertices[2] = Vector3::new(HALF_WORLD_MAX, HALF_WORLD_MAX,HALF_WORLD_MAX);
	polys[0].vertices[3] = Vector3::new(-HALF_WORLD_MAX, HALF_WORLD_MAX,HALF_WORLD_MAX);
	polys[0].base = polys[0].vertices[0];
	polys[0].normal = Vector3::new(0.0,  0.0,  1.0);

	polys[1].vertices[0] = Vector3::new(-HALF_WORLD_MAX, HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[1].vertices[1] = Vector3::new(HALF_WORLD_MAX, HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[1].vertices[2] = Vector3::new(HALF_WORLD_MAX,-HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[1].vertices[3] = Vector3::new(-HALF_WORLD_MAX,-HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[1].base = polys[1].vertices[0];
	polys[1].normal = Vector3::new(0.0,  0.0, -1.0);

	polys[2].vertices[0] = Vector3::new(-HALF_WORLD_MAX,HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[2].vertices[1] = Vector3::new(-HALF_WORLD_MAX,HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[2].vertices[2] = Vector3::new(HALF_WORLD_MAX,HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[2].vertices[3] = Vector3::new(HALF_WORLD_MAX,HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[2].base = polys[2].vertices[0];
	polys[2].normal = Vector3::new(0.0, 1.0, 0.0);

	polys[3].vertices[0] = Vector3::new(HALF_WORLD_MAX,-HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[3].vertices[1] = Vector3::new(HALF_WORLD_MAX,-HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[3].vertices[2] = Vector3::new(-HALF_WORLD_MAX,-HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[3].vertices[3] = Vector3::new(-HALF_WORLD_MAX,-HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[3].base = polys[3].vertices[0];
	polys[3].normal = Vector3::new(0.0, -1.0, 0.0);

	polys[4].vertices[0] = Vector3::new(HALF_WORLD_MAX, HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[4].vertices[1] = Vector3::new(HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[4].vertices[2] = Vector3::new(HALF_WORLD_MAX,-HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[4].vertices[3] = Vector3::new(HALF_WORLD_MAX,-HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[4].base = polys[4].vertices[0];
	polys[4].normal = Vector3::new(1.0,  0.0, 0.0);

	polys[5].vertices[0] = Vector3::new(-HALF_WORLD_MAX,-HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[5].vertices[1] = Vector3::new(-HALF_WORLD_MAX,-HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[5].vertices[2] = Vector3::new(-HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
	polys[5].vertices[3] = Vector3::new(-HALF_WORLD_MAX, HALF_WORLD_MAX,-HALF_WORLD_MAX);
	polys[5].normal = Vector3::new(-1.0, 0.0, 0.0);
	polys[5].base = polys[5].vertices[0];

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
        EBspOptimization::Good => cmp::max(1, poly_list.len() / 20),   // Test 20 nodes.
        EBspOptimization::Lame => cmp::max(1, poly_list.len() / 4),    // Test 4 nodes.
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

fn add_thing(vectors: &mut Vec<Vector3<f32>>, v: Vector3<f32>, threshold: f32, check: bool) -> usize {
    if check {
        for (i, vector) in vectors.iter().enumerate() {
            let mut temp = v.x - vector.x;
            if temp > -threshold && temp < threshold {
                temp = v.y - vector.y;
                if temp > -threshold && temp < threshold {
                    temp = v.z - vector.z;
                    if temp > -threshold && temp < threshold {
						// Found nearly-matching vector.
                        return i;
                    }
                }
            }
        }
    }
    vectors.push(v);
    vectors.len() - 1
}

fn bsp_add_point(model: &mut UModel, point: Vector3<f32>, exact: bool) {
    let threshold = if exact { THRESH_POINTS_ARE_SAME } else { THRESH_POINTS_ARE_NEAR };

	// Try to find a match quickly from the Bsp. This finds all potential matches
	// except for any dissociated from nodes/surfaces during a rebuild.
    let mut temp = Vector3::<f32>::new(0.0, 0.0, 0.0);
    let mut nearest_dist = model.find_nearest_vertex(point, temp, threshold, vertex_index);
    let fast_rebuild = false;   // TODO: this is some sort of global

    if nearest_dist >= 0.0 && nearest_dist <= threshold {
        // Found an existing point.
        return vertex_index;
    } else {
        // No match found; add it slowly to find duplicates.
        return add_thing(&mut model.points, point, threshold, !fast_rebuild);
    }
}


// Add an editor polygon to the Bsp, and also stick a reference to it
// in the editor polygon's BspNodes list. If the editor polygon has more sides
// than the Bsp will allow, split it up into several sub-polygons.
//
// Returns: Index to newly-created node of Bsp.  If several nodes were created because
// of split polys, returns the parent (highest one up in the Bsp).
fn bsp_add_node(model: &mut UModel, parent_index: usize, node_place: ENodePlace, mut node_flags: EBspNodeFlags, ed_poly: &FPoly) -> usize {
    if node_place == ENodePlace::Plane {
		// Make sure coplanars are added at the end of the coplanar list so that 
		// we don't insert NF_IsNew nodes with non NF_IsNew coplanar children.
        parent_index = model.nodes[parent_index].plane_index.unwrap();
    }

    let surf = if ed_poly.link.unwrap_or(-1) == model.surfs.len() {
        let mut surf = FBspSurf::new();
        surf.base = bsp_add_point(model, ed_poly.base, true);
        surf.normal_index = bsp_add_vector(model, ed_poly.normal, true);
        surf.texture_u_vector_index = bsp_add_vector(model, ed_poly.texture_u, false);
        surf.texture_v_vector_index = bsp_add_vector(model, ed_poly.texture_v, false);
        surf.material = ed_poly.material;
        surf.poly_flags = ed_poly.poly_flags & !EPolyFlags::NoAddToBSP;
        surf.light_map_scale = ed_poly.light_map_scale;
        surf.actor = ed_poly.actor;
        surf.brush_polygon_index = ed_poly.brush_polygon_index;
        surf.plane = FPlane::new(ed_poly.vertices[0], ed_poly.normal);
        // TODO: i think this needs to be ADDED to a list of some sort
        surf
    } else {
		//check(EdPoly->iLink != INDEX_NONE);
		//check(EdPoly->iLink < Model->Surfs.Num());
        model.surfs[ed_poly.link.unwrap()]
    };

	// Set NodeFlags.
    if surf.poly_flags.contains(EPolyFlags::NotSolid) {
        node_flags |= EBspNodeFlags::NotCsg;
    }
    if surf.poly_flags.intersects(EPolyFlags::Invisible | EPolyFlags::Portal) {
        node_flags |= EBspNodeFlags::NotVisBlocking;
    }
    if surf.poly_flags.contains(EPolyFlags::Masked) {
        node_flags |= EBspNodeFlags::ShootThrough;
    }

    if ed_poly.vertices.len() > MAX_NODE_VERTICES {
		// Split up into two coplanar sub-polygons (one with MAX_NODE_VERTICES vertices and
		// one with all the remaining vertices) and recursively add them.

		// Copy first bunch of verts.
        let ed_poly1 = Box::new(FPoly::new());
        *ed_poly1 = *ed_poly;
        ed_poly1.vertices.truncate(MAX_NODE_VERTICES);

        let ed_poly2 = Box::new(FPoly::new());
        *ed_poly2 = *ed_poly;
		// Copy first vertex then the remaining vertices.   // TODO: INCORRECT
        ed_poly2.vertices.drain(0..MAX_NODE_VERTICES);

        let node_index = bsp_add_node(model, parent_index, node_place, node_flags, &ed_poly1);  // Add this poly first.
        bsp_add_node(model, parent_index, node_place, node_flags, &ed_poly2); // Then add other (may be bigger).

        return node_index;
    } else {
        // Add node.
        if node_place != ENodePlace::Root {
            model.nodes.modify_item(parent_index);  // ??
        }

        let node_index = model.nodes.len();
        let node = FBspNode::new();
        model.nodes.push(node); // TODO: not right, this needs to be Rc'd I think

		// Tell transaction tracking system that parent is about to be modified.
        let parent = if node_place != ENodePlace::Root {
            Some(&mut model.nodes[parent_index])
        } else {
            None
        };

        node.surface_index = ed_poly.link.unwrap();
        node.node_flags = node_flags;
        node.render_bound_index = None;
        node.collision_bound_index = None;
        node.zone_mask = if parent.is_some() { parent.unwrap().zone_mask } else { 0xFFFFFFFFFFFFFFFF };
        node.plane = FPlane::new(ed_poly.vertices[0], ed_poly.normal);
        node.vertex_pool_index = model.verts.len();
        // TODO: add zeroed verts to the model list
        model.verts.resize_with(model.verts.len() + ed_poly.vertices.len(), || FVert::new());
        node.front_node_index = None;
        node.back_node_index = None;
        node.plane_index = None;

        match node_place {
            ENodePlace::Root => {
                node.leaf_indices[0] = None;
                node.leaf_indices[1] = None;
                node.zone_indices[0] = 0;
                node.zone_indices[0] = 0;
            }
            ENodePlace::Front | ENodePlace::Back => {
                let zone_front = if node_place == ENodePlace::Front { 1usize } else { 0usize };
                node.leaf_indices[0] = parent.leaf_indices[zone_front];
                node.leaf_indices[1] = parent.leaf_indices[zone_front];
                node.zone_indices[0] = parent.zone_indices[zone_front];
                node.zone_indices[0] = parent.zone_indices[zone_front];
            }
            _ => {
                let is_flipped = (node.plane.normal.dot(parent.plane.normal) < 0.0) as usize;
                node.leaf_indices[0] = parent.leaf_indices[is_flipped];
                node.leaf_indices[1] = parent.leaf_indices[1 - is_flipped];
                node.zone_indices[0] = parent.zone_indices[is_flipped];
                node.zone_indices[0] = parent.zone_indices[1 - is_flipped];
            }
        }

		// Link parent to this node.
        match node_place {
            ENodePlace::Front => {
                parent.front_node_index = Some(node_index);
            }
            ENodePlace::Back => {
                parent.back_node_index = Some(node_index);
            }
            ENodePlace::Plane => {
                parent.plane_index = Some(node_index);
            }
            _ => {}
        }

		// Add all points to point table, merging nearly-overlapping polygon points
		// with other points in the poly to prevent criscrossing vertices from
		// being generated.

		// Must maintain Node->NumVertices on the fly so that bspAddPoint is always
		// called with the Bsp in a clean state.

        let n = ed_poly.vertices.len();
        let points: ArrayVec<Vector3<f32>, MAX_NODE_VERTICES>;
        let vert_pool = &model.verts[node.vertex_pool_index..];
        for i in 0..n {
            let vertex_index = bsp_add_point(model, ed_poly.vertices[i], false);

            if node.vertex_count == 0 || vert_pool[node.vertex_count as usize - 1].vertex_index != vertex_index {
                points[node.vertex_count as usize] = ed_poly.vertices[i];
                vert_pool[node.vertex_count as usize].side = None;
                vert_pool[node.vertex_count as usize].vertex_index = vertex_index;
                node.vertex_count += 1;
            }
        }

        if node.vertex_count >= 2 && vert_pool[0].vertex_index == vert_pool[node.vertex_count as usize - 1].vertex_index {
            node.vertex_count -= 1;
        }

        if node.vertex_count < 3 {
            //g_errors += 1;
            println!("bspAddNBode: Infinitesimal polygon {} ({}", node.vertex_count, n);
            node.vertex_count = 0;
        }

        node.section_index = None;
        node.light_map_index = None;

		// Calculate a bounding sphere for this node.
        node.exclusive_sphere_bound = FSphere::new_from_points(&points);

        return node_index;
    }
}


// Pick a splitter poly then split a pool of polygons into front and back polygons and
// recurse.
//
// iParent = Parent Bsp node, or INDEX_NONE if this is the root node.
// IsFront = 1 if this is the front node of iParent, 0 of back (undefined if iParent==INDEX_NONE)
pub fn split_poly_list(
    model: &mut UModel, 
    parent_node_index: Option<usize>, 
    node_place: ENodePlace, 
    poly_count: usize, 
    poly_list: &Vec<&FPoly>, 
    optimization: EBspOptimization, 
    balance: usize, 
    portal_bias: f32, 
    rebuild_simple_polys: bool) {

    let num_polys_to_alloc = poly_count + 8 + poly_count / 4;
    let front_list: Vec<&FPoly> = Vec::with_capacity(num_polys_to_alloc);
    let back_list: Vec<&FPoly> = Vec::with_capacity(num_polys_to_alloc);

    let split_poly: &mut FPoly = find_best_split(poly_list.as_slice(), optimization, balance, portal_bias);

    // Add the splitter poly to the Bsp with either a new BspSurf or an existing one.
    if rebuild_simple_polys {
        split_poly.link = Some(model.surfs.len());
    }

    let our_node = bsp_add_node(model, parent_node_index, node_place, 0, split_poly);
    let plane_node = our_node;

	// Now divide all polygons in the pool into (A) polygons that are
	// in front of Poly, and (B) polygons that are in back of Poly.
	// Coplanar polys are inserted immediately, before recursing.

	// If any polygons are split by Poly, we ignrore the original poly,
	// split it into two polys, and add two new polys to the pool.

    let mut front_ed_poly = Box::new(FPoly::new());
    let mut back_ed_poly = Box::new(FPoly::new());

    for i in 0..poly_count {
        let ed_poly = poly_list[i];

        if ed_poly == split_poly {  // TODO: this is testing pointer equality, not value equality
            continue;
        }

        match ed_poly.split_with_plane(
            split_poly.vertices[0], 
            split_poly.normal, 
            Some(&mut front_ed_poly), 
            Some(&mut back_ed_poly), 
            false) 
            {
                ESplitType::Coplanar => {
                    if rebuild_simple_polys {
                        ed_poly.link = if model.surfs.is_empty() { None } else { Some(model.surfs.len() - 1) }
                    }
                    plane_node = bsp_add_node(model, plane_node, ENodePlace::Plane, 0, ed_poly);
                },
                ESplitType::Front => {
                    front_list.push(poly_list[i]);
                },
                ESplitType::Back => {
                    back_list.push(poly_list[i]);
                },
                ESplitType::Split => {
                    // REALIZATION: All the polys need to be heap allocated.
                    // Create front & back nodes.
                    front_list.push(&front_ed_poly);
                    back_list.push(&back_ed_poly);

				    // If newly-split polygons have too many vertices, break them up in half.
                    if front_ed_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
                        let mut temp = Box::new(front_ed_poly.split_in_half());
                        front_list.push(&temp);
                    }

                    if back_ed_poly.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
                        let mut temp = Box::new(back_ed_poly.split_in_half());
                        back_list.push(&temp);
                    }

                    front_ed_poly = Box::new(FPoly::new());
                    back_ed_poly = Box::new(FPoly::new());
                }
            }
    }

    if !front_list.is_empty() {
        split_poly_list(model, our_node, ENodePlace::Front, front_list.len(), &front_list, optimization, balance, portal_bias, rebuild_simple_polys);
    }
    if !back_list.is_empty() {
        split_poly_list(model, our_node, ENodePlace::Back, back_list.len(), &back_list, optimization, balance, portal_bias, rebuild_simple_polys);
    }
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
        let poly_count = model.polys.len();
        split_poly_list(model, None, ENodePlace::Root, poly_count, &poly_list,  optimization,  balance,  portal_bias,  rebuild_simple_polys);

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


struct BspBuilder {
    temp_model: UModel,
    g_errors: usize
}

impl BspBuilder {

    pub fn bsp_brush_csg(&mut self, actor: Rc<ABrush>, model: &mut UModel, poly_flags: EPolyFlags, csg_operation: ECsgOper, build_bounds: bool, merge_polygons: bool) -> usize {
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
            let poly = &brush.polys[i];

            if poly.material.is_none() {
                // ======================================================================
                // ======================================================================
                // ======================================================================
                // ======================================================================
                // ======================================================================
                //poly.material = current_material;   // TODO: where is this coming from?
            }

            let mut dest_ed_poly = poly.clone();

            // Set its backwards brush link.
            dest_ed_poly.actor = Some(actor.clone());
            dest_ed_poly.brush_polygon_index = Some(i);

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
            let mut ed_poly = self.temp_model.polys[i].clone();

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
                    ed_poly.link = self.temp_model.polys[ed_poly_link].link;
                }

                let bsp_filter_function = match csg_operation {
                    ECsgOper::Add => add_brush_to_world_func,
                    ECsgOper::Subtract => sutract_brush_from_world_func
                };
                
                bsp_filter_fpoly(bsp_filter_function, model, &mut ed_poly);
            }
        }

        if !model.nodes.is_empty() && !poly_flags.intersects(EPolyFlags::NotSolid | EPolyFlags::SemiSolid) {
            // Quickly build a Bsp for the brush, tending to minimize splits rather than balance
            // the tree.  We only need the cutting planes, though the entire Bsp struct (polys and
            // all) is built.
            bsp_build(&mut self.temp_model, EBspOptimization::Lame, 0, 0.7, true, 0);

            // ======================================================================
            // ======================================================================
            // ======================================================================
            // ======================================================================
            // ======================================================================
            //gmodel = brush;

            // ======================================================================
            // ======================================================================
            // ======================================================================
            // ======================================================================
            // ======================================================================
            //self.temp_model.build_bound();  // TODO: this is called twice for some reason in the original code.

            // ======================================================================
            // ======================================================================
            // ======================================================================
            // ======================================================================
            //filter_world_through_brush(model, &self.temp_model, csg_operation, 0, self.temp_model.bounding_shere);
        }

        // ======================================================================
        // ======================================================================
        // ======================================================================
        // ======================================================================
        // ======================================================================
		// Clean up nodes, reset node flags.
        //bsp_cleanup(model);

		// Rebuild bounding volumes.
        if build_bounds {
            bsp_build_bounds(model);
        }

	    // Release TempModel.
        self.temp_model.empty_model(true, true);

        return 1 + self.g_errors;
    }
}


