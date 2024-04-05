use cgmath::{Vector3, InnerSpace};
use arrayvec::ArrayVec;
use bitflags::bitflags;
use math::THRESH_SPLIT_POLY_PRECISELY;
use crate::coords::FModelCoords;
use crate::fpoly::ESplitPlaneStatus::Front;
use crate::math::{self, transform_vector_by_coords, FLOAT_NORMAL_THRESH, SMALL_NUMBER, THRESH_POINT_ON_PLANE};
use crate::math::{point_plane_distance, line_plane_intersection, THRESH_SPLIT_POLY_WITH_PLANE, points_are_near, THRESH_ZERO_NORM_SQUARED};
use crate::model::UModel;
use crate::brush::ABrush;
use std::rc::{Rc, Weak};
use std::cell::RefCell;

/// Maximum vertices an FPoly may have.
pub const FPOLY_MAX_VERTICES: usize = 16;
/// Threshold for splitting into two.
const FPOLY_VERTEX_THRESHOLD: usize = FPOLY_MAX_VERTICES - 2;

type FVector = Vector3<f32>;

/// Flags describing effects and properties of a Bsp polygon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EPolyFlags(u32);

bitflags! {
    impl EPolyFlags : u32 {
        // Regular in-game flags.
        /// Poly is invisible.
        const Invisible     = 0x00000001;
        /// Poly should be drawn masked.
	    const Masked		= 0x00000002;
        /// Poly is transparent.
	    const Translucent	= 0x00000004;
        /// Poly is not solid, doesn't block.
	    const NotSolid		= 0x00000008;
        /// Poly should be drawn environment mapped.
	    const Environment   = 0x00000010;
        /// Poly is semi-solid = collision solid, Csg nonsolid.
	    const Semisolid	  	= 0x00000020;
        /// Modulation transparency.
	    const Modulated 	= 0x00000040;
        /// Poly looks exactly like backdrop.
	    const FakeBackdrop	= 0x00000080;
        /// Poly is visible from both sides.
	    const TwoSided		= 0x00000100;
        /// Don't smooth textures.
	    const NoSmooth		= 0x00000800;
        /// Honor texture alpha (reuse BigWavy and SpecialPoly flags)
	    const AlphaTexture	= 0x00001000;
        /// Flat surface.
	    const Flat			= 0x00004000;
        /// Don't merge poly's nodes before lighting when rendering.
	    const NoMerge		= 0x00010000;
        /// Don't test Z buffer
	    const NoZTest		= 0x00020000;
        /// sjs - additive blending, (Aliases DirtyShadows).
	    const Additive		= 0x00040000;
        /// Only speciallit lights apply to this poly.
	    const SpecialLit	= 0x00100000;
        /// Render as wireframe
	    const Wireframe		= 0x00200000;
        /// Unlit.
	    const Unlit			= 0x00400000;
        /// Portal between iZones.
	    const Portal		= 0x04000000;
        /// Antiportal
	    const AntiPortal	= 0x08000000;
        /// Mirrored BSP surface.
	    const Mirrored      = 0x20000000;
        
        // Editor flags.
        /// Editor: Poly is remembered.
        const Memorized     = 0x01000000;
        /// Editor: Poly is selected.
        const Selected      = 0x02000000;
        /// sjs - subtractive blending
        const Subtractive	= 0x20000000;
        /// FPoly has been split by SplitPolyWithPlane.
        const FlatShaded	= 0x40000000;


	    // Internal.
        /// FPoly was already processed in editorBuildFPolys.
	    const EdProcessed 		= 0x40000000;
        /// FPoly has been split by SplitPolyWithPlane.
	    const EdCut       		= 0x80000000;
        /// Occludes even if PF_NoOcclude.
	    const Occlude			= 0x80000000;
    }
}

/// Results from FPoly.SplitWithPlane, describing the result of splitting
/// an arbitrary FPoly with an arbitrary plane.
#[derive(Debug, PartialEq)]
pub enum ESplitType
{
    /// Poly wasn't split, but is coplanar with plane
    Coplanar,
    /// Poly wasn't split, but is entirely in front of plane
    Front,
    /// Poly wasn't split, but is entirely in back of plane
    Back,
    /// Poly was split into two new editor polygons
    Split(FPoly, FPoly),
}

#[derive(Clone, Debug, PartialEq)]
pub enum RemoveColinearsResult {
    Convex,
    Concave,
    Collapsed
}

/// A general-purpose polygon used by the editor.  An FPoly is a free-standing
/// class which exists independently of any particular level, unlike the polys
/// associated with Bsp nodes which rely on scads of other objects.  FPolys are
/// used in UnrealEd for internal work, such as building the Bsp and performing
/// boolean operations.
#[derive(Clone, Debug, PartialEq)]
pub struct FPoly {
    /// Base point of polygon.
    pub base: FVector,
    /// Normal of polygon.
    pub normal: FVector,
    /// Texture U vector.
    pub texture_u: FVector,
    /// Texture V vector.
    pub texture_v: FVector,
    /// Actual vertices (formerly `vertex`)
    pub vertices: ArrayVec<FVector, FPOLY_MAX_VERTICES>,
    /// FPoly & Bsp poly bit flags (PF_).
    pub poly_flags: EPolyFlags,
    /// Brush where this originated, or NULL.
    pub actor: Option<Rc<RefCell<ABrush>>>,
    /// Material.
    //material: Rc<UMaterial>,
    /// Item name.
    //item_name: FName,
    /// iBspSurf, or brush fpoly index of first identical polygon, or MAXWORD.
    pub link: Option<usize>,
    /// Index of editor solid's polygon this originated from.
    pub brush_poly_index: Option<usize>,
    /// Used by multiple vertex editing to keep track of original PolyIndex into owner brush
    pub save_poly_index: Option<i32>,
    /// A temporary place to save the poly normal during vertex editing
    pub save_normal: FVector,
    /// UV coordinates corresponding to the first 3 vertices in the list.  Used by texture alignment tools. (X = U, Y = V, Z = N/A)
    pub uv: [FVector; 3],
    /// A mask used to determine which smoothing groups this polygon is in.  SmoothingMask & (1 << GroupNumber)
    pub smoothing_mask: u32,
    /// The number of units/lightmap texel on this surface.
    pub light_map_scale: f32
}

#[derive(Clone, Copy, PartialEq)]
enum ESplitPlaneStatus {
    Front,
    Back,
    Either
}

impl FPoly {

    pub fn from_vertices(vertices: &[FVector]) -> Self {
        let mut fpoly = FPoly::new();
        _ = fpoly.vertices.try_extend_from_slice(vertices);
        _ = fpoly.calc_normal();
        fpoly
    }

    /// Initialize everything in an editor polygon structure to defaults.
    pub fn new() -> FPoly {
        FPoly {
            base: FVector { x: 0.0, y: 0.0, z: 0.0 },
            normal: FVector { x: 0.0, y: 0.0, z: 0.0 },
            texture_u: FVector { x: 0.0, y: 0.0, z: 0.0 },
            texture_v: FVector { x: 0.0, y: 0.0, z: 0.0 },
            vertices: Default::default(),
            poly_flags: EPolyFlags::from_bits_retain(0),
            actor: None,
            link: None,
            brush_poly_index: None,
            save_poly_index: None,
            save_normal: FVector { x: 0.0, y: 0.0, z: 0.0 },
            uv: [
                FVector { x: 0.0, y: 0.0, z: 0.0 },
                FVector { x: 0.0, y: 0.0, z: 0.0 },
                FVector { x: 0.0, y: 0.0, z: 0.0 }
            ],
            smoothing_mask: 0,
            light_map_scale: 32.0
        }
    }

    /// Reverse an FPoly by reversing the normal and reversing the order of its vertices.
    pub fn reverse(&mut self) {
        self.normal *= -1.0;
        self.vertices.reverse()
    }

    /// Fix up an editor poly by deleting vertices that are identical. Sets
    /// vertex count to zero if it collapses.  Returns number of vertices, 0 or >=3.
    pub fn fix(&mut self) -> usize {
        use math::points_are_same;

        let mut prev = self.vertices.len() - 1;
        let mut j = 0usize;
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
            self.vertices.clear();
        }

        self.vertices.len()
    }

    /// Compute the 2D area.
    pub fn area2(&self) -> f32 {
        let mut area = 0.0;
        let mut side1 = self.vertices[1] - self.vertices[0];
        for i in 2..self.vertices.len() {
            let side2 = self.vertices[i] - self.vertices[0];
            area += side1.cross(side2).magnitude();
            side1 = side2;
        }
        area
    }

    /// Compute the 2D area.
    #[inline]
    pub fn area(&self) -> f32 {
        return self.area2() / 2.0;
    }

    /// Split with plane. Meant to be numerically stable.
    pub fn split_with_plane(&self, plane_base: FVector, plane_normal: FVector, very_precise: bool) -> ESplitType {
        let threshold = if very_precise {
            THRESH_SPLIT_POLY_PRECISELY
        } else {
            THRESH_SPLIT_POLY_WITH_PLANE
        };

        // See if the polygon is split by SplitPoly, or it's on either side, or the
        // polys are coplanar.  Go through all of the polygon points and
        // calculate the minimum and maximum signed distance (in the direction
        // of the normal) from each point to the plane of SplitPoly.
        let mut status_previous = ESplitPlaneStatus::Either;
        let mut distance_max = f32::MIN;
        let mut distance_min = f32::MAX;

        // See if the polygon is split by SplitPoly, or it's on either side, or the
        // polys are coplanar.  Go through all of the polygon points and
        // calculate the minimum and maximum signed distance (in the direction
        // of the normal) from each point to the plane of SplitPoly.
        for vertex in &self.vertices {
            let distance = point_plane_distance(vertex, &plane_base, &plane_normal);
            distance_max = distance.max(distance_max);
            distance_min = distance.min(distance_min);
            if distance > threshold {
                status_previous = ESplitPlaneStatus::Front
            } else if distance < -threshold {
                status_previous = ESplitPlaneStatus::Back
            }
        }

        if distance_max < threshold && distance_min > -threshold {
            ESplitType::Coplanar
        } else if distance_max < threshold {
            ESplitType::Back
        } else if distance_min > -threshold {
            ESplitType::Front
        } else {
            // TODO: copy this poly twice
            let mut front_poly = self.clone();
            front_poly.poly_flags.set(EPolyFlags::EdCut, true); // Mark as cut.
            front_poly.vertices.clear();

            let mut back_poly = self.clone();
            back_poly.poly_flags.set(EPolyFlags::EdCut, true);  // Mark as cut.
            back_poly.vertices.clear();

            let mut j = self.vertices.len() - 1; // Previous vertex; have PrevStatus already.
            let mut distance_previous = 0f32;

            for i in 0..self.vertices.len() {
                let distance = point_plane_distance(&self.vertices[i], &plane_base, &plane_normal);

                let status = if distance > threshold {
                    ESplitPlaneStatus::Front
                } else if distance < -threshold {
                    ESplitPlaneStatus::Back
                } else {
                    status_previous
                };

                if status != status_previous {
                    // Crossing.  Either Front-to-Back or Back-To-Front.
                    // Intersection point is naturally on both front and back polys.
                    if distance >= -threshold && distance < threshold {
                        // This point lies on plane.
                        if status_previous == Front {
                            front_poly.vertices.push(self.vertices[i]);
                            back_poly.vertices.push(self.vertices[i]);
                        } else {
                            back_poly.vertices.push(self.vertices[i]);
                            front_poly.vertices.push(self.vertices[i]);
                        }
                    } else if distance_previous >= -threshold && distance_previous < threshold {
                        // Previous point lies on plane.
                        if status == Front {
                            front_poly.vertices.push(self.vertices[j]);
                            front_poly.vertices.push(self.vertices[i]);
                        } else {
                            back_poly.vertices.push(self.vertices[j]);
                            back_poly.vertices.push(self.vertices[i]);
                        }
                    } else {
                        // Intersection point is in between.
                        let intersection = line_plane_intersection(&self.vertices[j], &self.vertices[i], &plane_base, &plane_normal);

                        if status_previous == Front {
                            front_poly.vertices.push(intersection.clone());
                            back_poly.vertices.push(intersection.clone());
                            back_poly.vertices.push(self.vertices[i]);
                        } else {
                            back_poly.vertices.push(intersection.clone());
                            front_poly.vertices.push(intersection.clone());
                            front_poly.vertices.push(self.vertices[i]);
                        }
                    }
                } else {
                    if status == Front {
                        front_poly.vertices.push(self.vertices[i]);
                    } else {
                        back_poly.vertices.push(self.vertices[i]);
                    }
                }

                j = i;
                status_previous = status;
                distance_previous = distance;
            }

            // Handle possibility of sliver polys due to precision errors.
            if front_poly.fix() < 3 {
                return ESplitType::Back
            } else if back_poly.fix() < 3 {
                return ESplitType::Front
            }

            return ESplitType::Split(front_poly, back_poly)
        }
    }

    /// Split with a Bsp node.
    pub fn split_with_node(&self, model: &UModel, node_index: usize, very_precise: bool) -> ESplitType {
        let node = &model.nodes[node_index];
        let surface = &model.surfaces[node.surface_index];
        let plane_base = model.points[model.vertices[node.vertex_pool_index].vertex_index];
        let plane_normal = model.vectors[surface.normal_index];

        self.split_with_plane(plane_base, plane_normal, very_precise)
    }

    /// Split with plane quickly for in-game geometry operations.
    /// Results are always valid. May return sliver polys.
    pub fn split_with_plane_fast(&self, plane_base: &FVector, plane_normal: &FVector) -> ESplitType {
        let mut vertex_statuses = [ESplitPlaneStatus::Front; FPOLY_MAX_VERTICES];
        let mut front = false;
        let mut back = false;

        for i in 0..self.vertices.len() {
            let distance = point_plane_distance(&self.vertices[i], plane_base, plane_normal);
            if distance >= 0.0 {
                vertex_statuses[i] = ESplitPlaneStatus::Front;
                if distance > THRESH_SPLIT_POLY_WITH_PLANE {
                    front = true;
                }
            } else {
                vertex_statuses[i] = ESplitPlaneStatus::Back;
                if distance < -THRESH_SPLIT_POLY_WITH_PLANE {
                    back = true;
                }
            }
        }

        if !front {
            if back {
                ESplitType::Back
            } else {
                ESplitType::Coplanar
            }
        }
        else if !back {
            ESplitType::Front
        }
        else {
            let mut front_poly: FPoly = FPoly::new();
            let mut back_poly = FPoly::new();

            let mut v = 0usize;
            let mut w = self.vertices.len() - 1;
            let mut prev_status = vertex_statuses[w];

            for i in 0..self.vertices.len() {
                let status = vertex_statuses[i];
                if status != prev_status {
                    // Crossing.
                    let intersection = line_plane_intersection(&self.vertices[w], &self.vertices[v], &plane_base, &plane_normal);
                    front_poly.vertices.push(intersection);
                    back_poly.vertices.push(intersection);
                    if prev_status == ESplitPlaneStatus::Front {
                        back_poly.vertices.push(self.vertices[v]);
                    } else {
                        front_poly.vertices.push(self.vertices[v]);
                    }
                } else if status == ESplitPlaneStatus::Front {
                    front_poly.vertices.push(self.vertices[v]);
                } else {
                    back_poly.vertices.push(self.vertices[v]);
                }

                prev_status = status;
                w = v;
                v += 1;
            }

            front_poly.base = self.base;
            front_poly.normal = self.normal;
            front_poly.poly_flags = self.poly_flags;

            back_poly.base = self.base;
            back_poly.normal = self.normal;
            back_poly.poly_flags = self.poly_flags;

            ESplitType::Split(front_poly, back_poly)
        }
    }

    /// Split an FPoly in half.
    pub fn split_in_half(&mut self) -> Option<FPoly> {
        if self.vertices.len() <= 3 || self.vertices.len() > FPOLY_MAX_VERTICES {
            return None;
        }

        let m = self.vertices.len() / 2;
        let mut other_half = self.clone();

        self.vertices.truncate(m + 1);
        other_half.vertices.drain(0..m);
        other_half.vertices.push(self.vertices[0]);

        self.poly_flags |= EPolyFlags::EdCut;
        other_half.poly_flags |= EPolyFlags::EdCut;

        Some(other_half)
    }

    /// Compute normal of an FPoly.  Works even if FPoly has 180-degree-angled sides (which
    /// are often created during T-joint elimination).  Returns nonzero result (plus sets
    /// normal vector to zero) if a problem occurs.
    pub fn calc_normal(&mut self) -> Result<FVector, String> {
        self.normal = FVector::new(0.0, 0.0, 0.0);
        for i in 2..self.vertices.len() {
            self.normal += (self.vertices[i - 1] - self.vertices[0]).cross(self.vertices[i] - self.vertices[0]);
        }
        if self.normal.magnitude2() < THRESH_ZERO_NORM_SQUARED {
            return Err("Zero-area normal".to_string());
        }
        self.normal = self.normal.normalize();
        return Ok(self.normal)
    }

    /// Remove colinear vertices and check convexity.
    pub fn remove_colinears(&mut self) -> RemoveColinearsResult {
        let mut side_plane_normals: ArrayVec<FVector, FPOLY_MAX_VERTICES> = ArrayVec::new();
        let mut i = 0;

        // Add as many side plane normals as there are vertices.
        for _ in 0..self.vertices.len() {
            side_plane_normals.push(FVector::new(0.0, 0.0, 0.0));
        }

        while i < self.vertices.len() {
            let j = if i == 0 { self.vertices.len() - 1 } else { i - 1 };

            // Create cutting plane perpendicular to both this side and the polygon's normal.
            let side = self.vertices[i] - self.vertices[j];
            let side_plane_normal = side.cross(self.normal);

            if side_plane_normal.dot(side_plane_normal) < SMALL_NUMBER {
                // Eliminate these nearly identical points.
                self.vertices.remove(i);
                side_plane_normals.truncate(self.vertices.len());
                if self.vertices.len() < 3 {
                    // Collapsed.
                    self.vertices.clear();
                    return RemoveColinearsResult::Collapsed;
                }
                if i > 0 {
                    i -= 1;
                    continue;
                }
            } else {
                side_plane_normals[i] = side_plane_normal.normalize();
            }

            i += 1;
        }

        i = 0;

        while i < self.vertices.len() {
            let j = (i + 1) % self.vertices.len();
            if points_are_near(&side_plane_normals[i], &side_plane_normals[j], FLOAT_NORMAL_THRESH) {
                // Eliminate colinear points.
                self.vertices.remove(i);
                side_plane_normals.remove(i);
                if self.vertices.len() < 3 {
                    // Collapsed.
                    self.vertices.clear();
                    return RemoveColinearsResult::Collapsed;
                }
                if i > 0 {
                    i -= 1;
                    continue;
                }
            } else {
                match self.split_with_plane(self.vertices[i], side_plane_normals[i], false) {
                    ESplitType::Front | ESplitType::Split(_, _) => {
                        return RemoveColinearsResult::Concave;
                    }
                    _ => {}
                }
            }

            i += 1;
        }

        RemoveColinearsResult::Convex
    }

    /// Checks to see if the specified vertex is on this poly.  Assumes the vertex is on the same
    /// plane as the poly and that the poly is convex.
    ///
    /// This can be combined with FLinePlaneIntersection to perform a line-fpoly intersection test.
    pub fn on_poly(&self, point: &FVector) -> bool {
        for i in 0..self.vertices.len() {
            let j = if i == 0 { self.vertices.len() - 1 } else { i - 1 };
            let side = self.vertices[i] - self.vertices[j];
            let side_plane_normal = side.cross(self.normal).normalize();

            if point_plane_distance(point, &self.vertices[i], &side_plane_normal) > THRESH_POINT_ON_PLANE {
                return false;
            }
        }

        true
    }

    /// Checks to see if the specified line intersects this poly or not and returns the intersection point and whether or not the intersection point is on the poly.
    /// Note that this function is not used in the CSG or BSP code, so we don't care to test it.
    pub fn does_line_intersect(&self, start: &FVector, end: &FVector, intersection: Option<&mut FVector>) -> bool {
        // If the ray doesn't cross the plane, don't bother going any further.
        let dist_start = point_plane_distance(start, &self.vertices[0], &self.normal);
        let dist_end = point_plane_distance(end, &self.vertices[0], &self.normal);

        if dist_start < 0.0 && dist_end < 0.0 || dist_start > 0.0 && dist_end > 0.0 {
            return false;
        }

        // Get the intersection of the line and the plane.
        let _intersection = line_plane_intersection(start, end, &self.vertices[0], &self.normal);

        match intersection {
            Some(f) => *f = _intersection,
            None => {}
        }

        if _intersection == *start || _intersection == *end {
            return false;
        }

        // Check if the intersection point is actually on the poly.
        return self.on_poly(&_intersection)
    }

    // Inserts a vertex into the poly at a specific position.
    pub fn insert_vertex(&mut self, index: usize, vertex: &FVector) {
        assert!(index <= self.vertices.len());
        self.vertices.insert(index, *vertex);
    }

    // Split a poly and keep only the front half. Returns number of vertices,
    // 0 if clipped away.
    pub fn split(&mut self, plane_normal: &FVector, plane_base: &FVector, no_overflow: bool) -> usize {
        if no_overflow && self.vertices.len() >= FPOLY_VERTEX_THRESHOLD {
            // Don't split it, just reject it.
            match self.split_with_plane_fast(plane_base, plane_normal) {
                ESplitType::Back => 0,
                _ => self.vertices.len()
            }
        } else {
            // Split it.
            match self.split_with_plane_fast(plane_base, plane_normal) {
                ESplitType::Back => 0,
                ESplitType::Split(front, _) => {
                    *self = front;
                    self.vertices.len()
                },
                _ => self.vertices.len()
            }
        }
    }

    pub fn finalize(&mut self) -> Result<(), String> {
        // Check for problems.
        self.fix();

        if self.vertices.len() < 3 {
            return Err(format!("Not enough vertices ({})", self.vertices.len()).to_string());
        }

        if self.normal == FVector::new(0.0, 0.0, 0.0) && self.vertices.len() >= 3 {
            if self.calc_normal().is_err() {
                return Err(format!("Normalization failed, verts={}, size={}", self.vertices.len(), self.normal.magnitude()).to_string());
            }
        }

        // If texture U and V coordinates weren't specified, generate them.
        if self.texture_u == FVector::new(0.0, 0.0, 0.0) && self.texture_v == FVector::new(0.0, 0.0, 0.0) {
            for i in 1..self.vertices.len() {
                self.texture_u = (self.vertices[0] - self.vertices[i]).cross(self.normal).normalize();
                self.texture_v = self.normal.cross(self.texture_u).normalize();
                if self.texture_u.magnitude2() != 0.0 && self.texture_v.magnitude2() != 0.0 {
                    break;
                }
            }
        }

        Ok(())
    }

    pub fn is_backfaced(&self, point: &FVector) -> bool {
        (point - self.base).dot(self.normal) < 0.0
    }

    pub fn is_coplanar(&self, other: &FPoly) -> bool {
        (self.base - other.base).dot(self.normal).abs() < 0.01 && self.normal.dot(other.normal).abs() > 0.9999
    }

    /// Return whether this poly and Test are facing each other.
    /// The polys are facing if they are noncoplanar, one or more of Test's points is in 
    /// front of this poly, and one or more of this poly's points are behind Test.
    pub fn faces(&self, other: &FPoly) -> bool {
        // Coplanar implies not facing.
        if self.is_coplanar(other) {
            return false;
        }
        // If this poly is frontfaced relative to all of ther other poly's vertices, they're not facing.
        for other_vertex in &other.vertices {
            if !self.is_backfaced(other_vertex) {
                // If test is frontfaced relative to one or more of this poly's points, they're facing.
                for vertex in &self.vertices {
                    if other.is_backfaced(vertex) {
                        return true;
                    }
                }
                return false;
            }
        }
        false
    }

    /// Transform an editor polygon with a coordinate system, a pre-transformation
    /// addition, and a post-transformation addition:
    pub fn transform(&mut self, coords: &FModelCoords, pre_subtract: &FVector, post_add: &FVector, orientation: f32) -> &mut Self {
        self.texture_u = transform_vector_by_coords(&self.texture_u, &coords.contravariant);
        self.texture_v = transform_vector_by_coords(&self.texture_v, &coords.contravariant);
        self.base = transform_vector_by_coords(&(self.base - pre_subtract), &coords.covariant) + post_add;
        self.vertices.iter_mut().for_each(|vertex| {
            *vertex = transform_vector_by_coords(&(*vertex - pre_subtract), &coords.covariant) + post_add;
        });
	    // Flip vertex order if orientation is negative.
        if orientation < 0.0 {
            self.vertices.reverse();
        }
        // Transform normal.  Since the transformation coordinate system is
        // orthogonal but not orthonormal, it has to be renormalized here.
        self.normal = transform_vector_by_coords(&self.normal, &coords.contravariant).normalize();
        self
    }

}
