use cgmath::InnerSpace;

use crate::math::points_are_same;
use crate::fpoly::{FPoly, RemoveColinearsResult, FPOLY_MAX_VERTICES};
use crate::model::{UModel, BSP_NODE_MAX_NODE_VERTICES};

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
pub fn merge_coplanars(model: &mut UModel, poly_indices: &[usize]) -> usize {
    let mut merged_count: usize = 0;
    let mut merge_again = true;
    while merge_again {
        merge_again = false;
        for i in 0..poly_indices.len() {
            let poly1 = &model.polys[poly_indices[i]];
            if poly1.vertices.len() == 0 {
                // Polygon has already been merged away.
                continue
            }
            for j in i + 1..poly_indices.len() {
                if let Ok([poly1, poly2]) = model.polys.get_many_mut([poly_indices[i], poly_indices[j]]) {
                    if poly2.vertices.len() == 0 {
                        continue
                    }
                    if try_to_merge(poly1, poly2) {
                        println!("Merged polygons {} and {}", poly_indices[i], poly_indices[j]);
                        merged_count += 1;
                        merge_again = true;
                    }
                }
            }
        }
    }
    merged_count
}

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
