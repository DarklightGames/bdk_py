use bdk_rs::{self, coords::FModelCoords, fpoly};

use cgmath::{InnerSpace, Vector3};
use fpoly::{ESplitType, FPoly, RemoveColinearsResult};
use bdk_rs::math::FVector;
use fpoly::EPolyFlags;

#[test]
fn fpoly_reverse_test() {
    let mut fpoly = FPoly::new();
    fpoly.normal = Vector3::unit_x();
    fpoly.vertices.push(Vector3 { x: 0.0, y: 1.0, z: 2.0});
    fpoly.vertices.push(Vector3 { x: 3.0, y: 4.0, z: 5.0});
    fpoly.vertices.push(Vector3 { x: 6.0, y: 7.0, z: 8.0});

    fpoly.reverse();

    assert_eq!(fpoly.normal, Vector3 { x: -1.0, y: 0.0, z: 0.0 });
    assert_eq!(fpoly.vertices.len(), 3);
    assert_eq!(fpoly.vertices[0], Vector3 { x: 6.0, y: 7.0, z: 8.0});
    assert_eq!(fpoly.vertices[1], Vector3 { x: 3.0, y: 4.0, z: 5.0});
    assert_eq!(fpoly.vertices[2], Vector3 { x: 0.0, y: 1.0, z: 2.0});
}

#[test]
fn fpoly_fix_no_collapse_test() {
    let mut fpoly = FPoly::new();

    fpoly.vertices.push(Vector3 { x: 0.0, y: 1.0, z: 2.0});
    fpoly.vertices.push(Vector3 { x: 3.0, y: 4.0, z: 5.0});
    fpoly.vertices.push(Vector3 { x: 6.0, y: 7.0, z: 8.0});
    fpoly.vertices.push(Vector3 { x: 6.0, y: 7.0, z: 8.0}); // Duplicate.
    fpoly.vertices.push(Vector3 { x: 6.0, y: 7.0, z: 8.0}); // Duplicate.
    fpoly.vertices.push(Vector3 { x: 9.0, y: 10.0, z: 11.0});
    fpoly.vertices.push(Vector3 { x: 0.0, y: 1.0, z: 2.0}); // Duplicate.

    let return_value = fpoly.fix();

    assert_eq!(return_value, 4);
    assert_eq!(fpoly.vertices.len(), 4);
    assert_eq!(fpoly.vertices[0], Vector3 { x: 3.0, y: 4.0, z: 5.0});
    assert_eq!(fpoly.vertices[1], Vector3 { x: 6.0, y: 7.0, z: 8.0});
    assert_eq!(fpoly.vertices[2], Vector3 { x: 9.0, y: 10.0, z: 11.0});
    assert_eq!(fpoly.vertices[3], Vector3 { x: 0.0, y: 1.0, z: 2.0});
}

#[test]
fn fpoly_fix_collapse_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector { x: 0.0, y: 1.0, z: 2.0 },
        FVector { x: 3.0, y: 4.0, z: 5.0 },
        FVector { x: 3.0, y: 4.0, z: 5.0 },
    ]);

    let return_value = fpoly.fix();

    assert_eq!(return_value, 0);
    assert_eq!(fpoly.vertices.len(), 0);
}

#[test]
fn fpoly_area_test() {
    let fpoly = FPoly::from_vertices(&[
        FVector { x: 0.0, y: 0.0, z: 0.0 },
        FVector { x: 1.0, y: 0.0, z: 0.0 },
        FVector { x: 1.0, y: 1.0, z: 0.0 },
    ]);

    let area = fpoly.area();

    assert_eq!(area, 0.5);
}

// A quad that is facing z-up, with it's center at the origin.
fn get_z_up_quad_fpoly() -> FPoly {
    FPoly::from_vertices(&[
        FVector { x: -1.0, y: -1.0, z: 0.0 },
        FVector { x: 1.0, y: -1.0, z: 0.0 },
        FVector { x: 1.0, y: 1.0, z: 0.0 },
        FVector { x: -1.0, y: 1.0, z: 0.0 },
    ])
}

#[test]
fn fpoly_split_with_plane_coplanar_test() {
    let fpoly = get_z_up_quad_fpoly();

    let split_type = fpoly.split_with_plane(
        FVector::new(0.0, 0.0, 0.0),
        Vector3::unit_z(),
        false);

    assert_eq!(split_type, ESplitType::Coplanar);
}

#[test]
fn fpoly_split_with_plane_back_test() {
    let fpoly = get_z_up_quad_fpoly();

    let split_type = fpoly.split_with_plane(
        FVector::new(0.0, 0.0, 1.0),
        Vector3::unit_z(),
        false
    );

    assert_eq!(split_type, ESplitType::Back);
}

#[test]
fn fpoly_split_with_plane_front_test() {
    let fpoly = get_z_up_quad_fpoly();

    let split_type = fpoly.split_with_plane(
        FVector::new(0.0, 0.0, -1.0),
        Vector3::unit_z(),
        false
    );

    assert_eq!(split_type, ESplitType::Front);
}

#[test]
fn fpoly_split_with_plane_split_test() {
    // Arrange
    let fpoly = get_z_up_quad_fpoly();
    let plane_base = FVector::new(0.0, 0.0, 0.0);
    let plane_normal = Vector3::unit_x();

    // Act
    let split_type = fpoly.split_with_plane(plane_base, plane_normal, false);

    // Assert
    let mut front_poly = FPoly::from_vertices(&[
        FVector::new(0.0, -1.0, 0.0),
        FVector::new(1.0, -1.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0)
    ]);
    front_poly.poly_flags.set(EPolyFlags::EdCut, true);
    let mut back_poly = FPoly::from_vertices(&[
        FVector::new(-1.0, -1.0, 0.0),
        FVector::new( 0.0, -1.0, 0.0),
        FVector::new( 0.0, 1.0, 0.0),
        FVector::new(-1.0, 1.0, 0.0)
    ]);
    back_poly.poly_flags.set(EPolyFlags::EdCut, true);
    assert_eq!(split_type, ESplitType::Split(front_poly, back_poly))
}

#[test]
fn fpoly_split_with_plane_quad_to_tris() {
    // A z-up quad.
    let fpoly = get_z_up_quad_fpoly();
    // A plane that is diagonally facing out from the origin towards one of the corner points.
    let plane_base = FVector::new(0.0, 0.0, 0.0);
    let plane_normal = FVector::new(1.0, 1.0, 0.0).normalize();

    let split_type = fpoly.split_with_plane(plane_base, plane_normal, false);

    let mut front_poly = FPoly::from_vertices(&[
        FVector::new(1.0, -1.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(-1.0, 1.0, 0.0)
    ]);
    front_poly.poly_flags.set(EPolyFlags::EdCut, true);

    let mut back_poly = FPoly::from_vertices(&[
        FVector::new(-1.0, 1.0, 0.0),
        FVector::new(-1.0, -1.0, 0.0),
        FVector::new(1.0, -1.0, 0.0)
    ]);
    back_poly.poly_flags.set(EPolyFlags::EdCut, true);

    assert_eq!(split_type, ESplitType::Split(front_poly, back_poly))
}

#[test]
fn fpoly_split_with_plane_fast_coplanar_test() {
    let fpoly = get_z_up_quad_fpoly();

    let split_type = fpoly.split_with_plane_fast(
        &FVector::new(0.0, 0.0, 0.0),
        &Vector3::unit_z());

    assert_eq!(split_type, ESplitType::Coplanar);
}

#[test]
fn fpoly_split_with_plane_fast_back_test() {
    let fpoly = get_z_up_quad_fpoly();

    let split_type = fpoly.split_with_plane_fast(
        &FVector::new(0.0, 0.0, 1.0),
        &Vector3::unit_z()
    );

    assert_eq!(split_type, ESplitType::Back);
}

#[test]
fn fpoly_split_with_plane_fast_front_test() {
    let fpoly = get_z_up_quad_fpoly();

    let split_type = fpoly.split_with_plane_fast(
        &FVector::new(0.0, 0.0, -1.0),
        &Vector3::unit_z()
    );

    assert_eq!(split_type, ESplitType::Front);
}

#[test]
fn fpoly_split_with_plane_fast_split_test() {
    // Arrange
    let fpoly = get_z_up_quad_fpoly();
    let plane_base = FVector::new(0.0, 0.0, 0.0);
    let plane_normal = Vector3::unit_x();

    // Act
    let split_type = fpoly.split_with_plane_fast(&plane_base, &plane_normal);

    // Assert
    let front_poly = FPoly::from_vertices(&[
        FVector::new(0.0, -1.0, 0.0),
        FVector::new(1.0, -1.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0)
    ]);
    let back_poly = FPoly::from_vertices(&[
        FVector::new(-1.0, -1.0, 0.0),
        FVector::new( 0.0, -1.0, 0.0),
        FVector::new( 0.0, 1.0, 0.0),
        FVector::new(-1.0, 1.0, 0.0)
    ]);
    assert_eq!(split_type, ESplitType::Split(front_poly, back_poly))
}

#[test]
fn fpoly_split_with_plane_fast_quad_to_tris() {
    // A z-up quad.
    let fpoly = get_z_up_quad_fpoly();
    // A plane that is diagonally facing out from the origin towards one of the corner points.
    let plane_base = FVector::new(0.0, 0.0, 0.0);
    let plane_normal = FVector::new(1.0, 1.0, 0.0).normalize();
    let split_type = fpoly.split_with_plane_fast(&plane_base, &plane_normal);

    // split_with_plane_fast does not remove duplicate vertices.
    let front_poly = FPoly::from_vertices(&[
        FVector::new(-1.0, 1.0, 0.0),
        FVector::new(1.0, -1.0, 0.0),
        FVector::new(1.0, -1.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(-1.0, 1.0, 0.0)
    ]);

    let back_poly = FPoly::from_vertices(&[
        FVector::new(-1.0, 1.0, 0.0),
        FVector::new(-1.0, -1.0, 0.0),
        FVector::new(1.0, -1.0, 0.0)
    ]);

    assert_eq!(split_type, ESplitType::Split(front_poly, back_poly))
}

#[test]
fn fpoly_split_in_half_triangle_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);

    let other_half = fpoly.split_in_half();

    // Cannot split a triangle in half, so no new polygon is created.
    assert_eq!(other_half, None);

    // Ensuyre the original polygon is unchanged.
    assert_eq!(fpoly.vertices.len(), 3);
}

#[test]
fn fpoly_split_in_half_quad_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);

    let other_half = fpoly.split_in_half();

    assert_eq!(fpoly.vertices.len(), 3);
    assert_eq!(fpoly.vertices.as_ref(), vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);
    assert_eq!(other_half.is_some(), true);
    assert_eq!(other_half.as_ref().unwrap().vertices.len(), 3);
    assert_eq!(other_half.as_ref().unwrap().vertices.as_ref(), vec![
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
        FVector::new(0.0, 0.0, 0.0),
    ]);
    let other_half = other_half.unwrap();
    assert_eq!(fpoly.poly_flags.contains(EPolyFlags::EdCut), true);
    assert_eq!(other_half.poly_flags.contains(EPolyFlags::EdCut), true);
}

#[test]
fn fpoly_calc_normal_triangle_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);

    let normal = fpoly.calc_normal();

    assert_eq!(normal.is_ok(), true);
    assert_eq!(normal, Ok(Vector3::unit_z()));
}

#[test]
fn fpoly_calc_normal_quad_test() {
    let mut fpoly = get_z_up_quad_fpoly();

    let normal = fpoly.calc_normal();

    assert_eq!(normal.is_ok(), true);
    assert_eq!(normal, Ok(Vector3::unit_z()));
}

#[test]
fn fpoly_calc_normal_degenerate_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(0.0, 0.0, 0.0),
    ]);

    let normal = fpoly.calc_normal();

    assert_eq!(normal.is_err(), true);
}

#[test]
fn fpoly_remove_colinears_convex_test() {
    let mut fpoly = get_z_up_quad_fpoly();

    let result = fpoly.remove_colinears();

    assert_eq!(result, RemoveColinearsResult::Convex);
    assert_eq!(fpoly.vertices.len(), 4);
}

#[test]
fn fpoly_remove_colinears_from_quad_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);

    let result = fpoly.remove_colinears();

    // Quad should have been reduced to a triangle.
    assert_eq!(result, RemoveColinearsResult::Convex);
    assert_eq!(fpoly.vertices.len(), 3);
}

#[test]
fn fpoly_remove_colinears_collapse_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0 + 1.0e-10, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
    ]);

    let result = fpoly.remove_colinears();

    // Degenerate polygon should be collapsed.
    assert_eq!(result, RemoveColinearsResult::Collapsed);
    assert_eq!(fpoly.vertices.len(), 0);
}

#[test]
fn fpoly_remove_colinears_concave_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);
    
    // Explicitly set the normal to z-up, otherwise remove colinears will not work
    // since the normal is (0, 0, 0).
    fpoly.normal = Vector3::unit_z();

    let result = fpoly.remove_colinears();

    assert_eq!(result, RemoveColinearsResult::Concave);
    // Concave polygon should not be modified.
    assert_eq!(fpoly.vertices.len(), 4);
}

#[test]
fn fpoly_remove_colinears_redundant_vertex_test() {
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),    // This vertex is redundant and should be removed.
        FVector::new(2.0, 0.0, 0.0),
    ]);

    assert_eq!(fpoly.remove_colinears(), RemoveColinearsResult::Convex);
    assert_eq!(fpoly.vertices.len(), 3);
    assert_eq!(fpoly.vertices.to_vec(), vec![
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(2.0, 0.0, 0.0),
    ]);
}

#[test]
fn fpoly_on_poly_point_coincident_test() {
    let fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);
    let point = FVector::new(0.0, 0.0, 0.0);
    assert_eq!(fpoly.on_poly(&point), true);
}

#[test]
fn fpoly_on_poly_point_in_middle_test() {
    let fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);
    let point = FVector::new(0.5, 0.5, 1.0);
    assert_eq!(fpoly.on_poly(&point), true);
}

#[test]
fn fpoly_on_poly_point_outside_test() {
    let fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);
    let point = FVector::new(2.0, 2.0, 1.0);
    assert_eq!(fpoly.on_poly(&point), false);
}

#[test]
fn fpoly_insert_vertex_middle_test() {
    // Arrange
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);
    let new_vertex = FVector::new(0.5, 0.5, 0.0);
    let insert_index = 2;

    // Act
    fpoly.insert_vertex(insert_index, &new_vertex);

    // Assert
    assert_eq!(fpoly.vertices.len(), 5);
    assert_eq!(fpoly.vertices[insert_index], new_vertex);
}

#[test]
fn fpoly_insert_vertex_end_test() {
    // Arrange
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);
    let new_vertex = FVector::new(0.5, 0.5, 0.0);
    let insert_index = 4;

    // Act
    fpoly.insert_vertex(insert_index, &new_vertex);

    // Assert
    assert_eq!(fpoly.vertices.len(), 5);
    assert_eq!(fpoly.vertices[insert_index], new_vertex);
}

#[test]
fn fpoly_split_quad_test() {
    // Arrange
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);
    let plane_base = FVector::new(0.5, 0.5, 0.0);
    let plane_normal = Vector3::unit_x();

    // Act
    let result = fpoly.split(&plane_normal, &plane_base, false);

    // Assert
    assert_eq!(result, 4);
    assert_eq!(fpoly.vertices.len(), result);
    assert_eq!(fpoly.vertices.to_vec(), vec![
        FVector::new(0.5, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.5, 1.0, 0.0),
    ]);
}

/// Test that a polygon that is transformed with the identity transformation
/// remains unchanged.
#[test]
fn fpoly_transform_identity_test() {
    // Arrange
    let mut fpoly = FPoly::from_vertices(&[
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);
    let coords = FModelCoords::new();
    let pre_subtract = FVector::new(0.0, 0.0, 0.0);
    let post_add = FVector::new(0.0, 0.0, 0.0);
    let orientation = 0.0;

    // Act
    fpoly.transform(&coords, &pre_subtract, &post_add, orientation);
    
    // Assert
    assert_eq!(fpoly.vertices.len(), 4);
    assert_eq!(fpoly.vertices.to_vec(), vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
    ]);
    assert_eq!(fpoly.normal, Vector3::unit_z());
}
