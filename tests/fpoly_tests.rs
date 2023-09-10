use bdk_bsp_python::{FPoly, FPlane, ESplitType, line_plane_intersection};
use cgmath::{Vector3, InnerSpace};

fn make_triangle_poly() -> FPoly {
    let vertices = [
        Vector3 { x: 0.0, y: 0.0, z: 0.0, },
        Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        Vector3 { x: 0.0, y: 1.0, z: 1.0, },
    ];
    let mut polygon = FPoly::new();
    polygon.vertices.clone_from_slice(vertices.as_slice());
    polygon.normal = Vector3 { x: 1.0, y: 0.0, z: 0.0, };
    polygon
}

fn make_square_poly() -> FPoly {
    let vertices = [
        Vector3 { x: 0.0, y: 0.0, z: 0.0, },
        Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        Vector3 { x: 0.0, y: 1.0, z: 1.0, },
        Vector3 { x: 0.0, y: 1.0, z: 0.0, },
    ];
    FPoly::new_from_vertices(vertices.as_slice())
}

#[test]
fn reverse_test() {
    let vertices = [
        Vector3 { x: 0.0, y: 0.0, z: 0.0, },
        Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        Vector3 { x: 0.0, y: 1.0, z: 1.0, },
    ];
    let mut polygon = FPoly::new_from_vertices(vertices.as_slice());
    assert_eq!(Vector3 { x: -1.0, y: 0.0, z: 0.0, }, polygon.normal);

    polygon.reverse();

    assert_eq!(Vector3 { x: 1.0, y: 0.0, z: 0.0, }, polygon.normal);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 1.0, }, polygon.vertices[0]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 1.0, }, polygon.vertices[1]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.0, }, polygon.vertices[2]);
}

#[test]
fn fix_degenerate_triangle() {
    let vertices = [
        Vector3 { x: 0.0, y: 0.0, z: 0.0, },
        Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        Vector3 { x: 0.0, y: 0.0, z: 1.0, },    // duplicate point
    ];
    let mut polygon = FPoly::new_from_vertices(vertices.as_slice());

    let new_vertex_count = polygon.fix();
    
    assert_eq!(0, new_vertex_count);
    assert_eq!(0, polygon.vertices.len());
}

#[test]
fn fix_square_with_one_duplicate_point() {
    let vertices = [
        Vector3 { x: 0.0, y: 0.0, z: 0.0, },
        Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        Vector3 { x: 0.0, y: 1.0, z: 1.0, },
        Vector3 { x: 0.0, y: 1.0, z: 1.0, },    // duplicate point
        Vector3 { x: 0.0, y: 1.0, z: 0.0, },
    ];
    let mut polygon = FPoly::new_from_vertices(vertices.as_slice());

    let new_vertex_count = polygon.fix();
    
    assert_eq!(4, new_vertex_count);
    assert_eq!(4, polygon.vertices.len(), "Vertex count should be 4, but is {}", polygon.vertices.len());
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.0, }, polygon.vertices[0]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 1.0, }, polygon.vertices[1]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 1.0, }, polygon.vertices[2]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 0.0, }, polygon.vertices[3]);
}

#[test]
fn split_with_plane_test_back() {
    // Split doesn't do anything because the polygon is entirely in behind the plane.
    let mut polygon = make_square_poly();
    let plane_base = Vector3 { x: 0.0, y: 0.0, z: 1.0, };
    let plane_normal = Vector3 { x: 0.0, y: 0.0, z: 1.0, };

    let split_type = polygon.split_with_plane(plane_base, plane_normal, None, None, false);

    assert_eq!(ESplitType::BACK, split_type);
}

#[test]
fn split_with_plane_test_front() {
    // Split doesn't do anything because the polygon is entirely in front of the plane.
    let mut polygon = make_square_poly();
    let plane_base = Vector3 { x: 0.0, y: 0.0, z: 0.0, };
    let plane_normal = Vector3 { x: 0.0, y: 0.0, z: 1.0, };
    
    let split_type = polygon.split_with_plane(plane_base, plane_normal, None, None, false);

    assert_eq!(ESplitType::FRONT, split_type);
}

#[test]
fn split_square_into_triangles_with_plane() {
    let mut polygon = make_square_poly();
    let plane_base = Vector3 { x: 0.0, y: 0.0, z: 0.0, };
    let plane_normal = Vector3 { x: 0.0, y: -1.0, z: 1.0, }.normalize();

    let mut front_poly = FPoly::new();
    let mut back_poly = FPoly::new();
    let split_type = polygon.split_with_plane(plane_base, plane_normal, Some(&mut front_poly), Some(&mut back_poly), false);

    assert!(split_type == ESplitType::SPLIT);
    assert!(front_poly.vertices.len() == 3);
    assert!(back_poly.vertices.len() == 3);
}

#[test]
fn split_with_plane_test_split() {
    let mut polygon = make_square_poly();
    let plane_base = Vector3 { x: 0.0, y: 0.0, z: 0.5, };
    let plane_normal = Vector3 { x: 0.0, y: 0.0, z: 1.0, };
    let mut front_poly = FPoly::new();
    let mut back_poly = FPoly::new();

    let split_type = polygon.split_with_plane(plane_base, plane_normal, Some(&mut front_poly), Some(&mut back_poly), false);

    assert_eq!(ESplitType::SPLIT, split_type);
    assert_eq!(4, front_poly.vertices.len());
    assert_eq!(4, back_poly.vertices.len());
    assert_eq!(front_poly.normal, back_poly.normal);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.5 }, front_poly.vertices[0]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 1.0 }, front_poly.vertices[1]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 1.0 }, front_poly.vertices[2]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 0.5 }, front_poly.vertices[3]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.0 }, back_poly.vertices[0]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.5 }, back_poly.vertices[1]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 0.5 }, back_poly.vertices[2]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 0.0 }, back_poly.vertices[3]);
}

#[test]
fn split_with_plane_test_coplanar() {
    // Split doesn't do anything because the polygon is entirely in front of the plane.
    let mut polygon = make_square_poly();
    let plane_base = Vector3 { x: 0.0, y: 0.0, z: 0.0, };
    let plane_normal = Vector3 { x: 1.0, y: 0.0, z: 0.0, };
    let split_type = polygon.split_with_plane(plane_base, plane_normal, None, None, false);
    assert_eq!(ESplitType::COPLANAR, split_type);
}

#[test]
fn split_with_plane_fast_test_coplanar() {
    // Split doesn't do anything because the polygon is entirely in front of the plane.
    let polygon = make_square_poly();
    let plane = FPlane {
        normal: Vector3 { x: 1.0, y: 0.0, z: 0.0, },
        distance: 0.0,
    };

    let split_type = polygon.split_with_plane_fast(plane, None, None);

    assert_eq!(ESplitType::COPLANAR, split_type);
}

#[test]
fn split_with_plane_fast_test_back() {
    // Split doesn't do anything because the polygon is entirely in front of the plane.
    let polygon = make_square_poly();
    let plane = FPlane {
        normal: Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        distance: 1.0,
    };

    let split_type = polygon.split_with_plane_fast(plane, None, None);

    assert_eq!(ESplitType::BACK, split_type);
}

#[test]
fn split_with_plane_fast_test_front() {
    // Split doesn't do anything because the polygon is entirely in front of the plane.
    let polygon = make_square_poly();
    let plane = FPlane {
        normal: Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        distance: 0.0,
    };
    
    let split_type = polygon.split_with_plane_fast(plane, None, None);

    assert_eq!(ESplitType::FRONT, split_type);
}

#[test]
fn split_with_plane_fast_test_split() {
    let polygon = make_square_poly();
    let plane = FPlane {
        normal: Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        distance: 0.5,
    };
    let mut front_poly = FPoly::new();
    let mut back_poly = FPoly::new();

    let split_type = polygon.split_with_plane_fast(plane, Some(&mut front_poly), Some(&mut back_poly));

    assert_eq!(ESplitType::SPLIT, split_type);
    assert_eq!(4, front_poly.vertices.len());
    assert_eq!(4, back_poly.vertices.len());
    assert_eq!(front_poly.normal, back_poly.normal);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.5 }, front_poly.vertices[0]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 1.0 }, front_poly.vertices[1]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 1.0 }, front_poly.vertices[2]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 0.5 }, front_poly.vertices[3]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.0 }, back_poly.vertices[0]);
    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.5 }, back_poly.vertices[1]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 0.5 }, back_poly.vertices[2]);
    assert_eq!(Vector3 { x: 0.0, y: 1.0, z: 0.0 }, back_poly.vertices[3]);
}

#[test]
fn line_plane_intersection_test() {
    let point1 = Vector3 { x: 0.0, y: 0.0, z: -1.0, };
    let point2 = Vector3 { x: 0.0, y: 0.0, z: 1.0, };
    let plane = FPlane {
        normal: Vector3 { x: 0.0, y: 0.0, z: 1.0, },
        distance: 0.5,
    };

    let intersection = line_plane_intersection(&point1, &point2, &plane);

    assert_eq!(Vector3 { x: 0.0, y: 0.0, z: 0.5 }, intersection);
}

#[test]
fn split_in_half_test() {
    let mut polygon = make_square_poly();

    let other_polygon = polygon.split_in_half(); // split_in_half modifies the original `polygon` and returns a new one

    assert_eq!(3, polygon.vertices.len());
    assert_eq!(3, other_polygon.vertices.len());
}

// #[test]
// fn split_in_half_triangle_test() {
//     let mut polygon = make_test_triangle();
//     let other_polygon = polygon.split_in_half();
//     assert_eq!(3, polygon.vertex_count);
//     assert_eq!(3, other_polygon.vertex_count);
// }
