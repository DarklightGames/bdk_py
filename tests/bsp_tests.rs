use std::io::Write;

use bdk_py::brush::ABrush;
use bdk_py::math::FVector;
use bdk_py::bsp::{bsp_add_node, bsp_brush_csg, bsp_validate_brush, find_best_split, merge_coplanars, try_to_merge, EBspOptimization, ENodePlace};
use bdk_py::fpoly::{EPolyFlags, FPoly};
use bdk_py::model::{EBspNodeFlags, UModel};

#[test]
fn try_to_merge_disjoint_test() {
    // Test 1: Polygons are not mergeable (disjointed).
    let mut poly1 = FPoly::from_vertices(&vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0)
    ]);

    let mut poly2 = FPoly::from_vertices(&vec![
        FVector::new(0.0, 1.0, 0.0),
        FVector::new(0.0, 2.0, 0.0),
        FVector::new(1.0, 2.0, 0.0)
    ]);

    assert_eq!(try_to_merge(&mut poly1, &mut poly2), false);
    assert_eq!(poly1.vertices.len(), 3);
    assert_eq!(poly2.vertices.len(), 3);
}

#[test]
fn try_to_merge_two_triangles_sharing_and_edge_test() {
    // The two triangles share an edge.
    let mut poly1 = FPoly::from_vertices(&vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);
    let mut poly2 = FPoly::from_vertices(&vec![
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(2.0, 0.0, 0.0),
    ]);
    assert_eq!(try_to_merge(&mut poly1, &mut poly2), true);
    assert_eq!(poly1.vertices.len(), 3);
    assert_eq!(poly1.vertices.to_vec(), vec![
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(2.0, 0.0, 0.0),
    ]);
    assert_eq!(poly2.vertices.len(), 0);    // Poly2 is merged into poly1.
}

#[test]
fn try_to_merge_two_triangles_sharing_vertex_test() {
    // The two triangles share a vertex.
    let mut poly1 = FPoly::from_vertices(&vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);
    let mut poly2 = FPoly::from_vertices(&vec![
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(2.0, 1.0, 0.0),
        FVector::new(2.0, 2.0, 0.0),
    ]);
    assert_eq!(try_to_merge(&mut poly1, &mut poly2), false);
    assert_eq!(poly1.vertices.len(), 3);
    assert_eq!(poly2.vertices.len(), 3);
}

#[test]
fn try_to_merge_identical_triangles_test() {
    // The two triangles are identical.
    // The original algorithm does not handle this case, and the polygons should not merged.
    let mut poly1 = FPoly::from_vertices(&vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);
    let mut poly2 = poly1.clone();
    assert_eq!(try_to_merge(&mut poly1, &mut poly2), false);
}

fn create_cube_polys(min: FVector, extents: FVector) -> Vec<FPoly> {
    create_cube_polys_with_poly_flags(min, extents, EPolyFlags::empty())
}

/// Creates a unit cube with 6 polygons, with normals pointing outwards.
fn create_cube_polys_with_poly_flags(min: FVector, extents: FVector, poly_flags: EPolyFlags) -> Vec<FPoly> {
    let vertices = vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
        FVector::new(0.0, 1.0, 0.0),
        FVector::new(0.0, 0.0, 1.0),
        FVector::new(1.0, 0.0, 1.0),
        FVector::new(1.0, 1.0, 1.0),
        FVector::new(0.0, 1.0, 1.0),
    ];

    // Apply the min and extents to the vertices.
    let vertices = vertices.iter().map(|v| {
        FVector::new(
            min.x + v.x * extents.x,
            min.y + v.y * extents.y,
            min.z + v.z * extents.z,
        )
    }).collect::<Vec<FVector>>();

    // Diagram of a unit cube:
    //
    //       7--------6
    //      /|       /|
    //     / |      / |
    //    4--------5  |
    //    |  3-----|--2
    //    | /      | /
    //    |/       |/
    //    0--------1
    let mut poly_vertex_indices: [Vec<usize>; 6] = [
        vec![0, 1, 2, 3],
        vec![4, 7, 6, 5],
        vec![0, 4, 5, 1],
        vec![2, 6, 7, 3],
        vec![3, 7, 4, 0],
        vec![1, 5, 6, 2],
    ];
    // // Reverse the order of all the faces to flip the normals.
    for indices in poly_vertex_indices.iter_mut() {
        indices.reverse();
    }

    let polys = poly_vertex_indices.iter().map(|indices| {
        let mut poly = FPoly::from_vertices(&indices.iter().map(|i| vertices[*i]).collect::<Vec<FVector>>());
        poly.poly_flags = poly_flags;
        poly
    }).collect::<Vec<FPoly>>();

    polys
}

fn create_quad_grid(width: usize, height: usize) -> Vec<FPoly> {
    let mut polys: Vec<FPoly> = Vec::new();
    for y in 0..height {
        for x in 0..width {
            polys.push(FPoly::from_vertices(&vec![
                FVector::new(x as f32, y as f32, 0.0),
                FVector::new((x + 1) as f32, y as f32, 0.0),
                FVector::new((x + 1) as f32, (y + 1) as f32, 0.0),
                FVector::new(x as f32, (y + 1) as f32, 0.0),
            ]));
        }
    }
    polys
}

#[test]
fn merge_coplanars_quad_grid_test() {
    // Arrange
    let mut polys = create_quad_grid(2, 2);
    let poly_indices = [0, 1, 2, 3];

    // Act
    let merge_count = merge_coplanars(&mut polys, &poly_indices);

    // Assert
    assert_eq!(merge_count, 3);
    let merged_polys = polys.iter().filter(|p| p.vertices.len() > 0).collect::<Vec<&FPoly>>();
    assert_eq!(merged_polys.len(), 1);
    assert_eq!(merged_polys[0].vertices.to_vec(), vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(2.0, 0.0, 0.0),
        FVector::new(2.0, 2.0, 0.0),
        FVector::new(0.0, 2.0, 0.0),
    ]);
}

#[test]
fn merge_coplanars_quad_grid_with_skipped_index_test() {
    // Arrange
    let mut polys = create_quad_grid(2, 2);
    let poly_indices = [0, 1, 3];

    // Act
    let merge_count = merge_coplanars(&mut polys, &poly_indices);

    // Assert
    assert_eq!(merge_count, 1);
    let merged_polys = polys.iter().filter(|p| p.vertices.len() > 0).collect::<Vec<&FPoly>>();
    assert_eq!(merged_polys.len(), 3);
    assert_eq!(merged_polys[0].vertices.to_vec(), vec![
        FVector::new(0.0, 1.0, 0.0),
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(2.0, 0.0, 0.0),
        FVector::new(2.0, 1.0, 0.0),
    ]);
    assert_eq!(merged_polys[1].vertices.to_vec(), polys[2].vertices.to_vec());
    assert_eq!(merged_polys[2].vertices.to_vec(), polys[3].vertices.to_vec());
}

#[test]
fn find_best_split_single_poly_test() {
    // Arrange
    let polys = vec![
        FPoly::from_vertices(&vec![
            FVector::new(0.0, 0.0, 0.0),
            FVector::new(0.0, 1.0, 0.0),
            FVector::new(1.0, 1.0, 0.0),
            FVector::new(1.0, 0.0, 0.0),
        ])
    ];

    // Act
    let split_index = find_best_split(&polys,  EBspOptimization::Optimal, 50, 50);

    // Assert
    assert_eq!(split_index, Some(0));
}

/// Test the find_best_split function with a cube where all the polygons are semisolids.
/// 
/// It must pick a polygon, even if it is a semisolid.
#[test]
fn find_best_split_all_semisolids_test() {
    // Arrange
    let polys = create_cube_polys_with_poly_flags(FVector::new(0.0, 0.0, 0.0), FVector::new(1.0, 1.0, 1.0), EPolyFlags::Semisolid);

    // Act
    let split_index = find_best_split(&polys, EBspOptimization::Lame, 70, 0);

    // Assert
    assert_eq!(split_index, Some(0));
}

/// Test the find_best_split function with a cube.
/// The best split is any of the 6 faces of the cube, but the first face is chosen.
#[test]
fn find_best_split_cube_test() {
    // Arrange
    let polys = create_cube_polys(FVector::new(0.0, 0.0, 0.0), FVector::new(1.0, 1.0, 1.0));

    // Act
    let split_index = find_best_split(&polys, EBspOptimization::Lame, 70, 0);

    // Assert
    assert_eq!(split_index, Some(0));
}

/// Stack 3 identical polygons with 1 unit of difference between them along Z.
/// 
/// The middle polygon should be the best split polygon because it has one
/// polygon behind it and one polygon in front of it.
#[test]
fn find_best_split_test() {
    // Arrange
    let polys = vec![
        FPoly::from_vertices(&[
            FVector::new(0.0, 0.0, 0.0),
            FVector::new(0.0, 1.0, 0.0),
            FVector::new(1.0, 1.0, 0.0),
            FVector::new(1.0, 0.0, 0.0)]),
        FPoly::from_vertices(&[
            FVector::new(0.0, 0.0, 1.0),
            FVector::new(0.0, 1.0, 1.0),
            FVector::new(1.0, 1.0, 1.0),
            FVector::new(1.0, 0.0, 1.0)]),
        FPoly::from_vertices(&[
            FVector::new(0.0, 0.0, 2.0),
            FVector::new(0.0, 1.0, 2.0),
            FVector::new(1.0, 1.0, 2.0),
            FVector::new(1.0, 0.0, 2.0)]),
    ];

    // Act
    let split_index = find_best_split(&polys, EBspOptimization::Optimal, 50, 50);

    // Assert
    assert_eq!(split_index, Some(1))
}

#[test]
fn bsp_add_node_root_node() {
    let mut model = UModel::new();
    let mut poly = FPoly::from_vertices(&vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);

    bsp_add_node(&mut model, 
        None, 
        ENodePlace::Root, 
        EBspNodeFlags::empty(),
        &poly
    );

    assert!(model.nodes.len() == 1); 
}

fn output_obj(model: &UModel, path: &str) {
    // Output the surfaces to an OBJ file for debugging.
    if let Ok(mut obj) = std::fs::File::create(path) {
        for point in &model.points {
            obj.write(format!("v {} {} {}\n", point.x, point.y, point.z).as_bytes());
        }
    
        for node in &model.nodes {
            if node.vertex_count == 0 {
                continue;
            }
    
            let vertices = &model.vertices[node.vertex_pool_index..node.vertex_pool_index + node.vertex_count];
            let point_indices = vertices.iter().map(|vertex| vertex.vertex_index).collect::<Vec<usize>>();
            let face_indices = &point_indices.iter().map(|i| (i + 1).to_string()).collect::<Vec<String>>().join(" ");
    
            obj.write(format!("f {}\n", face_indices).as_bytes());
        }
    } else {
        println!("Failed to create OBJ file.");
    }
}

#[test]
fn bsp_brush_subtract_and_add_test() {
    // Arrange
    let mut model = UModel::new();

    // Create the main subtraction brush.
    let polys = create_cube_polys(FVector::new(0.0, 0.0, 0.0), FVector::new(1.0, 1.0, 1.0));
    let mut subtraction_brush = ABrush {
        model: UModel::new_from_polys(&polys),
        location: FVector::new(0.0, 0.0, 0.0),
        pre_pivot: FVector::new(0.0, 0.0, 0.0),
        csg_operation: bdk_py::bsp::ECsgOper::Subtract,
        poly_flags: EPolyFlags::empty(),
    };
    bsp_validate_brush(&mut subtraction_brush.model, false);

    // TODO: all the normals are flipped on the addition brush...

    // Create a smaller additive brush inside the subtraction brush.
    let mut polys = create_cube_polys(FVector::new(0.5, 0.5, 0.5), FVector::new(1.0, 1.0, 1.0));
    // Flip the normals.
    for poly in polys.iter_mut() {
        poly.normal = -poly.normal;
    }
    let mut addition_brush = ABrush {
        model: UModel::new_from_polys(&polys),
        location: FVector::new(0.0, 0.0, 0.0),
        pre_pivot: FVector::new(0.0, 0.0, 0.0),
        csg_operation: bdk_py::bsp::ECsgOper::Add,
        poly_flags: EPolyFlags::empty(),
    };
    bsp_validate_brush(&mut addition_brush.model, false);

    // Act
    model.is_root_outside = false;

    bsp_brush_csg(&subtraction_brush, &mut model, EPolyFlags::empty(), bdk_py::bsp::ECsgOper::Subtract, false);

    // model.is_root_outside = true;

    // addition_brush.model.is_root_outside = false;

    bsp_brush_csg(&addition_brush, &mut model, EPolyFlags::empty(), bdk_py::bsp::ECsgOper::Add, false);

    output_obj(&model, "subtract_and_add.obj");
}

#[test]
fn bsp_brush_csg_subtract_test() {
    // Arrange
    let mut model = UModel::new();
    let polys = create_cube_polys(FVector::new(0.0, 0.0, 0.0), FVector::new(1.0, 1.0, 1.0));
    let mut brush = ABrush {
        model: UModel::new_from_polys(&polys),
        location: FVector::new(5.0, 0.0, 0.0),
        pre_pivot: FVector::new(0.0, 0.0, 0.0),
        csg_operation: bdk_py::bsp::ECsgOper::Subtract,
        poly_flags: EPolyFlags::empty(),
    };

    bsp_validate_brush(&mut brush.model, false);

    model.is_root_outside = false;  // TODO: have this done in ULevel

    // Act
    bsp_brush_csg(&brush, &mut model, EPolyFlags::empty(), bdk_py::bsp::ECsgOper::Subtract, false);

    // Assert
    assert_eq!(model.nodes.len(), 6);
    assert_eq!(model.surfaces.len(), 6);

    output_obj(&model, "subtract.obj");
}