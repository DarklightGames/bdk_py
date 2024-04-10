use bdk_rs::math::FVector;
use bdk_rs::bsp::{EBspOptimization, merge_coplanars, try_to_merge, bsp_add_node, find_best_split};
use bdk_rs::fpoly::FPoly;
use bdk_rs::model::{EBspNodeFlags, UModel};

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

/// Creates a unit cube with 6 polygons, with normals pointing outwards.
fn create_unit_cube_polys() -> [FPoly; 6] {
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
    //
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
    // TODO: not sure about the winding order of the vertices.
    let polys: [FPoly; 6] = [
        FPoly::from_vertices(&vec![vertices[0], vertices[1], vertices[2], vertices[3]]),
        FPoly::from_vertices(&vec![vertices[4], vertices[7], vertices[6], vertices[5]]),
        FPoly::from_vertices(&vec![vertices[0], vertices[1], vertices[5], vertices[4]]),
        FPoly::from_vertices(&vec![vertices[3], vertices[7], vertices[6], vertices[2]]),
        FPoly::from_vertices(&vec![vertices[0], vertices[4], vertices[7], vertices[3]]),
        FPoly::from_vertices(&vec![vertices[1], vertices[2], vertices[6], vertices[5]]),
    ];

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
    println!("{:?}", merged_polys.iter().map(|f| f.vertices.to_vec()).collect::<Vec<Vec<FVector>>>());
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
    // 
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

#[test]
fn find_best_split_all_semisolids_test() {
}

#[test]
fn find_best_split_test() {
    // Arrange
    // Stack 3 identical polygons with 1 unit of difference between them along Z.
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
    // The middle polygon should be the best split polygon because it
    // has one polygon behind it and one polygon in front of it.
    assert_eq!(split_index, Some(1))
}


#[test]
fn bsp_add_node_root_node() {
    let mut model = UModel::new();
    let poly = FPoly::from_vertices(&vec![
        FVector::new(0.0, 0.0, 0.0),
        FVector::new(1.0, 0.0, 0.0),
        FVector::new(1.0, 1.0, 0.0),
    ]);
    bsp_add_node(&mut model, 
        0, 
        bdk_rs::bsp::ENodePlace::Root, 
        EBspNodeFlags::empty(),
        &poly
    );

    println!("{:?}", model.nodes);
}
