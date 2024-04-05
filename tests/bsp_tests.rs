use bdk_rs::coords::FModelCoords;
use bdk_rs::math::FVector;
use bdk_rs::bsp::{merge_coplanars, try_to_merge};
use bdk_rs::fpoly::{self, FPoly};
use bdk_rs::model::UModel;

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
