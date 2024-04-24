#![feature(get_many_mut)]

pub mod fpoly;
pub mod math;
pub mod bsp;
pub mod model;
pub mod box_;
pub mod sphere;
pub mod coords;
pub mod brush;
pub mod csg;

use std::collections::HashSet;
use fpoly::{EPolyFlags, FPOLY_MAX_VERTICES, FPOLY_VERTEX_THRESHOLD};
use model::{FBspNode, FBspSurf, FVert, UModel};
use pyo3::prelude::*;
use crate::fpoly::FPoly;

#[pyclass]
#[derive(Clone)]
struct Poly {
    vertices: Vec<(f32, f32, f32)>
}

#[pymethods]
impl Poly {
    #[new]
    fn new(vertices: Vec<(f32, f32, f32)>) -> Self {
        Poly { vertices }
    }

    fn split(&mut self) -> Option<Poly> {
        if self.vertices.len() < FPOLY_VERTEX_THRESHOLD + 1 {
            return None;
        }
        let mut other_half = self.clone();
        self.vertices.truncate(FPOLY_VERTEX_THRESHOLD);
        println!("self.vertices.len() = {}", self.vertices.len());
        other_half.vertices.drain(1..FPOLY_VERTEX_THRESHOLD - 1);
        println!("other_half.vertices.len() = {}", other_half.vertices.len());
        Some(other_half)
    }
}

fn create_circle_poly(num_verts: usize) -> Poly {
    let mut vertices: Vec<(f32, f32, f32)> = Vec::new();
    for i in 0..num_verts {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / num_verts as f32;
        vertices.push((angle.cos(), angle.sin(), 0.0));
    }
    Poly { vertices }
}

#[test]
fn brush_ensure_polys_no_degenerates_test() {
    // Create a 32-sided circle polygon.
    for i in 3..33 {
        let poly = create_circle_poly(i);
        let mut brush = Brush { id: 0, name: "Test".to_string(), polys: vec![poly], poly_flags: HashSet::new(), csg_operation: CsgOperation::Add };
        brush.ensure_polys();
        
        // None of the polys should be degenerate.
        for (j, poly) in brush.polys.iter().enumerate() {
            assert!(poly.vertices.len() >= 3, "Shape with {} verts, split poly {} has less than 3 vertices", i, j);
        }
    }
}

#[test]
fn brush_ensure_polys_17_sided_cylinder_test() {
    // Create a 17-sided cylinder.
    let poly = create_circle_poly(16);
    let mut brush = Brush { id: 0, name: "Test".to_string(), polys: vec![poly], poly_flags: HashSet::new(), csg_operation: CsgOperation::Add };
    brush.ensure_polys();

    assert_eq!(brush.polys.len(), 2, "Expected 2 polys, got {}", brush.polys.len());
    assert_eq!(brush.polys[0].vertices.len(), 14);
    assert_eq!(brush.polys[1].vertices.len(), 4);
}


#[pyclass]
#[derive(Clone, Copy, Debug)]
enum CsgOperation {
    Add = 0,
    Subtract = 1,
}

impl From<CsgOperation> for crate::bsp::ECsgOper {
    fn from(csg_operation: CsgOperation) -> Self {
        match csg_operation {
            CsgOperation::Add => crate::bsp::ECsgOper::Add,
            CsgOperation::Subtract => crate::bsp::ECsgOper::Subtract,
        }
    }
}

impl From<&str> for CsgOperation {
    fn from(s: &str) -> Self {
        match s {
            "ADD" => CsgOperation::Add,
            "SUBTRACT" => CsgOperation::Subtract,
            &_ => panic!("Invalid CSG operation")
        }
    }
}

#[pyclass]
struct Brush {
    id: usize,
    name: String,
    polys: Vec<Poly>,
    poly_flags: HashSet<String>,
    csg_operation: CsgOperation,
}

impl Brush {
    pub(crate) fn ensure_polys(&mut self) {
        let mut poly_stack = self.polys.clone();
        poly_stack.reverse();
        self.polys.clear();
        while let Some(mut poly) = poly_stack.pop() {
            let split_result = poly.split();
            self.polys.push(poly);
            if let Some(other_half) = split_result {
                poly_stack.push(other_half);
            }
        }
    }
}

// Create a static string has mapping EPolyFlags, where the string is in SCREAMING_SNAKE_CASE.
static POLY_FLAGS: phf::Map<&'static str, u32> = phf::phf_map! {
    "INVISIBLE" => 0x00000001,
    "MASKED" => 0x00000002,
    "TRANSLUCENT" => 0x00000004,
    "NOT_SOLID" => 0x00000008,
    "ENVIRONMENT" => 0x00000010,
    "SEMISOLID" => 0x00000020,
    "MODULATED" => 0x00000040,
    "FAKE_BACKDROP" => 0x00000080,
    "TWO_SIDED" => 0x00000100,
    "NO_SMOOTH" => 0x00000800,
    "ALPHA_TEXTURE" => 0x00001000,
    "FLAT" => 0x00004000,
    "NO_MERGE" => 0x00010000,
    "NO_Z_TEST" => 0x00020000,
    "ADDITIVE" => 0x00040000,
    "SPECIAL_LIT" => 0x00100000,
    "WIREFRAME" => 0x00200000,
    "UNLIT" => 0x00400000,
    "PORTAL" => 0x04000000,
    "ANTI_PORTAL" => 0x08000000,
    "MIRRORED" => 0x20000000,
};

impl From<&HashSet<String>> for EPolyFlags {
    fn from(poly_flags: &HashSet<String>) -> Self {
        let mut flags = EPolyFlags::empty();
        for flag in poly_flags.iter() {
            if let Some(bits) = POLY_FLAGS.get(flag.as_str()) {
                flags |= EPolyFlags::from_bits(*bits).unwrap();
            }
        }
        flags
    }
}

impl From<&PyRef<'_, Brush>> for crate::brush::ABrush {
    fn from(brush: &PyRef<Brush>) -> Self {
        // Incoming polys may have more than the maximum number of vertices.
        // Therefore, we need to split the large polys into smaller ones.
        // Try to split all of the polys, recursively.
        let mut brush_poly_stack = brush.polys.clone();
        brush_poly_stack.reverse();
        let mut brush_polys: Vec<Poly> = Vec::new();
        while let Some(mut poly) = brush_poly_stack.pop() {
            match poly.split() {
                Some(other_half) => {
                    brush_polys.push(poly);
                    brush_poly_stack.push(other_half);
                },
                None => {
                    brush_polys.push(poly);
                }
            }
        }

        let polys: Vec<FPoly> = brush_polys.iter().map(|poly| FPoly::from(poly)).collect();

        crate::brush::ABrush::new(
            brush.id,
            brush.name.clone(),
            polys.as_slice(),
            EPolyFlags::from(&brush.poly_flags),
            brush.csg_operation.into())
    }
}

#[pymethods]
impl Brush {
    #[new]
    fn new(id: usize, name: String, polys: Vec<PyRef<Poly>>, poly_flags: HashSet<String>, csg_operation: &str) -> Self {
        // Create a copy of the polys and pass them to the brush.
        let polys: Vec<Poly> = polys.iter().map(|poly|
            Poly { vertices: poly.vertices.clone() }
        ).collect();

        Brush { id, name, polys, poly_flags, csg_operation: CsgOperation::from(csg_operation) }
    }
}

impl From<&Poly> for FPoly {
    fn from(poly: &Poly) -> Self {
        let vertices: Vec<math::FVector> = poly.vertices.iter().map(|(x, y, z)| math::FVector::new(*x, *y, *z)).collect();
        FPoly::from_vertices(&vertices)
    }
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
struct BspSurface {
    #[pyo3(get)]
    pub normal_index: usize,
    #[pyo3(get)]
    pub texture_u_index: usize,
    #[pyo3(get)]
    pub texture_v_index: usize,
    #[pyo3(get)]
    pub brush_id: usize,
    #[pyo3(get)]
    pub brush_polygon_index: usize,
}

impl From<&FBspSurf> for BspSurface {
    fn from(surface: &FBspSurf) -> Self {
        BspSurface {
            normal_index: surface.normal_index,
            texture_u_index: surface.texture_u_index,
            texture_v_index: surface.texture_v_index,
            brush_id: surface.brush_id,
            brush_polygon_index: surface.brush_polygon_index.unwrap(),
        }
    }
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
struct Vertex {
    #[pyo3(get)]
    pub vertex_index: usize,
    #[pyo3(get)]
    pub side_index: Option<usize>,
}

impl From<&FVert> for Vertex {
    fn from(vert: &FVert) -> Self {
        Vertex {
            vertex_index: vert.vertex_index,
            side_index: vert.side_index,
        }
    }
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
struct BspNode {
    #[pyo3(get)]
    pub vertex_pool_index: usize,
    #[pyo3(get)]
    pub vertex_count: usize,
    #[pyo3(get)]
    pub surface_index: usize,
}

impl From<&FBspNode> for BspNode {
    fn from(node: &FBspNode) -> Self {
        BspNode {
            vertex_pool_index: node.vertex_pool_index,
            vertex_count: node.vertex_count,
            surface_index: node.surface_index,
        }
    }
}

#[pyclass]
struct Model {
    #[pyo3(get)]
    pub points: Vec<(f32, f32, f32)>,
    #[pyo3(get)]
    pub nodes: Vec<BspNode>,
    #[pyo3(get)]
    pub surfaces: Vec<BspSurface>,
    #[pyo3(get)]
    pub vertices: Vec<Vertex>,
}

impl From<&UModel> for Model {
    fn from(model: &UModel) -> Self {
        let points: Vec<(f32, f32, f32)> = model.points.iter().map(|point| (point.x, point.y, point.z)).collect();
        let nodes: Vec<BspNode> = model.nodes.iter().map(|node| BspNode::from(node)).collect();
        let surfaces: Vec<BspSurface> = model.surfaces.iter().map(|surface| BspSurface::from(surface)).collect();
        let vertices: Vec<Vertex> = model.vertices.iter().map(|vert| Vertex::from(vert)).collect();
        Model {
            points,
            nodes,
            surfaces,
            vertices,
        }
    }
}

/// Formats the sum of two numbers as string.
#[pyfunction]
fn csg_rebuild(brushes: Vec<PyRef<Brush>>) -> PyResult<Model> {
    use crate::csg::{ULevel, csg_rebuild};

    // Convert the Brushes to ABrushes and add them to the level brush list.
    let brushes: Vec<crate::brush::ABrush> = brushes.iter().map(|brush| brush.into()).collect();

    let mut level = ULevel::new(brushes);

    // Rebuild the CSG.
    csg_rebuild(&mut level);
    
    Ok(Model::from(&level.model))
}

#[pymodule]
fn bdk_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Poly>()?;
    m.add_class::<Brush>()?;
    m.add_class::<Model>()?;
    m.add_class::<Vertex>()?;
    m.add_class::<BspSurface>()?;
    m.add_class::<BspNode>()?;
    m.add_function(wrap_pyfunction!(csg_rebuild, m)?)?;
    Ok(())
}