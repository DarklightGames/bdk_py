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
use bsp::{bsp_build, bsp_build_fpolys, bsp_calc_stats, bsp_merge_coplanars, bsp_opt_geom, EBspOptimization, FBspStats};
use fpoly::{EPolyFlags, FPOLY_VERTEX_THRESHOLD};
use model::{FBspNode, FBspSurf, FVert, UModel};
use pyo3::prelude::*;
use crate::fpoly::FPoly;

#[pyclass]
#[derive(Clone)]
struct Poly {
    vertices: Vec<(f32, f32, f32)>,
    origin: (f32, f32, f32),
    texture_u: (f32, f32, f32),
    texture_v: (f32, f32, f32),
    poly_flags: HashSet<String>,
    material_index: usize,
}

#[pymethods]
impl Poly {
    #[new]
    fn new(vertices: Vec<(f32, f32, f32)>, origin: (f32, f32, f32), texture_u: (f32, f32, f32), texture_v: (f32, f32, f32), poly_flags: HashSet<String>, material_index: usize) -> Self {
        Poly { vertices, origin, texture_u, texture_v, poly_flags, material_index }
    }

    fn split(&mut self) -> Option<Poly> {
        if self.vertices.len() < FPOLY_VERTEX_THRESHOLD + 1 {
            return None;
        }
        let mut other_half = self.clone();
        self.vertices.truncate(FPOLY_VERTEX_THRESHOLD);
        other_half.vertices.drain(1..FPOLY_VERTEX_THRESHOLD - 1);
        Some(other_half)
    }
}

fn create_circle_poly(num_verts: usize) -> Poly {
    let mut vertices: Vec<(f32, f32, f32)> = Vec::new();
    for i in 0..num_verts {
        let angle = 2.0 * std::f32::consts::PI * i as f32 / num_verts as f32;
        vertices.push((angle.cos(), angle.sin(), 0.0));
    }
    Poly { vertices, origin: (0.0, 0.0, 0.0), texture_u: (1.0, 0.0, 0.0), texture_v: (0.0, 1.0, 0.0), poly_flags: HashSet::new(), material_index: 0}
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

#[pymethods]
impl Brush {
    #[new]
    fn new(id: usize, name: String, polys: Vec<PyRef<Poly>>, poly_flags: HashSet<String>, csg_operation: &str) -> Self {
        // Create a copy of the polys and pass them to the brush.
        let polys: Vec<Poly> = polys.iter().map(|poly|
            Poly {
                vertices: poly.vertices.clone(),
                origin: poly.origin,
                texture_u: poly.texture_u,
                texture_v: poly.texture_v,
                poly_flags: poly.poly_flags.clone(),
                material_index: poly.material_index,
            }
        ).collect();

        Brush { id, name, polys, poly_flags, csg_operation: CsgOperation::from(csg_operation) }
    }
}

// Create a static string has mapping EPolyFlags, where the string is in SCREAMING_SNAKE_CASE.
static POLY_FLAGS: phf::Map<&'static str, u32> = phf::phf_map! {
    "INVISIBLE" => 0x00000001,
    "MASKED" => 0x00000002,
    "TRANSLUCENT" => 0x00000004,
    "NOT_SOLID" => 0x00000008,
    "ENVIRONMENT" => 0x00000010,
    "SEMI_SOLID" => 0x00000020,
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

impl From<&Poly> for FPoly {
    fn from(poly: &Poly) -> Self {
        let vertices: Vec<math::FVector> = poly.vertices.iter().map(|(x, y, z)| math::FVector::new(*x, *y, *z)).collect();
        let mut fpoly = FPoly::new();
        fpoly.vertices.extend(vertices);
        fpoly.base = math::FVector::new(poly.origin.0, poly.origin.1, poly.origin.2);
        fpoly.texture_u = math::FVector::new(poly.texture_u.0, poly.texture_u.1, poly.texture_u.2);
        fpoly.texture_v = math::FVector::new(poly.texture_v.0, poly.texture_v.1, poly.texture_v.2);
        fpoly.material_index = poly.material_index;
        _ = fpoly.calc_normal();
        fpoly.poly_flags = (&poly.poly_flags).into();
        fpoly
    }
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
struct BspSurface {
    #[pyo3(get)]
    pub base_point_index: usize,
    #[pyo3(get)]
    pub normal_index: usize,
    #[pyo3(get)]
    pub texture_u_index: usize,
    #[pyo3(get)]
    pub texture_v_index: usize,
    #[pyo3(get)]
    pub brush_id: usize,
    #[pyo3(get)]
    pub material_index: usize,
    #[pyo3(get)]
    pub brush_polygon_index: usize,
    #[pyo3(get)]
    pub poly_flags: u32,
    #[pyo3(get)]
    pub light_map_scale: f32,
}

impl From<&FBspSurf> for BspSurface {
    fn from(surface: &FBspSurf) -> Self {
        BspSurface {
            base_point_index: surface.base_point_index,
            normal_index: surface.normal_index,
            texture_u_index: surface.texture_u_index,
            texture_v_index: surface.texture_v_index,
            brush_id: surface.brush_id,
            brush_polygon_index: surface.brush_polygon_index.unwrap(),
            material_index: surface.material_index,
            poly_flags: surface.poly_flags.bits(),
            light_map_scale: surface.light_map_scale,
        }
    }
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
struct Vertex {
    #[pyo3(get)]
    pub point_index: usize,
    #[pyo3(get)]
    pub side_index: Option<usize>,  // TODO: this is actually the edge, not "side".
}

impl From<&FVert> for Vertex {
    fn from(vert: &FVert) -> Self {
        Vertex {
            point_index: vert.point_index,
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
#[derive(Clone, Copy, Debug)]
struct BspStats {
    #[pyo3(get)]
    pub depth_count: usize,
    #[pyo3(get)]
    pub depth_max: usize,
    #[pyo3(get)]
    pub front_leaves: usize,
    #[pyo3(get)]
    pub back_leaves: usize,
    #[pyo3(get)]
    pub branches: usize,
    #[pyo3(get)]
    pub leaves: usize,
    #[pyo3(get)]
    pub coplanars: usize,
    #[pyo3(get)]
    pub depth_average: f32,
}

impl From<FBspStats> for BspStats {
    fn from(stats: FBspStats) -> Self {
        BspStats {
            depth_count: stats.depth_count,
            depth_max: stats.depth_max,
            front_leaves: stats.front_leaves,
            back_leaves: stats.back_leaves,
            branches: stats.branches,
            leaves: stats.leaves,
            coplanars: stats.coplanars,
            depth_average: stats.depth_average,
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
    #[pyo3(get)]
    pub vectors: Vec<(f32, f32, f32)>,
    #[pyo3(get)]
    pub stats: BspStats,
}

impl TryFrom<&str> for EBspOptimization {
    type Error = &'static str;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        match value {
            "LAME" => Ok(EBspOptimization::Lame),
            "GOOD" => Ok(EBspOptimization::Good),
            "OPTIMAL" => Ok(EBspOptimization::Optimal),
            _ => Err("Invalid value, must be one of [LAME, GOOD, OPTIMAL]")
        }
    }
}

impl ToString for EBspOptimization {
    fn to_string(&self) -> String {
        match self {
            EBspOptimization::Lame => "LAME".to_string(),
            EBspOptimization::Good => "GOOD".to_string(),
            EBspOptimization::Optimal => "OPTIMAL".to_string(),
        }
    }
}

impl IntoPy<PyObject> for EBspOptimization {
    fn into_py(self, py: Python) -> PyObject {
        self.to_string().into_py(py)
    }
}

impl FromPyObject<'_> for EBspOptimization {
    fn extract(ob: &PyAny) -> PyResult<Self> {
        let string = ob.extract::<String>()?;
        EBspOptimization::try_from(string.as_str()).map_err(|_| PyErr::new::<pyo3::exceptions::PyValueError, _>("Invalid optimization value"))
    }
}

impl From<&UModel> for Model {
    fn from(model: &UModel) -> Self {
        let points: Vec<(f32, f32, f32)> = model.points.iter().map(|point| (point.x, point.y, point.z)).collect();
        let nodes: Vec<BspNode> = model.nodes.iter().map(|node| BspNode::from(node)).collect();
        let surfaces: Vec<BspSurface> = model.surfaces.iter().map(|surface| BspSurface::from(surface)).collect();
        let vertices: Vec<Vertex> = model.vertices.iter().map(|vert| Vertex::from(vert)).collect();
        let vectors: Vec<(f32, f32, f32)> = model.vectors.iter().map(|vector| (vector.x, vector.y, vector.z)).collect();
        let stats = BspStats::from(bsp_calc_stats(model));
        Model {
            points,
            nodes,
            surfaces,
            vertices,
            vectors,
            stats,
        }
    }
}

#[pyclass]
#[derive(Clone)]
pub struct BspBuildOptions {
    #[pyo3(get, set)]
    pub do_geometry: bool,
    #[pyo3(get, set)]
    pub do_bsp: bool,
    #[pyo3(get, set)]
    pub do_lighting: bool,
    #[pyo3(get, set)]
    pub dither_lightmaps: bool,
    #[pyo3(get, set)]
    pub lightmap_format: String,
    #[pyo3(get, set)]
    pub bsp_optimization: EBspOptimization,
    #[pyo3(get, set)]
    pub bsp_balance: u8,
    #[pyo3(get, set)]
    pub bsp_portal_bias: u8,
    #[pyo3(get, set)]
    pub should_optimize_geometry: bool,
}

impl Default for BspBuildOptions {
    fn default() -> Self {
        BspBuildOptions {
            do_geometry: true,
            do_bsp: true,
            do_lighting: true,
            dither_lightmaps: true,
            lightmap_format: "RGB8".to_string(),
            bsp_optimization: EBspOptimization::Lame,
            bsp_balance: 15,
            bsp_portal_bias: 70,
            should_optimize_geometry: true,
        }
    }
}

#[pymethods]
impl BspBuildOptions {
    #[new]
    fn new() -> Self {
        BspBuildOptions::default()
    }
}

/// Formats the sum of two numbers as string.
#[pyfunction]
// Have the progress callback be an optional argument.
fn csg_rebuild(brushes: Vec<PyRef<Brush>>, options: BspBuildOptions, progress_callback: Option<&Bound<PyAny>>) -> PyResult<Model> {
    use crate::csg::{ULevel, csg_rebuild};

    // Convert the Brushes to ABrushes and add them to the level brush list.
    let brushes: Vec<crate::brush::ABrush> = brushes.iter().map(|brush| brush.into()).collect();

    let mut level = ULevel::new(brushes);

    // Rebuild the CSG.
    csg_rebuild(&mut level, &options);

    if let Some(progress_callback) = progress_callback {
        progress_callback.call0()?;
    }

    if options.do_bsp {
        bsp_rebuild(&mut level.model, options);
    }

    let stats = bsp_calc_stats(&level.model);

    println!("BSP stats: {:?}", stats);
    
    Ok(Model::from(&level.model))
}

fn bsp_rebuild(model: &mut UModel, options: BspBuildOptions) {
    bsp_build_fpolys(model, true, 0);
    bsp_merge_coplanars(model, false, false);
    bsp_build(model, options.bsp_optimization, options.bsp_balance, options.bsp_portal_bias, bsp::BspRebuildMode::Nodes);
    //test_visibility
    bsp_opt_geom(model);
}

#[pymodule]
fn bdk_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Poly>()?;
    m.add_class::<Brush>()?;
    m.add_class::<Model>()?;
    m.add_class::<Vertex>()?;
    m.add_class::<BspSurface>()?;
    m.add_class::<BspNode>()?;
    m.add_class::<BspBuildOptions>()?;
    m.add_function(wrap_pyfunction!(csg_rebuild, m)?)?;
    Ok(())
}
