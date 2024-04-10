#![feature(get_many_mut)]

pub mod fpoly;
pub mod math;
pub mod bsp;
pub mod model;
pub mod box_;
pub mod sphere;
pub mod coords;
pub mod brush;

use pyo3::prelude::*;

#[pyclass]
struct Poly {
    vertices: Vec<(f32, f32, f32)>
}

#[pymethods]
impl Poly {
    #[new]
    fn new(vertices: Vec<(f32, f32, f32)>) -> Self {
        Poly { vertices }
    }
}

#[pyclass]
#[derive(Clone, Copy)]
enum CsgOperation {
    Add = 0,
    Subtract = 1,
}

#[pyclass]
struct Brush {
    polys: Vec<Py<Poly>>,
    poly_flags: usize,
    csg_operation: CsgOperation,
}

#[pymethods]
impl Brush {
    #[new]
    fn new(polys: Vec<Py<Poly>>, poly_flags: usize, csg_operation: CsgOperation) -> Self {
        Brush { polys, poly_flags, csg_operation: CsgOperation::Add }
    }
}

use crate::fpoly::FPoly;

impl From<Poly> for FPoly {
    fn from(poly: Poly) -> Self {
        let vertices: Vec<math::FVector> = poly.vertices.iter().map(|(x, y, z)| math::FVector::new(*x, *y, *z)).collect();
        FPoly::from_vertices(&vertices)
    }
}

/// Formats the sum of two numbers as string.
#[pyfunction]
fn csg_rebuild(brushes: Vec<Py<Brush>>) -> PyResult<()> {
    println!("CSG rebuild with {} brushes", brushes.len());
    Ok(())
}

#[pymodule]
fn bdk_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Poly>()?;
    m.add_class::<Brush>()?;
    m.add_function(wrap_pyfunction!(csg_rebuild, m)?)?;
    Ok(())
}