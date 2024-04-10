use bdk_py::{self, sphere};
use bdk_py::math::FVector;

use sphere::FSphere;

#[test]
fn sphere_new_test() {
    let sphere = FSphere::new();
    assert_eq!(sphere.origin, FVector::new(0.0, 0.0, 0.0));
    assert_eq!(sphere.radius, 0.0);
}

#[test]
fn sphere_new_from_origin_and_radius_test() {
    let origin = FVector::new(1.0, 2.0, 3.0);
    let radius = 4.0;
    let sphere = FSphere::new_from_origin_and_radius(&origin, radius);
    assert_eq!(sphere.origin, origin);
    assert_eq!(sphere.radius, radius);
}

#[test]
fn sphere_new_from_points_test() {
    let points = vec![
        FVector::new(1.0, 2.0, 3.0),
        FVector::new(4.0, 5.0, 6.0),
        FVector::new(7.0, 8.0, 9.0)
    ];
    let sphere = FSphere::new_from_points(&points);
    assert_eq!(sphere.origin, FVector::new(4.0, 5.0, 6.0));
    // (4,5,6) -> (7,8,9) = sqrt(3^2 + 3^2 + 3^2) = sqrt(27) = 5.196152 * 1.001 = 5.201349
    assert_eq!(sphere.radius, 5.201349);
}