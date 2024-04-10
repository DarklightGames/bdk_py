use bdk_py::box_;
use bdk_py::math::FVector;

use box_::FBox;

#[test]
fn box_new_test() {
    let box_ = FBox::new();
    assert_eq!(box_.min, FVector::new(0.0, 0.0, 0.0));
    assert_eq!(box_.max, FVector::new(0.0, 0.0, 0.0));
    assert_eq!(box_.is_valid, false);
}

#[test]
fn box_new_from_points_test() {
    let points = vec![
        FVector::new(1.0, 2.0, 3.0),
        FVector::new(4.0, 5.0, 6.0),
        FVector::new(7.0, 8.0, 9.0)
    ];
    let box_ = FBox::new_from_points(&points);
    assert_eq!(box_.min, FVector::new(1.0, 2.0, 3.0));
    assert_eq!(box_.max, FVector::new(7.0, 8.0, 9.0));
    assert_eq!(box_.is_valid, true);
}

#[test]
fn box_new_from_min_max_test() {
    let min = FVector::new(1.0, 2.0, 3.0);
    let max = FVector::new(4.0, 5.0, 6.0);
    let box_ = FBox::new_from_min_max(min, max);
    assert_eq!(box_.min, min);
    assert_eq!(box_.max, max);
    assert_eq!(box_.is_valid, true);
}

#[test]
fn box_center_test() {
    let box_ = FBox::new_from_min_max(FVector::new(1.0, 2.0, 3.0), FVector::new(4.0, 5.0, 6.0));
    assert_eq!(box_.center(), FVector::new(2.5, 3.5, 4.5));
}

#[test]
fn box_extent_test() {
    let box_ = FBox::new_from_min_max(FVector::new(1.0, 2.0, 3.0), FVector::new(4.0, 5.0, 6.0));
    assert_eq!(box_.extent(), FVector::new(1.5, 1.5, 1.5));
}

#[test]
fn box_expand_by_test() {
    let box_ = FBox::new_from_min_max(FVector::new(1.0, 2.0, 3.0), FVector::new(4.0, 5.0, 6.0));
    let expanded_box = box_.expand_by(1.0);
    assert_eq!(expanded_box.min, FVector::new(0.0, 1.0, 2.0));
    assert_eq!(expanded_box.max, FVector::new(5.0, 6.0, 7.0));
    assert_eq!(expanded_box.is_valid, true);
}

#[test]
fn box_index_test() {
    let box_ = FBox::new_from_min_max(FVector::new(1.0, 2.0, 3.0), FVector::new(4.0, 5.0, 6.0));
    assert_eq!(box_[0], FVector::new(1.0, 2.0, 3.0));
    assert_eq!(box_[1], FVector::new(4.0, 5.0, 6.0));
}

#[test]
fn box_intersect_disjoint_test() {
    let box1 = FBox::new_from_min_max(FVector::new(1.0, 2.0, 3.0), FVector::new(4.0, 5.0, 6.0));
    let box2 = FBox::new_from_min_max(FVector::new(5.0, 6.0, 7.0), FVector::new(8.0, 9.0, 10.0));
    assert_eq!(box1.intersect(&box2), false);
}

#[test]
fn box_intersect_contained_test() {
    let box1 = FBox::new_from_min_max(FVector::new(1.0, 2.0, 3.0), FVector::new(4.0, 5.0, 6.0));
    let box2 = FBox::new_from_min_max(FVector::new(2.0, 3.0, 4.0), FVector::new(3.0, 4.0, 5.0));
    assert_eq!(box1.intersect(&box2), true);
}

#[test]
fn box_intersect_overlap_test() {
    let box1 = FBox::new_from_min_max(FVector::new(0.0, 0.0, 0.0), FVector::new(1.0, 1.0, 1.0));
    let box2 = FBox::new_from_min_max(FVector::new(0.5, 0.5, 0.5), FVector::new(1.5, 1.5, 1.5));
    assert_eq!(box1.intersect(&box2), true);
}