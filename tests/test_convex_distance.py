import itertools

import numpy as np
import pytest

from backend.world_model.convex_distance import convex_distance_bounds


def box(lower, upper):
    return np.asarray(list(itertools.product(*zip(lower, upper))), dtype=float)


@pytest.mark.parametrize("offset", [[3, 0, 0], [3, 4, 0], [3, 4, 5], [2.00001, 0, 0]])
def test_separated_box_distance_bounds_enclose_exact_distance(offset):
    first = box([-1]*3, [1]*3)
    second = first + offset
    expected = np.linalg.norm(np.maximum(np.abs(offset) - 2, 0))
    result = convex_distance_bounds(first, second)
    assert result["status"] == "separated_hulls"
    assert result["distance_lower_bound_m"] <= expected + 1e-9
    assert result["distance_upper_bound_m"] >= expected - 1e-9
    assert result["distance_upper_bound_m"] - result["distance_lower_bound_m"] < 1e-6
    assert result["margin_separation_certified"] is bool(expected > 0.01)


@pytest.mark.parametrize("offset", [[0, 0, 0], [1, 1, 1], [2, 0, 0]])
def test_overlapping_and_touching_boxes_are_not_certified_separate(offset):
    first = box([-1]*3, [1]*3)
    result = convex_distance_bounds(first, first + offset)
    assert result["status"] == "overlap_or_within_tolerance"
    assert result["distance_upper_bound_m"] <= 1e-7
    assert result["margin_separation_certified"] is False


def test_translations_rotations_and_swapping_preserve_distance_bounds():
    angle = 0.63
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    first, second = box([-1]*3, [1]*3), box([3, 3, 3], [4, 4, 4])
    expected = np.sqrt(12)
    first, second = [points @ rotation.T + [71, 140, 1] for points in (first, second)]
    for a, b in ((first, second), (second, first)):
        result = convex_distance_bounds(a, b)
        assert result["distance_lower_bound_m"] == pytest.approx(expected, abs=1e-6)
        assert result["distance_upper_bound_m"] == pytest.approx(expected, abs=1e-6)


def test_degenerate_points_lines_and_planes_are_supported_without_false_clear():
    assert convex_distance_bounds([[0, 0, 0]], [[3, 4, 0]])["distance_upper_bound_m"] == 5
    crossing = convex_distance_bounds([[-1, 0, 0], [1, 0, 0]], [[0, -1, 0], [0, 1, 0]])
    assert crossing["status"] == "overlap_or_within_tolerance"
    first = box([-1, -1, 0], [1, 1, 0])
    assert convex_distance_bounds(first, first + [0, 0, 1])["distance_lower_bound_m"] == pytest.approx(1)


def test_iteration_budget_only_returns_a_certified_projection_when_available():
    first = box([-1]*3, [1]*3)
    result = convex_distance_bounds(first, first + [1.5, 1.5, 1.5], max_iterations=1)
    assert result["status"] == "indeterminate"
    assert not result["margin_separation_certified"]


@pytest.mark.parametrize("first", [[], [[float("nan"), 0, 0]], [[1, 2]], [[1e7, 0, 0]]])
def test_invalid_point_clouds_are_rejected(first):
    with pytest.raises(ValueError): convex_distance_bounds(first, [[0, 0, 0]])


def test_random_hulls_agree_with_independent_linear_feasibility_oracle():
    optimize = pytest.importorskip("scipy.optimize")
    rng = np.random.default_rng(710)
    for offset in [0, 0.5, 1, 2, 3, 6]:
        first, second = rng.normal(size=(12, 3)), rng.normal(size=(10, 3)) + [offset, 0, 0]
        constraints = np.vstack([np.column_stack([first.T, -second.T]),
                                 np.r_[np.ones(12), np.zeros(10)], np.r_[np.zeros(12), np.ones(10)]])
        reference = optimize.linprog(np.zeros(22), A_eq=constraints, b_eq=[0, 0, 0, 1, 1], bounds=(0, None), method="highs")
        result = convex_distance_bounds(first, second)
        assert result["status"] == ("overlap_or_within_tolerance" if reference.success else "separated_hulls")
