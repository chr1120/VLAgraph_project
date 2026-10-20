"""Bound the distance between convex hulls of finite 3D point clouds.

Support projections certify a lower bound; a closest simplex supplies an
upper bound. Exhausted/degenerate searches never certify an intersection.
"""
from __future__ import annotations

import itertools
import math

import numpy as np


def _closest_simplex(points):
    best = None
    for count in range(1, min(4, len(points)) + 1):
        for indices in itertools.combinations(range(len(points)), count):
            subset = np.asarray([points[index] for index in indices])
            if count == 1:
                weights = np.ones(1)
            else:
                coefficients = np.linalg.lstsq((subset[1:] - subset[0]).T, -subset[0], rcond=None)[0]
                weights = np.r_[1 - coefficients.sum(), coefficients]
                if np.min(weights) < -1e-12:
                    continue
                weights = np.maximum(weights, 0)
                weights /= weights.sum()
            closest = weights @ subset
            squared = float(closest @ closest)
            if best is None or squared < best[0]:
                best = (squared, closest, [point for point, weight in zip(subset, weights) if weight > 1e-14])
    return best[1], best[2]


def convex_distance_bounds(first, second, *, margin_m=0.01, tolerance_m=1e-7, max_iterations=128):
    arrays = []
    for values in (first, second):
        points = np.asarray(values, dtype=float)
        if (points.ndim != 2 or points.shape[1:] != (3,) or not 1 <= len(points) <= 20000
                or not np.all(np.isfinite(points)) or np.max(np.abs(points)) > 1e6):
            raise ValueError("Require 1..20000 finite 3D points within supported coordinate range")
        arrays.append(points)
    if (isinstance(margin_m, bool) or not isinstance(margin_m, (int, float)) or not math.isfinite(margin_m)
            or not 0 <= margin_m <= 1 or isinstance(tolerance_m, bool)
            or not isinstance(tolerance_m, (int, float)) or not math.isfinite(tolerance_m) or not 1e-10 <= tolerance_m <= 1e-4):
        raise ValueError("Invalid distance margin or tolerance")
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or not 1 <= max_iterations <= 256:
        raise ValueError("max_iterations must be 1..256")
    origin = arrays[0].mean(axis=0)
    first, second = [points - origin for points in arrays]
    numerical_guard = float(np.finfo(float).eps * max(1, *(np.max(np.abs(points)) for points in arrays)) * 128)
    epsilon = tolerance_m + numerical_guard

    def support(direction):
        return first[np.argmax(first @ direction)] - second[np.argmin(second @ direction)]

    simplex = [support(np.array([1.0, 0, 0]))]
    lower, upper, axis, reason = 0.0, None, None, "iteration_budget"
    for iteration in range(1, max_iterations + 1):
        closest, simplex = _closest_simplex(simplex)
        upper = float(np.linalg.norm(closest))
        if upper <= epsilon:
            reason = "simplex_within_tolerance"
            break
        direction = closest / upper
        point = support(-direction)
        projected = max(0.0, float(point @ direction) - numerical_guard)
        if projected > lower:
            lower, axis = projected, direction.tolist()
        if upper - lower <= epsilon:
            reason = "bounds_converged"
            break
        if any(float(np.linalg.norm(point - prior)) <= epsilon for prior in simplex):
            reason = "repeated_support"
            break
        simplex.append(point)
    status = ("overlap_or_within_tolerance" if upper <= epsilon else "separated_hulls" if lower > epsilon else "indeterminate")
    return {"status": status, "distance_lower_bound_m": lower, "distance_upper_bound_m": upper,
            "requested_margin_m": margin_m, "margin_separation_certified": lower > margin_m + epsilon,
            "separating_axis_world": axis, "iterations": iteration, "termination": reason,
            "tolerance_m": tolerance_m, "numerical_guard_m": numerical_guard}
