"""Fit a proposed controller-base to IK-base transform from paired TCP poses.

Numerical consistency alone does not verify TCP calibration or robot identity.
"""
from __future__ import annotations

import numpy as np


def _rigid(value):
    if not isinstance(value, list) or len(value) != 4 or any(not isinstance(row, list) or len(row) != 4 for row in value):
        raise ValueError("Reference pose must be a 4x4 matrix")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for row in value for v in row):
        raise ValueError("Reference pose entries must be finite numbers")
    matrix = np.asarray(value, dtype=float)
    rotation = matrix[:3, :3]
    if (not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("Reference pose must be rigid, right-handed, and unscaled")
    return matrix


def fit_base_alignment(evidence, *, source_sha256, candidate_sha256):
    if not isinstance(evidence, dict) or evidence.get("schema") != "vlagraph.fanuc_alignment_references.v1":
        raise ValueError("Unsupported alignment reference schema")
    if evidence.get("units") != "m":
        raise ValueError("Reference translations must explicitly use meters")
    if evidence.get("source_sha256") != source_sha256 or evidence.get("candidate_sha256") != candidate_sha256:
        raise ValueError("Reference evidence must match the reviewed program and TOOLDATA digests")
    references = evidence.get("references")
    if not isinstance(references, list) or not 3 <= len(references) <= 1000:
        raise ValueError("Require 3..1000 paired TCP reference poses")
    if any(not isinstance(ref, dict) for ref in references):
        raise ValueError("Each reference must be an object")
    source = np.asarray([_rigid(ref.get("controller_tcp_in_base")) for ref in references])
    target = np.asarray([_rigid(ref.get("ik_tcp_in_base")) for ref in references])
    x, y = source[:, :3, 3], target[:, :3, 3]
    x_mean, y_mean = x.mean(axis=0), y.mean(axis=0)
    centered_x, centered_y = x - x_mean, y - y_mean
    # Collinear or nearly coincident measurements cannot establish a stable base rotation.
    for points in (centered_x, centered_y):
        singular = np.linalg.svd(points, compute_uv=False)
        if singular[1] < max(1e-4, singular[0] * 1e-4):
            raise ValueError("Reference positions must be sufficiently separated and non-collinear")
    u, _, vt = np.linalg.svd(centered_x.T @ centered_y)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, y_mean - rotation @ x_mean
    predicted = transform @ source
    position_errors = np.linalg.norm(predicted[:, :3, 3] - y, axis=1)
    orientation_errors = []
    for actual, expected in zip(predicted, target):
        cosine = (np.trace(actual[:3, :3].T @ expected[:3, :3]) - 1) / 2
        orientation_errors.append(float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))
    consistent = float(max(position_errors)) <= 0.001 and max(orientation_errors) <= 1.0
    return {"schema": "vlagraph.fanuc_base_alignment_fit.v1",
            "status": "reference_residuals_passed" if consistent else "reference_residuals_failed",
            "source_sha256": source_sha256, "candidate_sha256": candidate_sha256,
            "controller_base_to_ik_base": transform.tolist(), "reference_count": len(references),
            "position_errors_m": position_errors.tolist(), "orientation_errors_deg": orientation_errors,
            "max_position_error_m": float(max(position_errors)), "max_orientation_error_deg": max(orientation_errors),
            "limits": {"position_m": 0.001, "orientation_deg": 1.0},
            "limitations": ["Fitted and evaluated on the same references; independent validation is required",
                            "Robot identity, measured provenance, TCP and external-axis calibration remain unverified"],
            "transforms_applied": False, "calibration_verified": False, "execution_ready": False}
