import copy

import numpy as np
import pytest

from backend.world_model.fanuc_alignment import fit_base_alignment


def evidence_fixture():
    transform = np.array([[0., -1, 0, 4], [1, 0, 0, -2], [0, 0, 1, .5], [0, 0, 0, 1]])
    references = []
    for point in ([0, 0, 0], [1, 0, 0], [0, 1, 0], [.3, .4, .6]):
        pose = np.eye(4)
        pose[:3, 3] = point
        references.append({"controller_tcp_in_base": pose.tolist(), "ik_tcp_in_base": (transform @ pose).tolist()})
    return {"schema": "vlagraph.fanuc_alignment_references.v1", "units": "m", "source_sha256": "source",
            "candidate_sha256": "candidate", "references": references}, transform


def fit(evidence):
    return fit_base_alignment(evidence, source_sha256="source", candidate_sha256="candidate")


def test_recovers_known_rigid_transform_without_granting_calibration():
    evidence, transform = evidence_fixture()
    original = copy.deepcopy(evidence)
    result = fit(evidence)
    assert result["status"] == "reference_residuals_passed"
    np.testing.assert_allclose(result["controller_base_to_ik_base"], transform, atol=1e-12)
    assert result["max_position_error_m"] < 1e-12
    assert not result["calibration_verified"] and not result["transforms_applied"] and not result["execution_ready"]
    assert evidence == original


@pytest.mark.parametrize("case", ["units", "digest", "missing", "collinear", "reflection", "scale", "nan", "boolean"])
def test_invalid_or_unbound_measurements_fail(case):
    evidence, _ = evidence_fixture()
    if case == "units": evidence["units"] = "mm"
    if case == "digest": evidence["candidate_sha256"] = "stale"
    if case == "missing": evidence["references"] = evidence["references"][:2]
    if case == "collinear":
        for index, ref in enumerate(evidence["references"]): ref["controller_tcp_in_base"][0][3], ref["controller_tcp_in_base"][1][3], ref["controller_tcp_in_base"][2][3] = index, 0, 0
    if case == "reflection": evidence["references"][0]["controller_tcp_in_base"][0][0] = -1
    if case == "scale": evidence["references"][0]["controller_tcp_in_base"][0][0] = 2
    if case == "nan": evidence["references"][0]["controller_tcp_in_base"][0][3] = float("nan")
    if case == "boolean": evidence["references"][0]["controller_tcp_in_base"][0][3] = True
    with pytest.raises(ValueError): fit(evidence)


@pytest.mark.parametrize("case", ["position", "orientation"])
def test_reference_residual_failure_is_reported(case):
    evidence, _ = evidence_fixture()
    if case == "position": evidence["references"][0]["ik_tcp_in_base"][0][3] += .02
    else:
        rotation = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        pose = np.array(evidence["references"][0]["ik_tcp_in_base"])
        pose[:3, :3] = pose[:3, :3] @ rotation
        evidence["references"][0]["ik_tcp_in_base"] = pose.tolist()
    assert fit(evidence)["status"] == "reference_residuals_failed"
