import pytest

from backend.world_model.fanuc_calibration_review import review_calibration_candidate
from test_fanuc_nodes import SAMPLE_LS


CANDIDATE = """ROBOT_NAME,m710ic_20l_HFS1_R01
UF,1,1875,6978,-836,0,0,-90
UT,1,0,0,335,0,0,0
POSE,start,90,-38,16,0,-106,0
"""


def test_matching_ids_are_candidates_without_calibration_or_execution_approval():
    result = review_calibration_candidate(SAMPLE_LS, CANDIDATE)
    assert result["status"] == "requires_coordinate_alignment"
    assert result["required_frames"] == {"UF": ["1"], "UT": ["1"]}
    assert result["cartesian_references"] == [{"line": 2, "position_id": 2, "uf": 1, "ut": 1}]
    assert result["issues"] == [] and result["pending_evidence"]
    assert not any(result[key] for key in ("transforms_applied", "calibration_verified", "execution_ready"))


@pytest.mark.parametrize("change,code", [
    (lambda s: s.replace("m710ic_20l_HFS1_R01", "m20ia_HFS1_R07"), "robot_model_mismatch"),
    (lambda s: s.replace("UF,1,", "UF,2,"), "missing_frame_definitions"),
    (lambda s: s + "UF,01,0,0,0,0,0,0\n", "duplicate_record"),
    (lambda s: s.replace("UT,1,0,0,335", "UT,1,0,0,nan"), "invalid_record"),
    (lambda s: s.replace("POSE,start,90", "POSE,start,190"), "joint_seed_out_of_limits"),
    (lambda s: s.replace("ROBOT_NAME,m710ic_20l_HFS1_R01\n", ""), "robot_model_missing"),
    (lambda s: s + 'UT,"unterminated\n', "malformed_csv"),
])
def test_bad_candidates_are_rejected(change, code):
    result = review_calibration_candidate(SAMPLE_LS, change(CANDIDATE))
    assert result["status"] == "rejected"
    assert code in {issue["code"] for issue in result["issues"]}


def test_active_selection_fallback_and_zero_frame_require_explicit_values():
    source = SAMPLE_LS.replace("UF : 1, UT : 1,", "").replace("UFRAME_NUM=1", "UFRAME_NUM=0")
    result = review_calibration_candidate(source, CANDIDATE)
    assert result["missing_frames"]["UF"] == ["0"]
    assert result["cartesian_references"][0]["ut"] == 1


def test_unknown_selection_and_absent_motion_fail_and_digests_change():
    result = review_calibration_candidate(SAMPLE_LS.replace("UF : 1, UT : 1,", "").replace("UFRAME_NUM=1", "! no frame"), CANDIDATE)
    assert any(issue["code"] == "unknown_frame_selection" for issue in result["issues"])
    empty = review_calibration_candidate("/MN\n/POS", CANDIDATE)
    assert any(issue["code"] == "no_supported_motion_frames" for issue in empty["issues"])
    assert result["source_sha256"] != empty["source_sha256"]
    assert result["candidate_sha256"] == empty["candidate_sha256"]
