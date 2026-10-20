import copy

import pytest

pytest.importorskip("pxr")
from pxr import UsdGeom

from backend.world_model.fanuc_hull_inspection import inspect_review_hulls, _positions_at
from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy
from backend.world_model.fanuc_collision import check_fanuc_collision
from backend.world_model.fanuc_preflight import action_fingerprint
from backend.world_model.fanuc_geometry_review import build_geometry_review
from test_fanuc_usd_proxy import stage_fixture, inventory
from test_fanuc_preflight import action_fixture


def fixture(*, units=1, up="Z", sphere=False):
    stage = stage_fixture(units=units, up=up)
    if sphere:
        UsdGeom.Sphere.Define(stage, "/World/Robot/link1/shape")
    data = inventory(stage)
    scene = build_fanuc_usd_proxy(data)["scene"]
    # Deliberately coarse synthetic capsules to exercise refinement diagnostics.
    for capsule in scene["link_capsules"]: capsule["radius_m"] = 2
    action = action_fixture()
    for frame in action["joint_trajectory"]: frame["joint_positions"] = [0]*6
    collision = check_fanuc_collision(action, scene)
    return stage, data, scene, action, collision


def test_refinement_is_read_only_and_preserves_all_review_decisions():
    args = fixture()
    stage, data, scene, action, collision = args
    before = stage.GetRootLayer().ExportToString()
    original = copy.deepcopy(scene)
    result = inspect_review_hulls(*args)
    assert result["pairs"]
    assert any(pair["status"] == "separated_hulls" for pair in result["pairs"])
    assert all(pair["time_sec"] == 2 for pair in result["pairs"])
    assert result["exclusion_policy"] == "unchanged"
    assert result["execution_ready"] is False and result["physical_collision_verified"] is False
    assert stage.GetRootLayer().ExportToString() == before and scene == original
    plan = build_geometry_review(data, scene, collision_report=collision, hull_inspection=result)
    assert all(pair["decision"] == "keep" and not pair["reviewed"] for pair in plan["pair_decisions"])
    assert any(pair["hull_inspection"] for pair in plan["pair_decisions"])


def test_meter_zup_and_millimeter_yup_give_same_pair_distance():
    first = inspect_review_hulls(*fixture())
    second = inspect_review_hulls(*fixture(units=0.001, up="Y"))
    assert len(first["pairs"]) == len(second["pairs"])
    for a, b in zip(first["pairs"], second["pairs"]):
        assert a["pair"] == b["pair"] and a["status"] == b["status"]
        assert a["distance_lower_bound_m"] == pytest.approx(b["distance_lower_bound_m"], abs=1e-6)
        assert a["distance_upper_bound_m"] == pytest.approx(b["distance_upper_bound_m"], abs=1e-6)


@pytest.mark.parametrize("change", ["stage", "action", "snapshot_ref", "scene"])
def test_stale_evidence_is_rejected(change):
    args = list(fixture())
    stage, data, scene, action, collision = args
    if change == "stage": UsdGeom.Cube(stage.GetPrimAtPath("/World/Robot/link1/shape")).CreateSizeAttr(0.2)
    if change == "action": action["joint_trajectory"][1]["joint_positions"][0] = 1
    if change == "snapshot_ref": collision["snapshot_ref"] = {"episode_id": "other"}
    if change == "scene": scene["padding_m"] = 0.02
    with pytest.raises(ValueError): inspect_review_hulls(*args)


def test_unsupported_shape_is_indeterminate_without_fallback_to_bounds():
    result = inspect_review_hulls(*fixture(sphere=True))
    unsupported = [pair for pair in result["pairs"] if any("link1/shape" in path for path in pair["pair"])]
    assert unsupported and all(pair["status"] == "indeterminate" for pair in unsupported)
    assert all(pair["margin_separation_certified"] is False for pair in unsupported)


def test_tampered_hull_evidence_is_rejected_by_review_generation():
    args = fixture()
    _, data, scene, _, collision = args
    result = inspect_review_hulls(*args)
    result["pairs"][0]["status"] = "separated_hulls"
    result["pairs"][0]["distance_lower_bound_m"] = 999
    with pytest.raises(ValueError, match="must match"):
        build_geometry_review(data, scene, collision_report=collision, hull_inspection=result)


def test_pose_lookup_uses_smoothstep_and_never_extrapolates():
    frames = [{"time_sec": 0, "positions": {"J1": -1}}, {"time_sec": 2, "positions": {"J1": 1}}]
    assert _positions_at(frames, 0.5)["J1"] == pytest.approx(-0.6875)
    with pytest.raises(ValueError): _positions_at(frames, -0.1)
    with pytest.raises(ValueError): _positions_at(frames, 3)


def test_different_inspection_time_cannot_be_attached_even_with_updated_checksum():
    args = fixture()
    _, data, scene, _, collision = args
    result = inspect_review_hulls(*args)
    result["pairs"][0]["time_sec"] += 0.5
    result["inspection_digest"] = action_fingerprint({key: value for key, value in result.items() if key != "inspection_digest"})
    with pytest.raises(ValueError, match="recorded pair diagnostic pose"):
        build_geometry_review(data, scene, collision_report=collision, hull_inspection=result)


def test_snapshot_pose_can_be_inspected_without_treating_it_as_live_physics():
    from test_fanuc_initial_state import snapshot_fixture
    stage, data, scene, action, _ = fixture()
    snapshot = snapshot_fixture()
    snapshot["prim_path"] = snapshot["raw_state"]["prim_path"] = data["robot_prim_path"]
    snapshot["geometry"].update(stage_identifier=data["stage_identifier"], meters_per_unit=data["meters_per_unit"], up_axis=data["up_axis"])
    collision = check_fanuc_collision(action, scene, snapshot=snapshot)
    result = inspect_review_hulls(stage, data, scene, action, collision, snapshot=snapshot)
    assert result["pairs"] and all(pair["time_sec"] == 0 for pair in result["pairs"])
    assert result["physical_collision_verified"] is False
