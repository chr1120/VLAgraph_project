import copy

import pytest

pytest.importorskip("pxr")
from pxr import Gf, UsdGeom

from backend.world_model.fanuc_hull_sweep import inspect_review_hull_sweep, _motion_bound, _motion_levers
from backend.world_model.fanuc_collision import check_fanuc_collision
from backend.world_model.fanuc_geometry_review import build_geometry_review
from backend.world_model.fanuc_preflight import action_fingerprint
from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy
from test_fanuc_hull_inspection import fixture
from test_fanuc_usd_proxy import stage_fixture, inventory
from test_fanuc_preflight import action_fixture


def test_static_selected_pairs_certify_intervals_without_exclusion_or_execution():
    stage, data, scene, action, collision = fixture()
    before = stage.GetRootLayer().ExportToString()
    original = copy.deepcopy(scene)
    result = inspect_review_hull_sweep(stage, data, scene, action, collision, max_step_sec=0.5)
    assert result["status"] == "separated_over_interval"
    assert result["sample_count"] == 5 and result["all_selected_pairs_certified"] is True
    assert all(pair["between_samples_certified"] for pair in result["pairs"])
    assert result["exclusion_policy"] == "unchanged" and result["execution_ready"] is False
    assert result["physical_collision_verified"] is False
    assert stage.GetRootLayer().ExportToString() == before and scene == original
    plan = build_geometry_review(data, scene, collision_report=collision, hull_sweep=result)
    assert all(pair["decision"] == "keep" and not pair["reviewed"] for pair in plan["pair_decisions"])
    assert any(pair["hull_sweep"] for pair in plan["pair_decisions"])


def moving_fixture():
    stage = stage_fixture()
    shape = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Robot/link1/shape"))
    shape.ClearXformOpOrder()
    shape.AddTranslateOp().Set(Gf.Vec3d(0.15, 0, 1.15))
    data = inventory(stage)
    scene = build_fanuc_usd_proxy(data)["scene"]
    for capsule in scene["link_capsules"]: capsule["radius_m"] = 2
    action = action_fixture()
    action["joint_trajectory"][0]["joint_positions"][1] = -90
    action["joint_trajectory"][1]["joint_positions"] = [0, 90, 0, 0, 0, 0]
    collision = check_fanuc_collision(action, scene)
    return stage, data, scene, action, collision


def target_pair(result):
    return next(pair for pair in result["pairs"] if set(pair["pair"]) == {"/World/Robot/link1/shape", "/World/Robot/link3/shape"})


def test_clear_endpoints_cannot_hide_middle_motion_in_coarse_sweep():
    args = moving_fixture()
    coarse = target_pair(inspect_review_hull_sweep(*args, max_step_sec=2))
    assert coarse["margin_violation_samples"] == 0
    assert coarse["status"] == "indeterminate" and coarse["unresolved_interval_count"] == 1
    assert coarse["between_samples_certified"] is False
    fine = target_pair(inspect_review_hull_sweep(*args, max_step_sec=0.5))
    assert fine["status"] == "margin_violation"
    assert fine["min_upper_bound_time_sec"] == 3
    assert fine["margin_violation_samples"] >= 1


def test_snapshot_adds_first_segment_and_its_sample_budget():
    from test_fanuc_initial_state import snapshot_fixture
    stage, data, scene, action, _ = fixture()
    snapshot = snapshot_fixture()
    snapshot["prim_path"] = snapshot["raw_state"]["prim_path"] = data["robot_prim_path"]
    snapshot["geometry"].update(stage_identifier=data["stage_identifier"])
    collision = check_fanuc_collision(action, scene, snapshot=snapshot)
    result = inspect_review_hull_sweep(stage, data, scene, action, collision, snapshot=snapshot, max_step_sec=1)
    assert result["checked_time_range_sec"] == [0, 4]
    assert result["initial_segment_checked"] is True and result["sample_count"] == 5
    with pytest.raises(ValueError, match="budget"):
        inspect_review_hull_sweep(stage, data, scene, action, collision, snapshot=snapshot, max_step_sec=1, max_samples=4)


def test_original_segment_boundaries_are_sampled_even_when_step_is_coarse():
    stage, data, scene, action, _ = fixture()
    middle = copy.deepcopy(action["joint_trajectory"][0])
    middle["time_sec"] = 2.25
    action["joint_trajectory"].insert(1, middle)
    collision = check_fanuc_collision(action, scene)
    result = inspect_review_hull_sweep(stage, data, scene, action, collision, max_step_sec=2)
    assert result["sample_count"] == 3 and result["all_selected_pairs_certified"] is True


def test_world_base_and_joint_motion_bounds_use_correct_attachment():
    left, right = {"J1": 0, "externalAxisJoint": 0}, {"J1": 0.1, "externalAxisJoint": 0.25}
    assert _motion_bound({"attachment": "world"}, [], left, right) == 0
    assert _motion_bound({"attachment": "base"}, [], left, right) == 0.25
    assert _motion_bound({"attachment": "base"}, [], left, right, rail_scale=1.0000005) == pytest.approx(0.250000125)
    assert _motion_bound({"attachment": "joint"}, [2], left, right) == pytest.approx(0.45)
    assert _motion_levers([[1, 0, 0], [0, 2, 0]], {"attachment": "joint", "joint_index": 0}) == [2]


def test_unsupported_shapes_and_no_selected_pairs_never_certify_whole_scene():
    result = inspect_review_hull_sweep(*fixture(sphere=True), max_step_sec=1)
    unsupported = [pair for pair in result["pairs"] if pair["unsupported_geometry"]]
    assert unsupported and all(pair["status"] == "indeterminate" for pair in unsupported)
    stage, data, scene, action, collision = fixture()
    for diagnostic in collision["pair_diagnostics"]: diagnostic["margin_violation_samples"] = 0
    result = inspect_review_hull_sweep(stage, data, scene, action, collision)
    assert result["status"] == "not_applicable" and result["all_selected_pairs_certified"] is False


@pytest.mark.parametrize("step,samples", [(0, 100), (float("nan"), 100), (True, 100), (0.05, 2), (0.05, True), (0.001, 10000)])
def test_invalid_settings_and_exceeded_budgets_are_rejected(step, samples):
    with pytest.raises(ValueError): inspect_review_hull_sweep(*fixture(), max_step_sec=step, max_samples=samples)


def test_review_rejects_changed_sweep_evidence_even_with_other_identifiers_unchanged():
    args = fixture()
    _, data, scene, _, collision = args
    result = inspect_review_hull_sweep(*args, max_step_sec=1)
    result["pairs"][0]["status"] = "margin_violation"
    with pytest.raises(ValueError, match="must match"):
        build_geometry_review(data, scene, collision_report=collision, hull_sweep=result)
    result["sweep_digest"] = action_fingerprint({key: value for key, value in result.items() if key != "sweep_digest"})
    result["action_digest"] = "other"
    with pytest.raises(ValueError, match="must match"):
        build_geometry_review(data, scene, collision_report=collision, hull_sweep=result)


def test_selected_pair_sweep_is_not_an_execution_collision_report():
    from backend.executors.world_model_nodes import exec_execution_guard
    from backend.world_model.fanuc_preflight import validate_fanuc_trajectory

    stage, data, scene, action, _ = fixture()
    action.update(preview_only=False, execution_enabled=True, execution_ready=True)
    collision = check_fanuc_collision(action, scene)
    sweep = inspect_review_hull_sweep(stage, data, scene, action, collision, max_step_sec=1)
    assert sweep["all_selected_pairs_certified"] is True
    context = {"a": {"action": action}, "p": {"report": validate_fanuc_trajectory(action)}, "c": {"collision_report": sweep}}
    edges = [{"source": source, "sourceHandle": handle, "target": "g", "targetHandle": handle}
             for source, handle in [("a", "action"), ("p", "report"), ("c", "collision_report")]]
    assert exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation"}}, context, edges)["approved"] is False
