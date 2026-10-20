import copy
import itertools

import pytest

pytest.importorskip("pxr")
from pxr import UsdGeom, UsdPhysics

from backend.executors.world_model_nodes import exec_fanuc_usd_proxy_build, exec_execution_guard
from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy
from backend.world_model.fanuc_geometry_review import build_geometry_review, apply_geometry_review
from backend.world_model.fanuc_collision import check_fanuc_collision
from backend.world_model.fanuc_preflight import validate_fanuc_trajectory
from test_fanuc_usd_proxy import inventory, stage_fixture
from test_fanuc_preflight import action_fixture
from test_fanuc_collision import scene_fixture


def source():
    data = inventory()
    return data, build_fanuc_usd_proxy(data)["scene"]


def test_default_plan_never_excludes_adjacency_or_same_body_automatically():
    stage = stage_fixture()
    shape = UsdGeom.Cube.Define(stage, "/World/Robot/link1/extra_shape")
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    data = inventory(stage)
    scene = build_fanuc_usd_proxy(data)["scene"]
    plan = build_geometry_review(data, scene)
    assert len(plan["pair_decisions"]) == len(list(itertools.combinations(scene["link_capsules"], 2)))
    assert all(item["decision"] == "keep" and item["reviewed"] is False for item in plan["pair_decisions"])
    assert any(item["usd_relation"] == "same_rigid_body" and item["inspection_priority"] == "inspect" for item in plan["pair_decisions"])
    assert any(item["usd_relation"] == "adjacent_revolute_joint" for item in plan["pair_decisions"])
    assert plan["geometry_expectations"]["tool"]["declared"] is False


def test_pair_diagnostics_cover_all_samples_after_contact_log_cap():
    action = action_fixture()
    for frame in action["joint_trajectory"]: frame["joint_positions"][0] = 0
    report = check_fanuc_collision(action, scene_fixture(), max_step_sec=0.01)
    assert len(report["contacts"]) == report["contacts_limit"] == 100
    assert report["samples_checked"] == 201
    detail = report["pair_diagnostics"][0]
    assert detail["margin_violation_samples"] == detail["samples_checked"] == 201
    assert detail["min_sampled_clearance_m"] == pytest.approx(-0.05)


def test_review_includes_matching_diagnostics_but_rejects_other_scene():
    data, scene = source()
    report = check_fanuc_collision(action_fixture(), scene)
    plan = build_geometry_review(data, scene, collision_report=report)
    assert any(item["sampled_diagnostic"] for item in plan["pair_decisions"])
    report["scene_digest"] = "stale"
    with pytest.raises(ValueError, match="does not match"):
        build_geometry_review(data, scene, collision_report=report)


def test_explicit_exclusion_is_applied_with_audit_and_no_physical_permission():
    data, scene = source()
    original = copy.deepcopy(scene)
    plan = build_geometry_review(data, scene)
    decision = plan["pair_decisions"][0]
    decision.update(decision="exclude", reviewed=True, reviewed_by="fixture-reviewer", rationale="Synthetic test decision")
    result = apply_geometry_review(data, scene, plan)
    assert result["excluded_self_pairs"] == [decision["pair"]]
    assert result["geometry_review"]["excluded_pair_count"] == 1
    assert result["geometry_review"]["exclusion_audit"][0]["rationale"] == "Synthetic test decision"
    assert result["physical_collision_verified"] is False and result["execution_ready"] is False
    assert result["geometry_coverage"]["full_cell_coverage_verified"] is False
    assert scene == original


@pytest.mark.parametrize("case", ["unchecked", "no_reviewer", "no_reason", "duplicate", "missing_pair", "unknown_pair", "stale_scene", "stale_inventory"])
def test_invalid_or_unreviewed_exclusions_are_rejected(case):
    data, scene = source()
    plan = build_geometry_review(data, scene)
    first = plan["pair_decisions"][0]
    first.update(decision="exclude", reviewed=True, reviewed_by="reviewer", rationale="test")
    if case == "unchecked": first["reviewed"] = False
    if case == "no_reviewer": first["reviewed_by"] = ""
    if case == "no_reason": first["rationale"] = "  "
    if case == "duplicate": plan["pair_decisions"][1] = copy.deepcopy(first)
    if case == "missing_pair": plan["pair_decisions"].pop()
    if case == "unknown_pair": first["pair"][0] = "/World/Unknown"
    if case == "stale_scene": scene["padding_m"] = 0.05
    if case == "stale_inventory": plan["inventory_digest"] = "other"
    with pytest.raises(ValueError): apply_geometry_review(data, scene, plan)


def test_declared_expected_paths_distinguish_mapped_missing_and_undeclared():
    data, scene = source()
    plan = build_geometry_review(data, scene)
    plan["geometry_expectations"]["tool"] = {"declared": True, "required_prim_paths": ["/World/Robot/link6", "/World/Robot/MissingTool"]}
    result = apply_geometry_review(data, scene, plan)
    coverage = result["geometry_coverage"]
    entries = coverage["declared_geometry_expectations"]["tool"]["entries"]
    assert [entry["status"] for entry in entries] == ["mapped_declared_geometry", "missing_enabled_collision_geometry"]
    assert coverage["declared_geometry_expectations"]["environment"]["status"] == "undeclared"
    assert coverage["declared_geometry_incomplete"] is True
    assert coverage["tool_coverage"] == "unverified"


@pytest.mark.parametrize("paths,declared", [(["relative"], True), (["/World/A", "/World/A"], True), (["/World/../A"], True), (["/World/A"], False)])
def test_bad_or_implicit_geometry_declarations_are_rejected(paths, declared):
    data, scene = source()
    plan = build_geometry_review(data, scene)
    plan["geometry_expectations"]["tool"] = {"declared": declared, "required_prim_paths": paths}
    with pytest.raises(ValueError): apply_geometry_review(data, scene, plan)


def test_missing_declared_environment_prevents_clear_and_execution():
    data, scene = source()
    # Explicit synthetic proxies separated far apart to isolate the coverage rule.
    for index, capsule in enumerate(scene["link_capsules"]):
        capsule.update(start_local=[index*100, 0, 0], end_local=[index*100, 0, 0], radius_m=0.001)
    action = action_fixture()
    action.update(preview_only=False, execution_enabled=True, execution_ready=True)
    for frame in action["joint_trajectory"]: frame["joint_positions"] = [0]*6
    assert check_fanuc_collision(action, scene)["status"] == "clear"
    plan = build_geometry_review(data, scene)
    plan["geometry_expectations"]["environment"] = {"declared": True, "required_prim_paths": ["/World/MissingFixture"]}
    scene = apply_geometry_review(data, scene, plan)
    report = check_fanuc_collision(action, scene)
    assert report["status"] == "indeterminate"
    assert "declared_geometry_incomplete" in report["unmodeled_geometry"]
    context = {"a": {"action": action}, "p": {"report": validate_fanuc_trajectory(action)}, "c": {"collision_report": report}}
    edges = [{"source": source, "sourceHandle": handle, "target": "g", "targetHandle": handle}
             for source, handle in [("a", "action"), ("p", "report"), ("c", "collision_report")]]
    assert exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation"}}, context, edges)["approved"] is False


def test_proxy_node_fails_when_connected_review_is_missing():
    data, _ = source()
    edges = [{"source": "i", "sourceHandle": "inventory", "target": "proxy", "targetHandle": "inventory"},
             {"source": "r", "sourceHandle": "review", "target": "proxy", "targetHandle": "review"}]
    result = exec_fanuc_usd_proxy_build({"id": "proxy", "data": {}}, {"i": {"inventory": data}, "r": {"review": None}}, edges)
    assert result["ok"] is False and result["scene"] is None


def test_offline_cli_roundtrip_keeps_all_pairs_without_motion(tmp_path, monkeypatch):
    import json
    from scripts import review_fanuc_geometry as cli

    data, scene = source()
    inventory_path, proxy_path = tmp_path / "inventory.json", tmp_path / "proxy.json"
    plan_path, summary_path, scene_path = tmp_path / "review.json", tmp_path / "summary.json", tmp_path / "scene.json"
    inventory_path.write_text(json.dumps(data), encoding="utf-8")
    proxy_path.write_text(json.dumps({"scene": scene}), encoding="utf-8")
    monkeypatch.setattr("sys.argv", ["review_fanuc_geometry.py", str(inventory_path), str(proxy_path), "--output", str(plan_path)])
    assert cli.main() == 0
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    assert all(item["decision"] == "keep" for item in plan["pair_decisions"])
    monkeypatch.setattr("sys.argv", ["review_fanuc_geometry.py", str(inventory_path), str(proxy_path), "--review", str(plan_path),
                                   "--output", str(summary_path), "--scene-output", str(scene_path)])
    assert cli.main() == 0
    result = json.loads(scene_path.read_text(encoding="utf-8"))
    assert result["excluded_self_pairs"] == []
    assert result["geometry_review"]["excluded_pair_count"] == 0
    assert json.loads(summary_path.read_text(encoding="utf-8"))["motion_requested"] is False
