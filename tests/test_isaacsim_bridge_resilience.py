import asyncio
import importlib.util
import sys
import time
from pathlib import Path
from types import SimpleNamespace


def _load_bridge_module():
    path = Path(__file__).resolve().parents[1] / "examples" / "isaacsim" / "VlaGraphIsaacSimBridge.py"
    spec = importlib.util.spec_from_file_location("vlagraph_isaacsim_bridge_for_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_missing_usd_stage_does_not_crash_update_or_resolution(monkeypatch):
    module = _load_bridge_module()
    bridge = module.VlaGraphIsaacSimBridge()
    monkeypatch.setattr(bridge, "_get_stage", lambda: None)

    assert bridge._valid_prim_path("/World/Robot") is False
    assert bridge._find_robot_prim_candidate() == ""
    assert bridge._resolve_robot_prim_path() == bridge.robot_prim_path

    bridge._on_update(None)
    payload = bridge._build_state_payload(time.time())

    assert payload["type"] == "robot_state"
    assert "USD stage is not available" in payload["message"]


def test_backend_websocket_connection_failures_are_rate_limited(monkeypatch):
    module = _load_bridge_module()
    bridge = module.VlaGraphIsaacSimBridge()
    bridge._backend_ws_retry_interval_sec = 30.0
    attempts = 0

    async def connect(*_args, **_kwargs):
        nonlocal attempts
        attempts += 1
        raise TimeoutError("timed out during opening handshake")

    monkeypatch.setitem(sys.modules, "websockets", SimpleNamespace(connect=connect))

    async def run_check():
        first = await bridge._get_backend_websocket("state")
        second = await bridge._get_backend_websocket("state")
        return first, second

    first, second = asyncio.run(run_check())

    assert first is None
    assert second is None
    assert attempts == 1
    assert bridge._backend_ws_next_connect_at_by_kind["state"] > time.time()


def test_scene_scan_reports_stage_units_scope_and_truncation(monkeypatch):
    module = _load_bridge_module()
    bridge = module.VlaGraphIsaacSimBridge()

    class Prim:
        def __init__(self, path):
            self.path = path

        def IsValid(self): return True
        def IsActive(self): return True
        def IsAbstract(self): return False
        def GetPath(self): return self.path
        def GetName(self): return self.path.rsplit("/", 1)[-1]
        def GetTypeName(self): return "Cube"

    prims = [Prim("/World/Robot"), Prim("/World/Box1"), Prim("/World/Box2")]
    stage = SimpleNamespace(
        GetRootLayer=lambda: SimpleNamespace(identifier="cell.usd"),
        GetPrimAtPath=lambda path: Prim(path), Traverse=lambda: iter(prims),
    )
    monkeypatch.setitem(sys.modules, "pxr", SimpleNamespace(UsdGeom=SimpleNamespace(
        GetStageMetersPerUnit=lambda _: 0.001, GetStageUpAxis=lambda _: "Y",
    )))
    monkeypatch.setattr(bridge, "_get_stage", lambda: stage)
    monkeypatch.setattr(bridge, "_valid_prim_path", lambda _: True)
    monkeypatch.setattr(bridge, "_prim_world_pose", lambda _: {"position": [1, 2, 3]})
    monkeypatch.setattr(bridge, "_prim_world_bbox", lambda _: {"center": [1, 2, 3], "size": [1, 1, 1]})
    monkeypatch.setattr(bridge, "_prim_semantic_label", lambda _: "box")
    limited = bridge._scan_scene({"prim_path": "/World/Robot", "max_objects": 1})
    assert limited["meters_per_unit"] == 0.001
    assert limited["up_axis"] == "Y"
    assert limited["truncated"] is True
    assert limited["scan_scope"]["robot_excluded"] is True
    assert "sim_time" not in limited
    full = bridge._scan_scene({"prim_path": "/World/Robot", "max_objects": 10})
    assert full["truncated"] is False
    assert len(full["objects"]) == 2
