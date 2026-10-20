import copy
import json
import math
from unittest.mock import patch

import pytest

from backend.executors.world_model_nodes import FANUC_M710_PRIM_PATH, exec_fanuc_state_read
from backend.world_model.fanuc import normalize_fanuc_state


def state_fixture():
    names = [f"J{i}" for i in range(1, 7)] + ["externalAxisJoint"]
    dofs = ["external_axis_joint"] + names[:6]
    return {
        "robot_type": "fanuc_m710ic_20l", "prim_path": FANUC_M710_PRIM_PATH,
        "physics_ready": True, "motion_active": False,
        "joint_names": names, "dof_names": dofs,
        "joint_positions_rad": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 1.25],
        "joint_target_positions_rad": [0, 0, 0, 0, 0, 0, 1.5],
        "command_dof_map": [
            {"command_name": name, "dof_index": (i + 1) % 7, "dof_name": dofs[(i + 1) % 7], "virtual": False}
            for i, name in enumerate(names)
        ],
    }


def normalize(state):
    return normalize_fanuc_state(state, expected_prim_path=FANUC_M710_PRIM_PATH,
                                 request_id="test", observed_at=1, capture_duration_sec=0.01)


def test_command_order_and_mixed_external_axis_units():
    state = state_fixture()
    original = copy.deepcopy(state)
    result = normalize(state)
    assert result.joints[0].name == "J1"
    assert result.joints[0].position == 0.1
    assert result.joints[0].dof_index == 1
    assert result.external_axes[0].position == 1.25
    assert result.external_axes[0].target_position == 1.5
    assert result.external_axes[0].unit == "m"
    assert result.external_axes[0].dof_index == 0
    assert result.joints[0].unit == "rad"
    assert result.sim_time is None
    assert result.completion_verified is False
    assert result.prediction_ready is False
    assert state == original


def test_matching_degree_mm_evidence_and_conflicting_units():
    state = state_fixture()
    state["joint_positions_deg"] = [math.degrees(v) for v in state["joint_positions_rad"][:6]] + [1250]
    assert normalize(state).external_axes[0].position == 1.25
    state["joint_positions_deg"][-1] = math.degrees(1.25)
    with pytest.raises(ValueError, match="evidence conflicts"):
        normalize(state)


@pytest.mark.parametrize("case", ["wrong_robot", "wrong_path", "duplicate_name", "length", "nan", "bool",
                                 "mapping", "duplicate_dof", "virtual_arm", "target_length"])
def test_invalid_observations_are_rejected(case):
    state = state_fixture()
    if case == "wrong_robot": state["robot_type"] = "other"
    if case == "wrong_path": state["prim_path"] = "/World/Other"
    if case == "duplicate_name": state["joint_names"][1] = "J1"
    if case == "length": state["joint_positions_rad"].pop()
    if case == "nan": state["joint_positions_rad"][0] = float("nan")
    if case == "bool": state["joint_positions_rad"][0] = True
    if case == "mapping": state["command_dof_map"][0]["dof_name"] = "J6"
    if case == "duplicate_dof": state["command_dof_map"][1].update(dof_index=1, dof_name="J1")
    if case == "virtual_arm": state["command_dof_map"][0]["virtual"] = True
    if case == "target_length": state["joint_target_positions_rad"].pop()
    with pytest.raises(ValueError):
        normalize(state)


class FakeSocket:
    def __init__(self, *, mismatch=False):
        self.sent = None
        self.closed = False
        self.mismatch = mismatch

    def send(self, value):
        self.sent = json.loads(value)

    def recv(self):
        return json.dumps({"ok": True, "request_id": "wrong" if self.mismatch else self.sent["request_id"],
                           "command": "get_state", "state": state_fixture()})

    def close(self):
        self.closed = True


@pytest.mark.parametrize("mismatch", [False, True])
def test_node_only_requests_state_and_checks_response_identity(mismatch):
    socket = FakeSocket(mismatch=mismatch)
    with patch("websocket.create_connection", return_value=socket):
        result = exec_fanuc_state_read({"id": "read", "data": {}}, {}, [])
    assert socket.sent["command"] == "get_state"
    assert "action" not in socket.sent
    assert socket.closed
    assert result["ok"] is (not mismatch)
    if mismatch:
        assert result["state"] is None
