from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import Field, model_validator

from .schemas import Contract


class FanucAxisObservation(Contract):
    name: str
    kind: Literal["revolute", "prismatic"]
    unit: Literal["rad", "m"]
    position: float
    target_position: float | None = None
    dof_index: int | None = Field(default=None, ge=0)
    virtual: bool


class FanucObservation(Contract):
    schema_version: Literal[1] = 1
    robot_type: Literal["fanuc_m710ic_20l"] = "fanuc_m710ic_20l"
    prim_path: str = Field(min_length=1)
    request_id: str
    observed_at: float = Field(ge=0)
    capture_duration_sec: float = Field(ge=0)
    sim_time: float | None = Field(default=None, ge=0)
    physics_ready: bool
    motion_active: bool
    joints: list[FanucAxisObservation]
    external_axes: list[FanucAxisObservation]
    missing_fields: list[str]
    prediction_ready: Literal[False] = False
    completion_verified: Literal[False] = False
    source: Literal["fanuc_dedicated_bridge_get_state"] = "fanuc_dedicated_bridge_get_state"
    raw_state: dict[str, Any]


class SnapshotAxis(Contract):
    name: str
    position: float
    target_position: float
    velocity: float | None
    unit: Literal["rad", "m"]
    virtual: bool


class FanucPhysicsSnapshot(Contract):
    schema_name: Literal["vlagraph.fanuc_snapshot.v1"] = Field(alias="schema")
    episode_id: str = Field(min_length=1)
    state_version: int = Field(ge=1)
    sim_time: float = Field(ge=0)
    physics_step: int = Field(ge=0)
    prim_path: str
    observed_at: float = Field(ge=0)
    axes: list[SnapshotAxis]
    synchronized_joint_snapshot: Literal[True]
    geometry: dict[str, Any]
    prediction_ready: Literal[False]
    raw_state: dict[str, Any]

    @model_validator(mode="after")
    def check_axes(self):
        names = [axis.name for axis in self.axes]
        if len(names) != len(set(names)) or not {f"J{i}" for i in range(1, 7)}.issubset(names):
            raise ValueError("Snapshot requires unique J1..J6 axes")
        for axis in self.axes:
            if axis.name not in {f"J{i}" for i in range(1, 7)} | {"externalAxisJoint"}:
                raise ValueError("Unsupported snapshot axis")
            if axis.unit != ("m" if axis.name == "externalAxisJoint" else "rad"):
                raise ValueError("Snapshot axis unit mismatch")
            if not axis.virtual and axis.velocity is None:
                raise ValueError("Physical axes require measured velocities")
            if axis.virtual and axis.name != "externalAxisJoint":
                raise ValueError("Arm snapshot axes must be physical")
        if self.raw_state.get("prim_path") != self.prim_path or self.raw_state.get("physics_ready") is not True:
            raise ValueError("Snapshot robot identity or physics readiness mismatch")
        units = self.geometry.get("meters_per_unit")
        if isinstance(units, bool) or not isinstance(units, (int, float)) or not math.isfinite(units) or units <= 0:
            raise ValueError("Snapshot requires finite positive stage units")
        if self.geometry.get("up_axis") not in {"Y", "Z"}:
            raise ValueError("Unknown stage up-axis")
        return self


def normalize_fanuc_state(
    state: dict[str, Any], *, expected_prim_path: str, request_id: str,
    observed_at: float, capture_duration_sec: float,
) -> FanucObservation:
    """Normalize the dedicated M-710 bridge's command-ordered state.

    That bridge labels a mixed-unit array joint_positions_rad: J1..J6 are
    radians, externalAxisJoint is meters. Its *_deg arrays use degrees/mm.
    Never use raw articulation DOF indices to index this command-ordered array.
    """
    if state.get("robot_type") != "fanuc_m710ic_20l":
        raise ValueError("Unsupported FANUC state profile")
    if not expected_prim_path or state.get("prim_path") != expected_prim_path:
        raise ValueError("FANUC state prim_path does not match the selected robot")
    if not isinstance(state.get("physics_ready"), bool) or not isinstance(state.get("motion_active"), bool):
        raise ValueError("physics_ready and motion_active must be explicit booleans")
    names = state.get("joint_names")
    positions = state.get("joint_positions_rad")
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names) or len(set(names)) != len(names):
        raise ValueError("joint_names must contain unique names")
    if not isinstance(positions, list) or len(names) != len(positions):
        raise ValueError("Command-ordered joint names and positions must have matching lengths")
    required = {f"J{i}" for i in range(1, 7)}
    if not required.issubset(names) or set(names) - required - {"externalAxisJoint"}:
        raise ValueError("Expected J1..J6 and optional externalAxisJoint")

    def number(value):
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError("Axis values must be finite numbers")
        return float(value)

    targets = state.get("joint_target_positions_rad")
    if targets is not None and (not isinstance(targets, list) or len(targets) != len(names)):
        raise ValueError("Target positions must match command joint names")
    for sim_key, command_key in (("joint_positions_rad", "joint_positions_deg"),
                                 ("joint_target_positions_rad", "joint_target_positions_deg")):
        command_values = state.get(command_key)
        sim_values = state.get(sim_key)
        if command_values is not None:
            if not isinstance(command_values, list) or len(command_values) != len(names) or sim_values is None:
                raise ValueError("FANUC command-unit evidence must match joint names")
            for name, sim_value, command_value in zip(names, sim_values, command_values):
                expected = number(sim_value) * 1000 if name == "externalAxisJoint" else math.degrees(number(sim_value))
                if not math.isclose(expected, number(command_value), rel_tol=1e-5, abs_tol=1e-6):
                    raise ValueError("FANUC degree/mm evidence conflicts with the radians/meters profile")
    mappings = state.get("command_dof_map")
    if not isinstance(mappings, list) or not all(isinstance(item, dict) for item in mappings):
        raise ValueError("command_dof_map is required")
    by_name = {item.get("command_name"): item for item in mappings}
    if len(by_name) != len(mappings) or set(by_name) != set(names):
        raise ValueError("command_dof_map must match command joint names exactly")
    dof_names = state.get("dof_names")
    if not isinstance(dof_names, list):
        raise ValueError("dof_names is required")
    used_indices: set[int] = set()
    axes = []
    for index, name in enumerate(names):
        mapping = by_name[name]
        virtual = mapping.get("virtual")
        dof_index = mapping.get("dof_index")
        if not isinstance(virtual, bool):
            raise ValueError("Each command mapping must declare virtual status")
        if not virtual:
            if (isinstance(dof_index, bool) or not isinstance(dof_index, int)
                    or not 0 <= dof_index < len(dof_names) or dof_index in used_indices
                    or mapping.get("dof_name") != dof_names[dof_index]):
                raise ValueError("Invalid or ambiguous articulation DOF mapping")
            used_indices.add(dof_index)
        external = name == "externalAxisJoint"
        if virtual and not external:
            raise ValueError("Arm joints must map to physical DOFs")
        axes.append(FanucAxisObservation(
            name=name, kind="prismatic" if external else "revolute", unit="m" if external else "rad",
            position=number(positions[index]),
            target_position=number(targets[index]) if targets is not None else None,
            dof_index=dof_index, virtual=virtual,
        ))
    # motion_active=false is not a per-command completion or a success result.
    return FanucObservation(
        prim_path=expected_prim_path, request_id=request_id, observed_at=observed_at,
        capture_duration_sec=capture_duration_sec, physics_ready=state["physics_ready"],
        motion_active=state["motion_active"], joints=[axis for axis in axes if axis.kind == "revolute"],
        external_axes=[axis for axis in axes if axis.kind == "prismatic"],
        missing_fields=["synchronized_physics_snapshot", "sim_time", "joint_velocities", "tcp_pose",
                        "base_pose", "uf_ut_calibration", "collision_shapes", "command_completion"],
        raw_state=state,
    )
