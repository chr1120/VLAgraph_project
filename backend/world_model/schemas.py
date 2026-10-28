from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class Contract(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class ObservedEntity(Contract):
    entity_id: str
    prim_path: str
    label: str = ""
    position: tuple[float, float, float]
    bbox_center: tuple[float, float, float] | None = None
    size: tuple[float, float, float] | None = None

    @field_validator("prim_path")
    @classmethod
    def absolute_prim_path(cls, value):
        if not value.startswith("/") or value == "/" or any(part in {"", ".", ".."} for part in value[1:].split("/")):
            raise ValueError("An absolute entity prim path is required")
        return value

    @field_validator("position", "bbox_center", "size", mode="before")
    @classmethod
    def finite_vector(cls, value):
        if value is not None and (not isinstance(value, (list, tuple)) or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value
        )):
            raise ValueError("Vector components must be finite numbers")
        return value

    @field_validator("size")
    @classmethod
    def nonnegative_size(cls, value):
        if value is not None and any(component < 0 for component in value):
            raise ValueError("Object size must be nonnegative")
        return value


class JointObservation(Contract):
    name: str = Field(min_length=1)
    kind: Literal["revolute", "prismatic"]
    unit: Literal["rad", "m"]
    position: float = Field(strict=True)
    velocity: float | None = Field(default=None, strict=True)

    @model_validator(mode="after")
    def matching_unit(self):
        if self.unit != {"revolute": "rad", "prismatic": "m"}[self.kind]:
            raise ValueError("Joint unit must match joint kind")
        return self


class MobileBaseObservation(Contract):
    drive_type: Literal["unknown", "differential", "omnidirectional", "ackermann"] = "unknown"
    footprint_radius_m: float | None = Field(default=None, gt=0, strict=True)
    navigation_frame: str | None = Field(default=None, min_length=1)
    localization_valid: bool | None = Field(default=None, strict=True)


class RobotObservation(ObservedEntity):
    robot_kind: Literal["unknown", "articulated_arm", "amr", "humanoid", "mobile_manipulator"] = "unknown"
    manufacturer: str | None = Field(default=None, min_length=1)
    model: str | None = Field(default=None, min_length=1)
    # An arm can be mounted on a rail or a vehicle; its kind does not fix its base.
    base_motion: Literal["unknown", "fixed", "rail", "planar", "free"] = "unknown"
    orientation_xyzw: tuple[float, float, float, float] | None = None
    linear_velocity_m_s: tuple[float, float, float] | None = None
    angular_velocity_rad_s: tuple[float, float, float] | None = None
    joints: list[JointObservation] | None = None
    mobile_base: MobileBaseObservation | None = None
    missing_fields: list[str] = Field(default_factory=list)

    @field_validator("orientation_xyzw", "linear_velocity_m_s", "angular_velocity_rad_s", mode="before")
    @classmethod
    def finite_robot_vector(cls, value):
        return ObservedEntity.finite_vector(value)

    @field_validator("orientation_xyzw")
    @classmethod
    def unit_orientation(cls, value):
        if value is not None and not math.isclose(sum(v*v for v in value), 1, abs_tol=1e-6):
            raise ValueError("orientation_xyzw must be a unit quaternion")
        return value

    @model_validator(mode="after")
    def unique_joints(self):
        names = [joint.name for joint in self.joints or []]
        if len(names) != len(set(names)):
            raise ValueError("Joint names must be unique")
        return self


class WorldState(Contract):
    schema_version: Literal[1] = 1
    scene_id: str = Field(min_length=1)
    scene_version: str = Field(min_length=1)
    episode_id: str = Field(min_length=1)
    state_version: int = Field(ge=1)
    sim_time: float | None = Field(default=None, ge=0)
    observed_at: float | None = Field(default=None, ge=0)
    frame: Literal["right_handed_z_up_m"] = "right_handed_z_up_m"
    source: Literal["isaac_sim_stage"] = "isaac_sim_stage"
    robot: RobotObservation
    # robot remains the selected robot for existing graphs; robots includes it.
    robots: list[RobotObservation] = Field(default_factory=list)
    objects: list[ObservedEntity]
    missing_fields: list[str]
    # Stage observations are not synchronized physics snapshots or executable states.
    prediction_ready: Literal[False] = False
    raw_scan: dict[str, Any]

    @model_validator(mode="after")
    def consistent_entities(self):
        # Pre-extension v1 documents omitted robots; preserve that read path.
        if "robots" not in self.model_fields_set:
            self.robots = [self.robot.model_copy(deep=True)]
        matching = [robot for robot in self.robots if robot.prim_path == self.robot.prim_path]
        if len(matching) != 1 or matching[0] != self.robot:
            raise ValueError("Selected robot must match exactly one entry in robots")
        entities = self.robots + self.objects
        for field in ("prim_path", "entity_id"):
            values = [getattr(entity, field) for entity in entities]
            if len(values) != len(set(values)):
                raise ValueError(f"World entities require unique {field}")
        return self
