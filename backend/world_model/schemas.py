from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator


class Contract(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class ObservedEntity(Contract):
    entity_id: str
    prim_path: str
    label: str = ""
    position: tuple[float, float, float]
    bbox_center: tuple[float, float, float] | None = None
    size: tuple[float, float, float] | None = None

    @field_validator("size")
    @classmethod
    def nonnegative_size(cls, value):
        if value is not None and any(component < 0 for component in value):
            raise ValueError("Object size must be nonnegative")
        return value


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
    robot: ObservedEntity
    objects: list[ObservedEntity]
    missing_fields: list[str]
    # This first adapter produces geometric observations, not executable states.
    prediction_ready: Literal[False] = False
    raw_scan: dict[str, Any]
