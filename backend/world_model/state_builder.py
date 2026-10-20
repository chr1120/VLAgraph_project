from __future__ import annotations

import math
from typing import Any

from .schemas import ObservedEntity, WorldState


def build_world_state(
    scan: dict[str, Any], *, episode_id: str, scene_version: str, state_version: int,
) -> WorldState:
    """Convert known stage geometry to meters/Z-up without fabricating physics.

    A Y-up stage is rotated +90 degrees around X: (x, y, z) -> (x, -z, y).
    Unknown units, coordinates or duplicate prims reject the observation.
    Missing simulation time remains unknown, never copied from wall time.
    """
    if scan.get("ok") is False:
        raise ValueError("Cannot build world state from a failed scan")
    scale = scan.get("meters_per_unit")
    if isinstance(scale, bool) or not isinstance(scale, (float, int)) or not math.isfinite(scale) or scale <= 0:
        raise ValueError("meters_per_unit must be a known finite positive number")
    axis = scan.get("up_axis")
    if axis not in ("Y", "Z"):
        raise ValueError("up_axis must be Y or Z")
    scene_id = scan.get("scene_id")
    if not isinstance(scene_id, str) or not scene_id.strip():
        raise ValueError("scene_id is required")

    def vector(raw, *, size=False):
        if not isinstance(raw, (list, tuple)) or len(raw) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in raw
        ):
            raise ValueError("Expected a finite three-component vector")
        x, y, z = [float(v) * scale for v in raw]
        if size and min(x, y, z) < 0:
            raise ValueError("Object size must be nonnegative")
        return (x, z if size else -z, y) if axis == "Y" else (x, y, z)

    seen: set[str] = set()

    def entity(item):
        if not isinstance(item, dict):
            raise ValueError("Entity must be an object")
        path = item.get("prim_path")
        if not isinstance(path, str) or not path.startswith("/") or path in seen:
            raise ValueError("Each entity requires a unique absolute prim_path")
        seen.add(path)
        return ObservedEntity(
            entity_id=f"{scene_id}:{scene_version}:{path}",
            prim_path=path, label=str(item.get("label") or item.get("name") or ""),
            position=vector(item.get("position")),
            bbox_center=vector(item["bbox_center"]) if item.get("bbox_center") is not None else None,
            size=vector(item["size"], size=True) if item.get("size") is not None else None,
        )

    objects = scan.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects must be an explicit list")
    missing = ["physics_snapshot", "robot_orientation", "robot_velocity", "joint_state", "collision_shapes"]
    if scan.get("sim_time") is None:
        missing.append("sim_time")
    if scan.get("truncated") is not False:
        missing.append("untruncated_scan")
    # Even an untruncated prim scan is filtered and is not a free-space map.
    missing.append("verified_observation_coverage")
    return WorldState(
        scene_id=scene_id, scene_version=scene_version, episode_id=episode_id,
        state_version=state_version, sim_time=scan.get("sim_time"),
        observed_at=scan.get("timestamp"), robot=entity(scan.get("robot")),
        objects=[entity(item) for item in objects], missing_fields=missing, raw_scan=scan,
    )
