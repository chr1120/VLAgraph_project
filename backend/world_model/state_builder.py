from __future__ import annotations

import math
from typing import Any

from .schemas import JointObservation, MobileBaseObservation, ObservedEntity, RobotObservation, WorldState


def build_world_state(
    scan: dict[str, Any], *, episode_id: str, scene_version: str, state_version: int,
) -> WorldState:
    """Normalize stage geometry and optional robot observations to meters/Z-up.

    A Y-up stage is rotated +90 degrees around X: (x, y, z) -> (x, -z, y).
    Unknown units, coordinates or duplicate prims reject the observation.
    Missing simulation time remains unknown, never copied from wall time.
    Robot pose and twist are stage-world values. Joints already use rad/m and
    rad/s or m/s; mobile footprint radii already use meters.
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

    def vector(raw, *, size=False, units=None):
        if not isinstance(raw, (list, tuple)) or len(raw) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in raw
        ):
            raise ValueError("Expected a finite three-component vector")
        x, y, z = [float(v) * (scale if units is None else units) for v in raw]
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

    def number(raw):
        if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(raw):
            raise ValueError("Expected a finite number")
        return float(raw)

    def robot(item):
        geometry = entity(item)
        orientation = item.get("orientation_xyzw")
        if orientation is not None:
            if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
                raise ValueError("orientation_xyzw requires four finite components")
            x, y, z, w = [number(v) for v in orientation]
            if not math.isclose(x*x + y*y + z*z + w*w, 1, abs_tol=1e-6):
                raise ValueError("orientation_xyzw must be a unit quaternion")
            # World-frame change: q_zup = q_x(+90 degrees) * q_stage.
            orientation = ((w+x, y-z, y+z, w-x) if axis == "Y" else (x, y, z, w))
            if axis == "Y":
                orientation = tuple(v / math.sqrt(2) for v in orientation)

        joints = item.get("joints")
        if joints is not None:
            if not isinstance(joints, list):
                raise ValueError("joints must be an explicit list")
            normalized = []
            names = set()
            for joint in joints:
                if not isinstance(joint, dict):
                    raise ValueError("Each joint must be an object")
                parsed = JointObservation.model_validate(joint)
                if parsed.name in names:
                    raise ValueError("Joint names must be unique")
                names.add(parsed.name)
                normalized.append(parsed)
            joints = normalized

        mobile = item.get("mobile_base")
        if mobile is not None:
            mobile = MobileBaseObservation.model_validate(mobile)
        result = RobotObservation(
            **geometry.model_dump(),
            **{key: item[key] for key in ("robot_kind", "manufacturer", "model", "base_motion") if key in item},
            orientation_xyzw=orientation,
            linear_velocity_m_s=vector(item["linear_velocity"]) if item.get("linear_velocity") is not None else None,
            angular_velocity_rad_s=vector(item["angular_velocity_rad_s"], units=1) if item.get("angular_velocity_rad_s") is not None else None,
            joints=joints, mobile_base=mobile,
        )
        missing = [key for key in ("robot_kind", "base_motion") if getattr(result, key) == "unknown"]
        missing += [key for key in ("orientation_xyzw", "linear_velocity_m_s", "angular_velocity_rad_s")
                    if getattr(result, key) is None]
        if result.robot_kind in {"unknown", "articulated_arm", "humanoid", "mobile_manipulator"}:
            if not result.joints:
                missing.append("joint_state")
        if result.joints and any(joint.velocity is None for joint in result.joints):
            missing.append("joint_velocity")
        if result.robot_kind in {"amr", "mobile_manipulator"}:
            for key in ("drive_type", "footprint_radius_m", "navigation_frame", "localization_valid"):
                value = getattr(mobile, key) if mobile else None
                if value is None or value == "unknown":
                    missing.append(f"mobile_base.{key}")
            if mobile and mobile.localization_valid is False:
                missing.append("valid_localization")
        result.missing_fields = missing
        return result

    raw_robots = scan.get("robots")
    if raw_robots is not None and (not isinstance(raw_robots, list) or not raw_robots):
        raise ValueError("robots must be a nonempty explicit list")
    primary = scan.get("robot")
    if raw_robots is None:
        raw_robots = [primary]
    elif primary is not None and primary not in raw_robots:
        raise ValueError("Selected robot must match an entry in robots")
    robots = [robot(item) for item in raw_robots]
    selected = robots[raw_robots.index(primary)] if primary is not None else robots[0]

    objects = scan.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects must be an explicit list")
    missing = ["physics_snapshot", "collision_shapes"]
    if selected.orientation_xyzw is None:
        missing.append("robot_orientation")
    if selected.linear_velocity_m_s is None or selected.angular_velocity_rad_s is None:
        missing.append("robot_velocity")
    if "joint_state" in selected.missing_fields:
        missing.append("joint_state")
    for observed in robots:
        missing.extend(f"robots[{observed.prim_path}].{field}" for field in observed.missing_fields)
    if scan.get("sim_time") is None:
        missing.append("sim_time")
    if scan.get("truncated") is not False:
        missing.append("untruncated_scan")
    # Even an untruncated prim scan is filtered and is not a free-space map.
    missing.append("verified_observation_coverage")
    return WorldState(
        scene_id=scene_id, scene_version=scene_version, episode_id=episode_id,
        state_version=state_version, sim_time=scan.get("sim_time"),
        observed_at=scan.get("timestamp"), robot=selected, robots=robots,
        objects=[entity(item) for item in objects], missing_fields=missing, raw_scan=scan,
    )
