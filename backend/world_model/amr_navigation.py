"""Conservative grid planning and differential-drive control (stdlib only).

Shared by the backend and Isaac Sim. Coordinates are world Z-up meters;
nonzero occupancy, including unknown cells, is blocked. No physics claims.
"""
from __future__ import annotations

import hashlib
import heapq
import json
import math


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def finite(value, name, lower=None, upper=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if lower is not None and value < lower or upper is not None and value > upper:
        raise ValueError(f"{name} is out of range")
    return float(value)


def point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("Expected world XY coordinates in meters")
    return tuple(finite(v, "coordinate") for v in value)


class OccupancyGrid:
    def __init__(self, data):
        if not isinstance(data, dict) or data.get("frame") != "right_handed_z_up_m":
            raise ValueError("Navigation map requires right_handed_z_up_m coordinates")
        if not isinstance(data.get("stage_identifier"), str) or not data["stage_identifier"]:
            raise ValueError("Navigation map requires stage_identifier")
        self.resolution = finite(data.get("resolution_m"), "resolution_m", .01, 10)
        self.origin = point(data.get("origin_m"))
        self.cells = data.get("cells")
        if not isinstance(self.cells, list) or not self.cells or not isinstance(self.cells[0], list) or not self.cells[0]:
            raise ValueError("Navigation cells must be a nonempty row-major grid")
        self.height, self.width = len(self.cells), len(self.cells[0])
        if self.width * self.height > 65536 or any(
            not isinstance(row, list) or len(row) != self.width or
            any(type(v) is not int or v not in {-1, 0, 1} for v in row) for row in self.cells
        ):
            raise ValueError("Invalid occupancy grid; use -1 unknown, 0 free, 1 occupied (max 65536 cells)")
        self.data = data
        self.digest = fingerprint({k: data[k] for k in
                                   ("stage_identifier", "frame", "resolution_m", "origin_m", "cells")})

    def cell(self, p):
        return tuple(math.floor((p[i] - self.origin[i]) / self.resolution) for i in (0, 1))

    def center(self, cell):
        return tuple(self.origin[i] + (cell[i] + .5) * self.resolution for i in (0, 1))

    def segment_clear(self, a, b, radius):
        """Exact segment/AABB tests; square inflation conservatively bounds a disk."""
        for i, count in ((0, self.width), (1, self.height)):
            if min(a[i], b[i]) - radius <= self.origin[i] or max(a[i], b[i]) + radius >= self.origin[i] + count*self.resolution:
                return False
        lo = self.cell((min(a[0], b[0])-radius, min(a[1], b[1])-radius))
        hi = self.cell((max(a[0], b[0])+radius, max(a[1], b[1])+radius))
        for y in range(max(0, lo[1]), min(self.height-1, hi[1])+1):
            for x in range(max(0, lo[0]), min(self.width-1, hi[0])+1):
                if self.cells[y][x] == 0:
                    continue
                enter, leave = 0., 1.
                for i, c in ((0, x), (1, y)):
                    minimum = self.origin[i] + c*self.resolution - radius
                    maximum = minimum + self.resolution + 2*radius
                    delta = b[i]-a[i]
                    if abs(delta) < 1e-12:
                        if not minimum <= a[i] <= maximum:
                            enter, leave = 1., 0.
                            break
                    else:
                        t1, t2 = (minimum-a[i])/delta, (maximum-a[i])/delta
                        enter, leave = max(enter, min(t1, t2)), min(leave, max(t1, t2))
                if enter <= leave:
                    return False
        return True


def plan_path(map_data, start, goal, *, robot_prim_path, radius_m, episode_id, observed_at, margin_m=.1):
    grid = OccupancyGrid(map_data)
    start, goal = point(start), point(goal)
    radius = finite(radius_m, "radius_m", .01, 10)
    margin = finite(margin_m, "margin_m", .01, 5)
    clearance = radius + margin
    if not isinstance(episode_id, str) or not episode_id:
        raise ValueError("An observation episode_id is required")
    observed_at = finite(observed_at, "observed_at", 0)
    if not isinstance(robot_prim_path, str) or not robot_prim_path.startswith("/"):
        raise ValueError("An absolute robot prim path is required")
    if not grid.segment_clear(start, start, clearance) or not grid.segment_clear(goal, goal, clearance):
        raise ValueError("Start or goal has insufficient clearance or unknown coverage")
    if grid.segment_clear(start, goal, clearance):
        path = [start, goal]
    else:
        first, last = grid.cell(start), grid.cell(goal)
        if not grid.segment_clear(start, grid.center(first), clearance) or not grid.segment_clear(grid.center(last), goal, clearance):
            raise ValueError("Start or goal cannot connect to grid centers at this resolution")
        queue, costs, parents = [(0., first)], {first: 0.}, {}
        visited = set()
        while queue:
            _, cell = heapq.heappop(queue)
            if cell in visited:
                continue
            visited.add(cell)
            if cell == last:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (cell[0]+dx, cell[1]+dy)
                cost = costs[cell] + 1
                if cost >= costs.get(neighbor, float("inf")):
                    continue
                if not grid.segment_clear(grid.center(cell), grid.center(neighbor), clearance):
                    continue
                costs[neighbor], parents[neighbor] = cost, cell
                heuristic = abs(neighbor[0]-last[0]) + abs(neighbor[1]-last[1])
                heapq.heappush(queue, (cost + heuristic, neighbor))
        if last not in visited:
            raise ValueError("No traversable AMR route")
        route = [last]
        while route[-1] != first:
            route.append(parents[route[-1]])
        raw = [start] + [grid.center(c) for c in reversed(route)] + [goal]
        path, index = [start], 0
        while index < len(raw)-1:
            following = index+1
            for candidate in range(index+2, len(raw)):
                if grid.segment_clear(path[-1], raw[candidate], clearance):
                    following = candidate
                else:
                    break
            path.append(raw[following])
            index = following
    result = {"schema": "vlagraph.amr_plan.v1", "simulation_only": True,
              "robot_prim_path": robot_prim_path, "stage_identifier": map_data["stage_identifier"],
              "map_digest": grid.digest, "start_m": list(start), "goal_m": list(goal),
              "episode_id": episode_id, "observed_at": observed_at,
              "radius_m": radius, "margin_m": margin, "waypoints_m": [list(p) for p in path]}
    result["digest"] = fingerprint(result)
    return result


def validate_plan(plan):
    if not isinstance(plan, dict) or plan.get("schema") != "vlagraph.amr_plan.v1" or plan.get("simulation_only") is not True:
        raise ValueError("A simulation AMR plan is required")
    if plan.get("digest") != fingerprint({k: v for k, v in plan.items() if k != "digest"}):
        raise ValueError("AMR plan digest mismatch")
    points = plan.get("waypoints_m")
    if not isinstance(points, list) or not 2 <= len(points) <= 65538:
        raise ValueError("Invalid AMR waypoints")
    for p in points:
        point(p)
    if point(points[0]) != point(plan.get("start_m")) or point(points[-1]) != point(plan.get("goal_m")):
        raise ValueError("Plan endpoints do not match waypoints")
    finite(plan.get("radius_m"), "radius_m", .01, 10)
    finite(plan.get("margin_m"), "margin_m", .01, 5)
    return plan


def differential_command(position, yaw, target, *, max_speed=.25, max_turn=.6):
    error = math.atan2(target[1]-position[1], target[0]-position[0]) - yaw
    error = math.atan2(math.sin(error), math.cos(error))
    distance = math.dist(position, target)
    turn = max(-max_turn, min(max_turn, 1.5*error))
    # Turn in place before translation to avoid cutting a grid corner.
    speed = min(max_speed, distance) * max(0., math.cos(error)) if abs(error) < .15 else 0.
    return speed, turn


def wheel_speeds(speed, turn, radius, separation):
    return ((speed-turn*separation/2)/radius, (speed+turn*separation/2)/radius)


def routes_conflict(first, second):
    """Reserve whole AMR route corridors; overlapping routes must wait/replan.

    This is spatial exclusion, not a time-dependent collision prediction.
    """
    a_points, b_points = first["waypoints_m"], second["waypoints_m"]
    clearance = sum(p["radius_m"] + p["margin_m"] for p in (first, second))
    if (len(a_points)-1)*(len(b_points)-1) > 10000:
        return True  # Keep reservation checking bounded; unresolved is blocked.
    def point_distance(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        denom = dx*dx + dy*dy
        t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denom)) if denom else 0.
        return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)
    def cross(a, b, p):
        return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    for a, b in zip(a_points, a_points[1:]):
        for c, d in zip(b_points, b_points[1:]):
            overlapping_bounds = all(max(min(a[i], b[i]), min(c[i], d[i])) <= min(max(a[i], b[i]), max(c[i], d[i])) for i in (0, 1))
            intersect = overlapping_bounds and cross(a, b, c)*cross(a, b, d) <= 0 and cross(c, d, a)*cross(c, d, b) <= 0
            if intersect or min(point_distance(a, c, d), point_distance(b, c, d),
                                point_distance(c, a, b), point_distance(d, a, b)) <= clearance:
                return True
    return False
