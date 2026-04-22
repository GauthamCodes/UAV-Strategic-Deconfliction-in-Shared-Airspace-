"""
JSON scenario loader.

Lets users define scenarios in a plain JSON file instead of Python, so
non-developers can configure the deconfliction service without touching
the codebase.

Inspired by the `save_grid` / `load_grid` pattern in my A* factory-nav
project — there, grid layouts were persisted as JSON. Here we persist
drone missions instead.

Schema:

    {
        "primary": {
            "drone_id": "PRIMARY",
            "waypoints": [[x, y, z], [x, y, z], ...],
            "speed": 10.0,
            "departure_time": 0.0,
            "mission_end_time": 20.0
        },
        "others": [
            {
                "drone_id": "DRONE-A",
                "waypoints": [...],
                "speed": 10.0,
                "departure_time": 0.0
            },
            ...
        ],
        "safety_buffer": 5.0
    }

`z` and `mission_end_time` are optional — omit them for 2D/open-ended missions.
"""
from __future__ import annotations

import json
from typing import Any

from core.models import DroneMission, Waypoint


def _waypoint_from_list(raw: list) -> Waypoint:
    if len(raw) == 2:
        return Waypoint(float(raw[0]), float(raw[1]), 0.0)
    if len(raw) == 3:
        return Waypoint(float(raw[0]), float(raw[1]), float(raw[2]))
    raise ValueError(
        f"Waypoint must have 2 or 3 coordinates, got {len(raw)}: {raw}"
    )


def _mission_from_dict(data: dict[str, Any]) -> DroneMission:
    return DroneMission(
        drone_id=str(data["drone_id"]),
        waypoints=[_waypoint_from_list(w) for w in data["waypoints"]],
        speed=float(data["speed"]),
        departure_time=float(data["departure_time"]),
        mission_end_time=(
            float(data["mission_end_time"])
            if data.get("mission_end_time") is not None
            else None
        ),
    )


def load_scenario(path: str) -> tuple[DroneMission, list[DroneMission], float]:
    """Load a scenario from a JSON file. Returns (primary, others, safety_buffer)."""
    with open(path, "r") as f:
        data = json.load(f)

    if "primary" not in data:
        raise ValueError(f"Scenario file {path!r} is missing 'primary' mission.")

    primary = _mission_from_dict(data["primary"])
    others_raw = data.get("others", [])
    others = [_mission_from_dict(o) for o in others_raw]
    safety_buffer = float(data.get("safety_buffer", 5.0))
    return primary, others, safety_buffer


def save_scenario(
    path: str,
    primary: DroneMission,
    others: list[DroneMission],
    safety_buffer: float = 5.0,
) -> None:
    """Serialize a scenario back out to JSON (useful for regression tests)."""
    def mission_to_dict(m: DroneMission) -> dict:
        return {
            "drone_id": m.drone_id,
            "waypoints": [[w.x, w.y, w.z] for w in m.waypoints],
            "speed": m.speed,
            "departure_time": m.departure_time,
            "mission_end_time": m.mission_end_time,
        }

    payload = {
        "primary": mission_to_dict(primary),
        "others": [mission_to_dict(o) for o in others],
        "safety_buffer": safety_buffer,
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
