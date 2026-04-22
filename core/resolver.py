"""
Conflict resolver using uniform-cost search over departure-time offsets.

Given a primary mission and other active drones, this module searches for the
minimum delay to the primary departure time that yields a clear report.
"""
from __future__ import annotations

from typing import Sequence, Any, Callable

from .models import DroneMission
from .deconfliction import check_mission


def _shifted_mission(primary: DroneMission, offset: float) -> DroneMission:
    """Return a copy of primary mission shifted by `offset` seconds."""
    return DroneMission(
        drone_id=primary.drone_id,
        waypoints=primary.waypoints,
        speed=primary.speed,
        departure_time=primary.departure_time + offset,
        mission_end_time=(
            primary.mission_end_time + offset
            if primary.mission_end_time is not None
            else None
        ),
    )


def find_safe_departure_offset(
    primary: DroneMission,
    others: Sequence[DroneMission],
    safety_buffer: float,
    max_delay_seconds: float = 300.0,
    step_seconds: float = 0.25,
    progress_callback: Callable[[float], None] | None = None,
) -> dict[str, Any]:
    """
    Find the minimum departure delay that resolves all conflicts.

    Search space (uniform-cost / Dijkstra over 1D offsets):
        state = departure offset in seconds
        start = 0.0
        goal = check_mission(shifted_primary, others).status == "clear"

    Returns:
        {
            "resolved": bool,
            "offset": float,
            "new_departure": float,
            "checks_performed": int,
            "status": str,
        }
        or
        {
            "resolved": False,
            "reason": str,
            "offset": None,
            "new_departure": None,
            "checks_performed": int,
            "status": "unresolved",
        }
    """
    if step_seconds <= 0:
        raise ValueError("step_seconds must be positive")
    if max_delay_seconds < 0:
        raise ValueError("max_delay_seconds must be non-negative")

    checked = 0
    n_steps = int(max_delay_seconds / step_seconds) + 1

    for i in range(n_steps):
        if progress_callback is not None and n_steps > 0:
            progress_callback(i / n_steps)
        offset = round(i * step_seconds, 6)
        # Round to 6 decimal places to avoid floating-point accumulation error
        # (e.g., 0.25 * 3 = 0.7499999... without rounding)
        checked += 1

        shifted = _shifted_mission(primary, offset)
        report = check_mission(shifted, others, safety_buffer=safety_buffer)
        if report.status == "clear":
            return {
                "resolved": True,
                "offset": offset,
                "new_departure": shifted.departure_time,
                "checks_performed": checked,
                "status": report.status,
            }

    if progress_callback is not None:
        progress_callback(1.0)

    return {
        "resolved": False,
        "reason": f"No conflict-free departure slot found within {max_delay_seconds:.1f}s",
        "offset": None,
        "new_departure": None,
        "checks_performed": checked,
        "status": "unresolved",
    }
