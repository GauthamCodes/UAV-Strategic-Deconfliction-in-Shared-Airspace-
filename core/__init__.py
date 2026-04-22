"""
Public API for the UAV deconfliction system.

Exposes high-level interfaces for mission planning and conflict detection.

Primary entry point: `check_mission(primary, others, safety_buffer)` from deconfliction.py
Convenience interface: `get_position(drone, time)` — returns the 3D position of a drone at a given time.
"""

from .models import Waypoint, DroneMission, ConflictEvent, ConflictReport
from .deconfliction import check_mission
from .geometry import closest_approach, buffer_crossing_times
from .resolver import find_safe_departure_offset


def get_position(drone: DroneMission, time: float) -> tuple[float, float, float]:
    """
    Get the 3D position of a drone at a specific time.

    Args:
        drone: The DroneMission to query.
        time: The time (in seconds) at which to compute position.

    Returns:
        A tuple (x, y, z) representing the drone's position in meters.

    Raises:
        ValueError: If the time is outside the drone's mission window.

    Example:
        >>> primary = DroneMission(
        ...     drone_id="UAV-1",
        ...     waypoints=[Waypoint(0, 0, 50), Waypoint(100, 0, 50)],
        ...     speed=10.0,
        ...     departure_time=0.0
        ... )
        >>> x, y, z = get_position(primary, 5.0)  # Position at t=5s
        >>> print(f"Position: ({x:.1f}, {y:.1f}, {z:.1f})")
        Position: (50.0, 0.0, 50.0)
    """
    pos = drone.position_at(time)
    if pos is None:
        end_time = drone.arrival_time
        raise ValueError(
            f"Time {time:.2f}s is outside the drone's mission window "
            f"[{drone.departure_time:.2f}s, {end_time:.2f}s]"
        )
    return (pos.x, pos.y, pos.z)


__all__ = [
    "Waypoint",
    "DroneMission",
    "ConflictEvent",
    "ConflictReport",
    "check_mission",
    "get_position",
    "closest_approach",
    "buffer_crossing_times",
    "find_safe_departure_offset",
]
