"""
Pre-built scenarios for demos and tests.

Each scenario returns a tuple (primary_mission, list_of_other_missions) so
the caller can drop them straight into `check_mission`.

The scenarios deliberately span:
    - clear cases (parallel lanes, safe temporal gap, altitude separation)
    - conflict cases (perpendicular collision, tailgating too close)
    - the v1.1 spec's explicit sample test (crossing paths, different speeds)
    - a multi-drone case for visualization
"""
from __future__ import annotations

from core.models import Waypoint, DroneMission


# ---------------------------------------------------------------------------
# CLEAR SCENARIOS
# ---------------------------------------------------------------------------

def scenario_clear_parallel() -> tuple[DroneMission, list[DroneMission]]:
    """Two drones on parallel east-west lanes 20 m apart. Never conflict."""
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[Waypoint(0, 0, 50), Waypoint(100, 0, 50)],
        speed=10.0,
        departure_time=0.0,
        mission_end_time=15.0,
    )
    others = [
        DroneMission(
            drone_id="DRONE-A",
            waypoints=[Waypoint(0, 20, 50), Waypoint(100, 20, 50)],
            speed=10.0,
            departure_time=0.0,
        ),
    ]
    return primary, others


def scenario_different_speeds_safe() -> tuple[DroneMission, list[DroneMission]]:
    """
    Crossing paths but with enough speed difference that the faster drone
    has passed the crossing point by the time the primary arrives.

    Primary: (0,50)->(100,50) at 10 m/s — reaches (50,50) at t = 5.0s.
    Other:   (50,0)->(50,100) at 20 m/s — reaches (50,50) at t = 2.5s.

    At t = 5.0s, the fast drone is already at (50,100). Safe.
    """
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[Waypoint(0, 50, 30), Waypoint(100, 50, 30)],
        speed=10.0,
        departure_time=0.0,
        mission_end_time=15.0,
    )
    others = [
        DroneMission(
            drone_id="DRONE-FAST",
            waypoints=[Waypoint(50, 0, 30), Waypoint(50, 100, 30)],
            speed=20.0,
            departure_time=0.0,
        ),
    ]
    return primary, others


def scenario_same_path_safe_gap() -> tuple[DroneMission, list[DroneMission]]:
    """Two drones on the same lane, with a 50 m safe following gap."""
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[Waypoint(0, 0, 40), Waypoint(100, 0, 40)],
        speed=10.0,
        departure_time=0.0,
        mission_end_time=15.0,
    )
    others = [
        DroneMission(
            drone_id="DRONE-LEAD",
            waypoints=[Waypoint(0, 0, 40), Waypoint(100, 0, 40)],
            speed=10.0,
            departure_time=-5.0,  # 5 s ahead -> 50 m spatial gap
        ),
    ]
    return primary, others


def scenario_altitude_separation_4d() -> tuple[DroneMission, list[DroneMission]]:
    """
    Two drones cross in 2D (x,y) but are separated in altitude.
    Demonstrates why 3D+time analysis matters — a pure 2D check would
    falsely flag this as a conflict.
    """
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[Waypoint(0, 50, 100), Waypoint(100, 50, 100)],
        speed=10.0,
        departure_time=0.0,
        mission_end_time=15.0,
    )
    others = [
        DroneMission(
            drone_id="DRONE-LOW",
            waypoints=[Waypoint(50, 0, 20), Waypoint(50, 100, 20)],
            speed=10.0,
            departure_time=0.0,
        ),
    ]
    return primary, others


# ---------------------------------------------------------------------------
# CONFLICT SCENARIOS
# ---------------------------------------------------------------------------

def scenario_perpendicular_collision() -> tuple[DroneMission, list[DroneMission]]:
    """
    The canonical sample test case from the v1.1 spec:

    Two drones with crossing paths. Both reach (50, 50) at exactly t = 5.0s.
    Safety buffer should flag this as a collision.
    """
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[Waypoint(0, 50, 30), Waypoint(100, 50, 30)],
        speed=10.0,
        departure_time=0.0,
        mission_end_time=15.0,
    )
    others = [
        DroneMission(
            drone_id="DRONE-X",
            waypoints=[Waypoint(50, 0, 30), Waypoint(50, 100, 30)],
            speed=10.0,
            departure_time=0.0,
        ),
    ]
    return primary, others


def scenario_tailgating_unsafe() -> tuple[DroneMission, list[DroneMission]]:
    """Same lane, lead drone is only 3 m ahead — inside the 5 m default buffer."""
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[Waypoint(0, 0, 40), Waypoint(100, 0, 40)],
        speed=10.0,
        departure_time=0.0,
        mission_end_time=15.0,
    )
    others = [
        DroneMission(
            drone_id="DRONE-LEAD",
            waypoints=[Waypoint(0, 0, 40), Waypoint(100, 0, 40)],
            speed=10.0,
            departure_time=-0.3,  # 3 m < 5 m safety buffer -> conflict at default settings
        ),
    ]
    return primary, others


def scenario_multi_drone_busy() -> tuple[DroneMission, list[DroneMission]]:
    """
    A busy airspace with five drones on multi-leg missions. At least one
    real conflict is expected. Good for demo video.
    """
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[
            Waypoint(0, 0, 50),
            Waypoint(50, 50, 50),
            Waypoint(100, 50, 60),
            Waypoint(150, 0, 60),
        ],
        speed=12.0,
        departure_time=0.0,
        mission_end_time=30.0,
    )
    others = [
        # A high-altitude fly-over, safe
        DroneMission(
            drone_id="DRONE-HIGH",
            waypoints=[Waypoint(0, 100, 90), Waypoint(150, 100, 90)],
            speed=10.0,
            departure_time=0.0,
        ),
        # Cuts perpendicular across the primary's path and reliably conflicts
        # at the default 5 m buffer in the demo run.
        DroneMission(
            drone_id="DRONE-CUT",
            waypoints=[Waypoint(75, 0, 55), Waypoint(75, 100, 55)],
            speed=10.0,
            departure_time=3.0,
        ),
        # Far below, safe
        DroneMission(
            drone_id="DRONE-LOW",
            waypoints=[Waypoint(0, -30, 30), Waypoint(150, -30, 30)],
            speed=15.0,
            departure_time=0.0,
        ),
        # Altitude-separated from the primary's last segment
        DroneMission(
            drone_id="DRONE-BELOW",
            waypoints=[Waypoint(100, 50, 20), Waypoint(150, 0, 20)],
            speed=12.0,
            departure_time=10.0,
        ),
    ]
    return primary, others


def scenario_spec_v11_sample() -> tuple[DroneMission, list[DroneMission]]:
    """
    The exact sample test case from spec v1.1 (section 2.2).

    Primary drone: East-West from (0, 50) to (100, 50) at 10 m/s, t ∈ [0, 10].
    Other drone:   North-South from (50, 0) to (50, 100) at 5 m/s, t ∈ [0, 20].

    Both pass through (50, 50) but at different times:
    - Primary: reaches (50, 50) at t = 5.0 s
    - Other:   reaches (50, 50) at t = 10.0 s

    At t = 5.0 s:
    - Primary at (50, 50), Other at (50, 25) → separation = 25 m

    Result: CLEAR (no conflict, even with default 5 m buffer).
    """
    primary = DroneMission(
        drone_id="PRIMARY",
        waypoints=[Waypoint(0, 50, 30), Waypoint(100, 50, 30)],
        speed=10.0,
        departure_time=0.0,
        mission_end_time=10.0,
    )
    others = [
        DroneMission(
            drone_id="OTHER",
            waypoints=[Waypoint(50, 0, 30), Waypoint(50, 100, 30)],
            speed=5.0,
            departure_time=0.0,
        ),
    ]
    return primary, others


# ---------------------------------------------------------------------------
# Dispatch table used by main.py and tests.
# ---------------------------------------------------------------------------

SCENARIOS: dict[str, callable] = {
    "clear_parallel": scenario_clear_parallel,
    "different_speeds_safe": scenario_different_speeds_safe,
    "same_path_safe_gap": scenario_same_path_safe_gap,
    "altitude_separation_4d": scenario_altitude_separation_4d,
    "perpendicular_collision": scenario_perpendicular_collision,
    "tailgating_unsafe": scenario_tailgating_unsafe,
    "spec_v11_sample": scenario_spec_v11_sample,
    "multi_drone_busy": scenario_multi_drone_busy,
}
