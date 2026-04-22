"""
Test suite for the UAV deconfliction system.

Coverage:
    1. Clear scenarios            — parallel, safe temporal gap, altitude sep
    2. Conflict scenarios         — perpendicular collision, tailgating
    3. Edge cases                 — no time overlap, parallel & equal velocity,
                                    infeasible window, segment-joint dedup
    4. 4D / extra credit          — altitude separation, 3D conflicts
    5. Scalability smoke test     — 50 drones, verify broad-phase pruning

Run with:  pytest tests/ -v
"""
import math

import pytest
from core.models import Waypoint, DroneMission
from core.deconfliction import check_mission
from core.geometry import closest_approach, buffer_crossing_times
from core.resolver import find_safe_departure_offset
from core import get_position
from data.loader import save_scenario, load_scenario
from data.scenarios import scenario_spec_v11_sample


# ---------------------------------------------------------------------------
# 1. CLEAR SCENARIOS
# ---------------------------------------------------------------------------

def test_parallel_paths_clear():
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 20.0)
    o = DroneMission("O", [Waypoint(0, 20), Waypoint(100, 20)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"
    assert len(report.conflicts) == 0


def test_same_path_safe_temporal_separation():
    """Leader started 5s earlier → 50m gap, which is > 5m buffer."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, -5.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"


def test_different_speeds_miss_crossing():
    """Fast drone already at (50,100) by the time primary arrives at (50,50)."""
    p = DroneMission("P", [Waypoint(0, 50), Waypoint(100, 50)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0), Waypoint(50, 100)], 20.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"


def test_different_speeds_ordering_matters():
    """
    Sample test case from spec v1.1: crossing paths with different speeds.

    Primary at 10 m/s on Y=50 reaches (50,50) at t=5.
    Other at 5 m/s on X=50, departing t=0 from (50,0) reaches (50,50) at t=10.
    They share the same spatial point but at different times → safe.
    """
    p = DroneMission("P", [Waypoint(0, 50), Waypoint(100, 50)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0), Waypoint(50, 100)], 5.0, 0.0)
    # At t=5: primary at (50,50), other at (50,25) → distance = 25 m.
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"


def test_spec_v11_sample_is_clear():
    """Built-in spec sample should remain clear with default 5m buffer."""
    primary, others = scenario_spec_v11_sample()
    report = check_mission(primary, others, safety_buffer=5.0)
    assert report.status == "clear"
    assert len(report.conflicts) == 0


# ---------------------------------------------------------------------------
# 2. CONFLICT SCENARIOS
# ---------------------------------------------------------------------------

def test_perpendicular_exact_collision():
    """Both drones reach (50, 50) at t=5.0. This is the canonical sample test."""
    p = DroneMission("P", [Waypoint(0, 50), Waypoint(100, 50)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0), Waypoint(50, 100)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "conflict detected"
    c = report.conflicts[0]
    assert math.isclose(c.time_of_conflict, 5.0, abs_tol=1e-6)
    assert math.isclose(c.primary_location.x, 50.0, abs_tol=1e-6)
    assert math.isclose(c.primary_location.y, 50.0, abs_tol=1e-6)
    assert math.isclose(c.distance, 0.0, abs_tol=1e-6)
    assert c.severity == "collision"


def test_tailgating_too_close():
    """Lead drone only 10 m ahead, in a 20 m buffer → conflict."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, -1.0)
    report = check_mission(p, [o], safety_buffer=20.0)
    assert report.status == "conflict detected"


def test_near_miss_not_flagged_by_default():
    """5.05 m is > 5 m buffer → no conflict by default."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(0, 5.05), Waypoint(100, 5.05)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"


def test_near_miss_flagged_when_requested():
    """Turn on include_near_misses → same 5.05 m pass is now reported."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(0, 5.05), Waypoint(100, 5.05)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0, include_near_misses=True)
    assert report.status.startswith("clear")
    assert any(c.severity == "near_miss" for c in report.conflicts)


# ---------------------------------------------------------------------------
# 3. EDGE CASES
# ---------------------------------------------------------------------------

def test_no_temporal_overlap():
    """Other finishes long before primary starts → safe."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 100.0, 120.0)
    o = DroneMission("O", [Waypoint(50, -10), Waypoint(50, 10)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"


def test_infeasible_window_raises():
    """Primary can't reach its final waypoint in time → ValueError."""
    with pytest.raises(ValueError, match="cannot complete"):
        DroneMission("P", [Waypoint(0, 0), Waypoint(1000, 0)], 10.0, 0.0, 50.0)


def test_mission_without_waypoints_raises():
    with pytest.raises(ValueError):
        DroneMission("EMPTY", [], 10.0, 0.0)


def test_zero_speed_nonzero_path_raises():
    with pytest.raises(ValueError):
        DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 0.0, 0.0)


def test_zero_length_segment_is_treated_as_static_point():
    """Two identical consecutive waypoints should not create a moving segment."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(0, 0)], 10.0, 0.0)
    o = DroneMission("O", [Waypoint(100, 100), Waypoint(120, 120)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"
    assert len(p.segments) == 0


def test_single_waypoint_mission_is_static_and_clear():
    """A single waypoint means no motion; the query should return clear."""
    p = DroneMission("P", [Waypoint(0, 0, 10)], 10.0, 0.0)
    o = DroneMission("O", [Waypoint(100, 100, 10), Waypoint(120, 120, 10)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"
    assert len(p.segments) == 0


def test_mission_window_exceeded_raises_value_error():
    """Primary should fail fast when it cannot finish before mission_end_time."""
    with pytest.raises(ValueError, match="cannot complete"):
        DroneMission("P", [Waypoint(0, 0), Waypoint(500, 0)], 10.0, 0.0, 20.0)


def test_parallel_same_velocity_constant_distance():
    """α ≈ 0 branch: identical velocity vectors → distance is constant."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(0, 3), Waypoint(100, 3)], 10.0, 0.0)
    result = closest_approach(p.segments[0], o.segments[0])
    assert result is not None
    assert math.isclose(result["distance"], 3.0, abs_tol=1e-6)
    assert result["alpha"] < 1e-9  # confirm we hit the degenerate branch


def test_deduplication_at_segment_joint():
    """
    A conflict exactly at a segment joint must be reported once.

    Primary has a 2-segment path through (50, 0) and an adjacent other drone
    crosses right at the joint. Without dedup this would fire twice.
    """
    p = DroneMission(
        "P",
        [Waypoint(0, 0), Waypoint(50, 0), Waypoint(100, 0)],
        10.0, 0.0, 15.0,
    )
    o = DroneMission("O", [Waypoint(50, -10), Waypoint(50, 10)], 10.0, 4.5)
    report = check_mission(p, [o], safety_buffer=5.0)
    # Expect at most one conflict, at approximately t=5.0 near (50,0).
    assert len(report.conflicts) == 1


def test_buffer_crossing_times():
    """Solve the entry/exit times of the buffer — used for time-interval reports."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, -10), Waypoint(50, 10)], 10.0, 4.5)
    crossing = buffer_crossing_times(p.segments[0], o.segments[0], 5.0)
    assert crossing is not None
    t_enter, t_exit = crossing
    assert t_enter < t_exit


# ---------------------------------------------------------------------------
# 4. 4D / EXTRA CREDIT — altitude-separated cases
# ---------------------------------------------------------------------------

def test_altitude_separation_3d_safe():
    """Crossing in XY but 80 m apart in Z → clearly safe."""
    p = DroneMission("P", [Waypoint(0, 50, 100), Waypoint(100, 50, 100)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0, 20), Waypoint(50, 100, 20)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=10.0)
    assert report.status == "clear"


def test_altitude_close_conflict_3d():
    """Crossing with only 2 m altitude diff → conflict if buffer is 5 m."""
    p = DroneMission("P", [Waypoint(0, 50, 50), Waypoint(100, 50, 50)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0, 52), Waypoint(50, 100, 52)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "conflict detected"
    c = report.conflicts[0]
    assert math.isclose(c.distance, 2.0, abs_tol=1e-6)


def test_altitude_just_outside_buffer_3d():
    """Crossing with 5.1 m altitude diff → just outside 5 m buffer."""
    p = DroneMission("P", [Waypoint(0, 50, 50), Waypoint(100, 50, 50)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0, 55.1), Waypoint(50, 100, 55.1)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"


# ---------------------------------------------------------------------------
# 5. SCALABILITY SMOKE TEST
# ---------------------------------------------------------------------------

def test_broad_phase_prunes_many_distant_drones():
    """With 50 unrelated drones far away, broad phase should prune >90%."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    others = []
    for i in range(50):
        others.append(
            DroneMission(
                f"O{i}",
                [Waypoint(10_000 + i * 10, 10_000), Waypoint(10_000 + i * 10 + 50, 10_000)],
                10.0,
                0.0,
            )
        )
    report = check_mission(p, others, safety_buffer=5.0)
    assert report.status == "clear"
    naive_pairs = len(p.segments) * sum(len(o.segments) for o in others)
    assert report.total_checks <= naive_pairs
    assert report.broad_phase_prunes / report.total_checks > 0.9


def test_exact_buffer_boundary_is_clear():
    """Exactly at S_min should be clear because breach requires distance < S_min."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(0, 5.0), Waypoint(100, 5.0)], 10.0, 0.0)
    report = check_mission(p, [o], safety_buffer=5.0)
    assert report.status == "clear"
    assert len(report.conflicts) == 0


def test_get_position_helper_returns_tuple_and_uses_arrival_window():
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0)
    x, y, z = get_position(p, 2.0)
    assert math.isclose(x, 20.0, abs_tol=1e-6)
    assert math.isclose(y, 0.0, abs_tol=1e-6)
    assert math.isclose(z, 0.0, abs_tol=1e-6)

    with pytest.raises(ValueError, match=r"\[0\.00s, 10\.00s\]"):
        get_position(p, 11.0)


def test_loader_round_trip(tmp_path):
    """Scenario save/load should preserve mission data and safety buffer."""
    p = DroneMission("P", [Waypoint(0, 0, 10), Waypoint(100, 0, 10)], 10.0, 0.0, 15.0)
    o1 = DroneMission("O1", [Waypoint(50, -10, 10), Waypoint(50, 10, 10)], 8.0, 1.0)

    out = tmp_path / "scenario.json"
    save_scenario(str(out), p, [o1], safety_buffer=7.5)
    p2, others2, buffer2 = load_scenario(str(out))

    assert p2.drone_id == "P"
    assert len(others2) == 1
    assert others2[0].drone_id == "O1"
    assert math.isclose(buffer2, 7.5, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# 6. RESOLVER MODULE TESTS
# ---------------------------------------------------------------------------

def test_resolver_already_clear():
    """When primary and other don't conflict at offset=0, return immediately."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(0, 20), Waypoint(100, 20)], 10.0, 0.0)
    result = find_safe_departure_offset(p, [o], 5.0, max_delay_seconds=5.0, step_seconds=0.25)
    assert result["resolved"] is True
    assert result["offset"] == 0.0
    assert result["checks_performed"] == 1
    assert result["status"] == "clear"


def test_resolver_finds_minimum_offset():
    """Perpendicular collision: resolver should find a safe departure delay."""
    p = DroneMission("P", [Waypoint(0, 50), Waypoint(100, 50)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0), Waypoint(50, 100)], 10.0, 0.0)
    result = find_safe_departure_offset(p, [o], 5.0, max_delay_seconds=5.0, step_seconds=0.25)
    assert result["resolved"] is True
    assert result["offset"] > 0.0
    assert result["new_departure"] > 0.0
    assert "checks_performed" in result


def test_resolver_no_solution_when_max_delay_insufficient():
    """When max_delay is too small, resolver should return resolved=False."""
    p = DroneMission("P", [Waypoint(0, 50), Waypoint(100, 50)], 10.0, 0.0, 15.0)
    o = DroneMission("O", [Waypoint(50, 0), Waypoint(50, 100)], 10.0, 0.0)
    result = find_safe_departure_offset(p, [o], 5.0, max_delay_seconds=0.1, step_seconds=0.05)
    assert result["resolved"] is False
    assert "reason" in result
    assert result["offset"] is None
    assert result["new_departure"] is None
    assert result["status"] == "unresolved"


def test_resolver_invalid_step_raises():
    """Resolver should raise ValueError for invalid step_seconds."""
    p = DroneMission("P", [Waypoint(0, 0), Waypoint(100, 0)], 10.0, 0.0)
    o = DroneMission("O", [Waypoint(50, 0), Waypoint(50, 100)], 10.0, 0.0)
    with pytest.raises(ValueError):
        find_safe_departure_offset(p, [o], 5.0, step_seconds=0.0)
    with pytest.raises(ValueError):
        find_safe_departure_offset(p, [o], 5.0, step_seconds=-0.5)
