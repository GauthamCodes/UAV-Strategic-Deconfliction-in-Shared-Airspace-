"""
Core data models for the UAV deconfliction system.

Everything is expressed in 3D space (x, y, z) plus a time dimension. For 2D
scenarios, simply leave z = 0 — the math degenerates cleanly.

Design decisions:
    - A `Waypoint` is a frozen 3D point.
    - A `Segment` is a constant-velocity linear edge between two waypoints, with
      exact (t0, t1) timestamps computed from the drone's constant speed.
    - A `DroneMission` owns its pre-built segment list so the solver never has
      to recompute kinematics.
    - `ConflictEvent` is a self-contained "certificate" explaining one conflict.
    - `ConflictReport` is the thing returned by the public query interface.

Inspired by the module layout in my ISRO/URSC A* factory-navigation project:
there, a `Cell` class owned its own draw/reset state; here, a `Segment` owns
its own kinematics and the solver stays pure.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
from math import sqrt
from typing import Optional

# Floating-point tolerance. Used everywhere we compare times or distances.
EPS = 1e-9


@dataclass(frozen=True)
class Waypoint:
    """A 3D waypoint in meters. For 2D problems, leave z = 0."""
    x: float
    y: float
    z: float = 0.0

    def distance_to(self, other: "Waypoint") -> float:
        """Euclidean distance to another waypoint."""
        return sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __repr__(self) -> str:
        return f"Wp({self.x:.1f}, {self.y:.1f}, {self.z:.1f})"


@dataclass(frozen=True)
class Segment:
    """
    A constant-velocity linear segment over an exact time interval [t0, t1].

    The position at any time t in [t0, t1] is:
        P(t) = start + velocity * (t - t0)

    This is the atomic unit the solver operates on. Every drone mission is
    decomposed into a sequence of these.
    """
    start: Waypoint
    end: Waypoint
    t0: float
    t1: float
    vx: float
    vy: float
    vz: float
    length: float  # spatial length of the segment in meters

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    def position_at(self, t: float) -> Waypoint:
        """Exact position at time t. Raises if t is outside [t0, t1]."""
        if t < self.t0 - EPS or t > self.t1 + EPS:
            raise ValueError(
                f"Time {t:.4f} is outside segment interval "
                f"[{self.t0:.4f}, {self.t1:.4f}]"
            )
        # Clamp tiny floating-point overshoot.
        t_clamped = min(max(t, self.t0), self.t1)
        dt = t_clamped - self.t0
        return Waypoint(
            self.start.x + self.vx * dt,
            self.start.y + self.vy * dt,
            self.start.z + self.vz * dt,
        )

    def bounding_box(self, buffer: float = 0.0) -> tuple[float, float, float, float, float, float]:
        """Spatial AABB of the segment, optionally inflated by `buffer` meters."""
        return (
            min(self.start.x, self.end.x) - buffer,
            min(self.start.y, self.end.y) - buffer,
            min(self.start.z, self.end.z) - buffer,
            max(self.start.x, self.end.x) + buffer,
            max(self.start.y, self.end.y) + buffer,
            max(self.start.z, self.end.z) + buffer,
        )


@dataclass
class DroneMission:
    """
    A drone mission = waypoints + constant speed + departure time.

    Because drones have constant velocity and only a departure time (per the
    v1.1 spec), every per-waypoint timestamp is *derived*, not supplied.

    For the primary drone, `mission_end_time` enforces the overall window:
    if the drone cannot reach its final waypoint by T_end, a ValueError is
    raised immediately so the operator never queries an infeasible plan.
    """
    drone_id: str
    waypoints: list[Waypoint]
    speed: float  # m/s, must be > 0 if path length > 0
    departure_time: float  # seconds (absolute)
    mission_end_time: Optional[float] = None
    segments: list[Segment] = field(init=False, default_factory=list)
    _segment_ends: list[float] = field(init=False, default_factory=list)
    arrival_time: float = field(init=False, default=0.0)

    def __post_init__(self):
        if not self.waypoints:
            raise ValueError(
                f"Drone '{self.drone_id}' has no waypoints."
            )

        # Total path length — used to sanity-check speed and window feasibility.
        total_length = 0.0
        for i in range(len(self.waypoints) - 1):
            total_length += self.waypoints[i].distance_to(self.waypoints[i + 1])

        if total_length <= EPS:
            # Static drone (single waypoint or zero-length path). Treat as a
            # point that exists only at its departure time. This is a deliberate
            # limitation — see reflection.md.
            self.arrival_time = self.departure_time
            return

        if self.speed <= EPS:
            raise ValueError(
                f"Drone '{self.drone_id}' has non-zero path but zero speed."
            )

        t_cursor = self.departure_time
        for i in range(len(self.waypoints) - 1):
            a = self.waypoints[i]
            b = self.waypoints[i + 1]
            length = a.distance_to(b)
            if length <= EPS:
                # Skip duplicate waypoints defensively.
                continue
            duration = length / self.speed
            t_next = t_cursor + duration
            vx = (b.x - a.x) / duration
            vy = (b.y - a.y) / duration
            vz = (b.z - a.z) / duration
            self.segments.append(
                Segment(
                    start=a,
                    end=b,
                    t0=t_cursor,
                    t1=t_next,
                    vx=vx,
                    vy=vy,
                    vz=vz,
                    length=length,
                )
            )
            self._segment_ends.append(t_next)
            t_cursor = t_next

        self.arrival_time = t_cursor

        if self.mission_end_time is not None:
            if self.arrival_time > self.mission_end_time + EPS:
                needed = self.arrival_time - self.departure_time
                have = self.mission_end_time - self.departure_time
                raise ValueError(
                    f"Drone '{self.drone_id}' cannot complete its mission "
                    f"inside the time window. Needs {needed:.2f}s but window "
                    f"only allows {have:.2f}s (T_end = {self.mission_end_time})."
                )

    def position_at(self, t: float) -> Optional[Waypoint]:
        """
        Returns the drone's position at time t, or None if the drone is not
        active at that time (i.e. before departure or after arrival).
        """
        if not self.segments:
            # Static drone — only "exists" at its departure time.
            if abs(t - self.departure_time) <= EPS:
                return self.waypoints[0]
            return None
        if t < self.departure_time - EPS or t > self.arrival_time + EPS:
            return None
        # Binary search for the segment containing t.
        idx = bisect_right(self._segment_ends, t - EPS)
        idx = min(idx, len(self.segments) - 1)
        return self.segments[idx].position_at(t)

    def total_path_length(self) -> float:
        return sum(s.length for s in self.segments)


@dataclass(frozen=True)
class ConflictEvent:
    """
    A single conflict certificate. Contains everything an operator or
    downstream system needs to understand, reproduce, and audit the conflict.
    """
    primary_drone_id: str
    other_drone_id: str
    time_of_conflict: float           # seconds (absolute)
    distance: float                    # meters at closest approach
    primary_location: Waypoint
    other_location: Waypoint
    primary_segment_index: int
    other_segment_index: int
    severity: str                      # "collision" | "buffer_breach" | "near_miss"
    overlap_start: float               # segment-pair overlap window
    overlap_end: float
    t_entry: Optional[float] = None    # entry time into safety buffer, if computed
    t_exit: Optional[float] = None     # exit time from safety buffer, if computed

    def explain(self) -> str:
        """Human-readable one-liner for logs and reports."""
        return (
            f"[{self.severity.upper()}] Primary '{self.primary_drone_id}' "
            f"vs '{self.other_drone_id}' at t = {self.time_of_conflict:.3f}s, "
            f"separation = {self.distance:.3f}m, "
            f"primary at ({self.primary_location.x:.2f}, "
            f"{self.primary_location.y:.2f}, {self.primary_location.z:.2f})"
        )


@dataclass(frozen=True)
class ConflictReport:
    """
    Top-level result returned by `check_mission`. This is the object the
    assignment's "query interface" hands back to the caller.
    """
    status: str                        # "clear" | "conflict detected" | "clear (with near misses)"
    conflicts: tuple[ConflictEvent, ...]
    primary_drone_id: str
    total_checks: int                  # temporal candidates evaluated post interval-tree
    broad_phase_prunes: int            # segment pairs rejected by AABB broad-phase
    safety_buffer: float               # the S_min used for this query

    def summary(self) -> str:
        if self.status.startswith("clear"):
            ratio = (
                f"{self.broad_phase_prunes}/{self.total_checks}"
                if self.total_checks
                else "0/0"
            )
            return (
                f"[OK] Mission CLEAR for '{self.primary_drone_id}'. "
                f"Broad-phase pruned {ratio} segment pairs."
            )
        return (
            f"[!] {len(self.conflicts)} conflict(s) detected for "
            f"'{self.primary_drone_id}' at S_min = {self.safety_buffer}m. "
            f"See conflict entries for exact breach windows."
        )

    def is_clear(self) -> bool:
        return self.status.startswith("clear")
