"""
Exact closest-point-of-approach (CPA) math for linear constant-velocity
segments in 3D space + time.

This is the mathematical core of the deconfliction engine. It implements
continuous-time conflict detection in closed form — no for-loop over
discrete time steps — which is the hard constraint in spec v1.1.

Derivation
----------
Two drones A and B each on one segment active over [t0_A, t1_A] and
[t0_B, t1_B]. On the overlap interval [a, b] = [max(t0s), min(t1s)] the
positions are:

    A(t) = a_0 + v_A * t       (absolute-time linear form)
    B(t) = b_0 + v_B * t

Relative position:

    R(t) = A(t) - B(t) = c + r * t,   c = a_0 - b_0,   r = v_A - v_B

Squared distance:

    d²(t) = |R(t)|² = (r·r)t² + 2(c·r)t + (c·c)
          = α t² + β t + γ

This is a convex parabola (α >= 0). The unconstrained minimum is at
t* = -β / (2α). Clamp it to the overlap [a, b] to get the constrained
global minimum.

Buffer crossing times (entry/exit of the S_min sphere) come from solving:

    α t² + β t + (γ - S²) = 0
    → t = (-β ± sqrt(β² - 4α(γ - S²))) / (2α)

Special case: α ≈ 0 means the drones have identical velocities. The
relative distance is then constant on the overlap.

Why this is better than discrete stepping
-----------------------------------------
A for-loop `for t in arange(t_start, t_end, dt)` misses any conflict whose
closest approach falls between two sample points. The assignment v1.1 spec
explicitly forbids that approach. The closed-form solution above is both
exact and ~100x faster.
"""
from __future__ import annotations

from math import sqrt
from typing import Optional

from .models import Segment, EPS


def _relative_quadratic_coeffs(s1: Segment, s2: Segment) -> tuple[float, float, float]:
    """
    Build the coefficients (α, β, γ) of d²(t) = α t² + β t + γ for the two
    segments in absolute time.

    Each segment stores its velocity in absolute coordinates, so the absolute-
    time form is P(t) = start + v * (t - t0) = (start - v * t0) + v * t.
    """
    # "Anchor" points: position at absolute time t = 0, extrapolated from
    # segment's own constant-velocity line.
    ax = s1.start.x - s1.vx * s1.t0
    ay = s1.start.y - s1.vy * s1.t0
    az = s1.start.z - s1.vz * s1.t0
    bx = s2.start.x - s2.vx * s2.t0
    by = s2.start.y - s2.vy * s2.t0
    bz = s2.start.z - s2.vz * s2.t0

    cx, cy, cz = ax - bx, ay - by, az - bz
    rx, ry, rz = s1.vx - s2.vx, s1.vy - s2.vy, s1.vz - s2.vz

    alpha = rx * rx + ry * ry + rz * rz
    beta = 2.0 * (cx * rx + cy * ry + cz * rz)
    gamma = cx * cx + cy * cy + cz * cz
    return alpha, beta, gamma


def closest_approach(s1: Segment, s2: Segment) -> Optional[dict]:
    """
    Exact closest-approach certificate between two segments over the window
    during which both are active.

    Returns
    -------
    None if the two segments are never simultaneously active. Otherwise a
    dict with the conflict certificate:
        t_min          : time of closest approach
        distance       : minimum separation distance in meters
        point_a        : position of drone A at t_min
        point_b        : position of drone B at t_min
        overlap_start  : start of temporal overlap
        overlap_end    : end of temporal overlap
        alpha, beta, gamma : the d²(t) quadratic coefficients (for audit)
    """
    overlap_start = max(s1.t0, s2.t0)
    overlap_end = min(s1.t1, s2.t1)

    if overlap_end < overlap_start - EPS:
        return None  # No temporal overlap → no possible conflict

    alpha, beta, gamma = _relative_quadratic_coeffs(s1, s2)

    def d2_at(t: float) -> float:
        return alpha * t * t + beta * t + gamma

    # The true minimum is either at the interior stationary point or at one
    # of the overlap endpoints. Evaluate all three and take the smallest.
    candidates = [overlap_start, overlap_end]
    if alpha > EPS:
        t_interior = -beta / (2.0 * alpha)
        if overlap_start <= t_interior <= overlap_end:
            candidates.append(t_interior)

    t_min = min(candidates, key=d2_at)
    d2_min = max(0.0, d2_at(t_min))  # guard against -0.0 from float error
    # Keep this hot path in scalar math.sqrt (not numpy) to avoid array overhead
    # inside the per-segment-pair inner loop.
    distance = sqrt(d2_min)

    return {
        "t_min": t_min,
        "distance": distance,
        "point_a": s1.position_at(t_min),
        "point_b": s2.position_at(t_min),
        "overlap_start": overlap_start,
        "overlap_end": overlap_end,
        "alpha": alpha,
        "beta": beta,
        "gamma": gamma,
    }


def buffer_crossing_times(
    s1: Segment, s2: Segment, safety_buffer: float
) -> Optional[tuple[float, float]]:
    """
    Compute the exact times (t_enter, t_exit) during which the two segments
    are *inside* the safety buffer. Returns None if they never breach.

    Solves  α t² + β t + (γ - S²) = 0  and clips the roots to the overlap.
    """
    overlap_start = max(s1.t0, s2.t0)
    overlap_end = min(s1.t1, s2.t1)
    if overlap_end < overlap_start - EPS:
        return None

    alpha, beta, gamma = _relative_quadratic_coeffs(s1, s2)
    s2_val = safety_buffer * safety_buffer
    c = gamma - s2_val

    # Degenerate case: α ≈ 0 means zero relative velocity → constant distance.
    if abs(alpha) <= EPS:
        # d² is linear in t: d²(t) = β t + γ
        if abs(beta) <= EPS:
            # d² is completely constant across the overlap.
            return (overlap_start, overlap_end) if c < 0 else None
        # Linear: find the root, then classify each endpoint to build the
        # breach interval.
        root = -c / beta
        d2_start = beta * overlap_start + gamma
        d2_end = beta * overlap_end + gamma
        start_in = d2_start < s2_val
        end_in = d2_end < s2_val
        if start_in and end_in:
            return (overlap_start, overlap_end)
        if start_in:
            return (overlap_start, max(overlap_start, min(overlap_end, root)))
        if end_in:
            return (max(overlap_start, min(overlap_end, root)), overlap_end)
        return None

    # General case: α > 0. Parabola opens upward; breach zone (if any) is
    # between the two real roots of α t² + β t + c = 0.
    disc = beta * beta - 4.0 * alpha * c
    if disc < -EPS:
        return None
    disc = max(0.0, disc)
    sqrt_disc = sqrt(disc)
    t_root_1 = (-beta - sqrt_disc) / (2.0 * alpha)
    t_root_2 = (-beta + sqrt_disc) / (2.0 * alpha)
    t_low, t_high = min(t_root_1, t_root_2), max(t_root_1, t_root_2)

    t_enter = max(t_low, overlap_start)
    t_exit = min(t_high, overlap_end)
    if t_exit < t_enter - EPS:
        return None
    return (t_enter, t_exit)
