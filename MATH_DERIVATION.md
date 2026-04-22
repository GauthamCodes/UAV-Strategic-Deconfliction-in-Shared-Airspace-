# CPA Math Derivation (Continuous 4D Deconfliction)

This document derives the core equations used by the conflict detector.
The system models each segment as constant velocity in 3D, then solves
for exact closest approach and safety-buffer entry/exit times.

## 1. Segment Kinematics

For two drones A and B on overlapping time interval [t_start, t_end]:

- A(t) = A0 + vA * t
- B(t) = B0 + vB * t

Define relative motion:

- r(t) = A(t) - B(t) = r0 + v * t
- r0 = A0 - B0
- v = vA - vB

Squared separation is:

D^2(t) = ||r(t)||^2 = (r0 + v t) . (r0 + v t)

Expand:

D^2(t) = alpha t^2 + beta t + gamma

where:

- alpha = v . v
- beta  = 2 (r0 . v)
- gamma = r0 . r0

## 2. Closest Point of Approach (CPA)

If alpha > 0, D^2(t) is a convex quadratic. Its unconstrained minimum is at:

t_cpa = -beta / (2 alpha)

The physically valid minimum in an overlap window [t_start, t_end] is:

t_min = clamp(t_cpa, t_start, t_end)

Minimum distance:

D_min = sqrt(D^2(t_min))

Conflict condition for safety buffer S:

D_min < S

Special case (parallel or equal velocities): alpha ~= 0.
Then distance is effectively linear/constant over time; evaluate window bounds
(and equivalently the single constant value when relative velocity is zero).

## 3. Buffer Breach Time Window

To find exact interval where drones are inside safety radius S, solve:

D^2(t) = S^2

That gives:

alpha t^2 + beta t + (gamma - S^2) = 0

Let c = gamma - S^2 and discriminant:

Delta = beta^2 - 4 alpha c

Cases:

1. Delta < 0: no crossing times.
   - If D_min < S over overlap window, entire overlap is breach.
   - Else no breach.

2. Delta = 0: tangential touch at one time t0.

3. Delta > 0: two crossings t1 <= t2:

- t1 = (-beta - sqrt(Delta)) / (2 alpha)
- t2 = (-beta + sqrt(Delta)) / (2 alpha)

Breach interval in time is [t1, t2], clipped to temporal overlap:

[t_entry, t_exit] = [max(t1, t_start), min(t2, t_end)]

A valid breach exists only if t_entry <= t_exit.

## 4. Why This Satisfies the Spec

- Continuous: no sampled time grid, no discretization error.
- Exact: closed-form quadratic minimization and crossing roots.
- Explainable: each conflict includes time, location, minimum separation,
  and breach duration [t_entry, t_exit].

## 5. Numerical Stability Notes

- Use epsilon thresholds for alpha ~= 0 and interval comparisons.
- Clamp candidate times to overlap bounds before evaluating D^2(t).
- Keep all checks in squared distance space until final reporting to avoid
  unnecessary sqrt calls.
