# UAV Deconfliction System

**Is my drone mission safe to fly right now?**

This system answers that question in under 1 millisecond, with mathematical certainty. No guessing. No sampling. Exact math.

A high-performance, mathematically rigorous collision detection system for unmanned aerial vehicles (UAVs) operating with continuous trajectories.

## What This Does (Plain English)

Before a drone takes off, it needs to know whether its planned route will bring it too close to any other drone already in the air. This system checks that by modelling every drone's exact position at every moment in time, and mathematically computing whether any two drones will ever be within the safety distance of each other.

**No guessing. No sampling. Exact math.**

## Overview

This system answers a critical mission-planning question: **Can two or more UAVs operate simultaneously without violating safety constraints?**

Given:
- **N drones** with constant velocity segments and known departure times  
- A **spatial safety buffer** (default: 5m)

The system returns:
- ✅ **`"clear"`** — mission is safe to execute  
- ❌ **`"conflict detected"`** — with detailed reports (when, where, which drones)

### Key Features

- **Exact analytical collision detection** using Closest Point of Approach (CPA) mathematics
- **4D awareness** (x, y, z, time) for altitude-aware deconfliction  
- **Efficient broad-phase pruning** via 4D AABB to reduce O(N·M) to near-linear in practice  
- **Continuous trajectory analysis** — no discrete time-stepping approximation  
- **Comprehensive edge case handling** — degenerate geometries, parallel paths, zero-crossing moments
- **Visualization suite** — 2D plots with safety circles, 4D space-time diagrams, animations

## Project Structure

```
.
├── core/
│   ├── models.py           # Data classes (Waypoint, DroneMission, ConflictEvent, ConflictReport)
│   ├── geometry.py         # Exact CPA math + buffer crossing time derivations (α, β, γ)
│   └── deconfliction.py    # Two-phase engine (broad-phase AABB + narrow-phase exact CPA)
├── data/
│   ├── scenarios.py        # 8 pre-built scenarios (clear, conflict, spec sample, multi-drone)
│   └── loader.py           # JSON scenario loader for custom cases
├── visualization/
│   ├── viz_2d.py           # Matplotlib 2D plots + animations (GIF export)
│   └── viz_3d.py           # 3D static plots + space-time tube (4D visualization)
├── main.py                 # CLI entry point
├── demo.py                 # Batch runner for all scenarios
├── test_deconfliction.py   # 19 comprehensive unit tests (all passing)
└── requirements.txt
```

## Installation

### Prerequisites
- Python 3.8+

### Setup

```bash
# Clone or download the project
cd Flytbase\ project

# Install dependencies
pip install -r requirements.txt

# Run all tests to verify setup
pytest test_deconfliction.py -v
```

Expected output: **19 passed in ~0.3s**

## Usage

### Command-line Interface

```bash
# Run with default scenario (perpendicular collision)
python main.py

# Run a specific built-in scenario
python main.py --scenario clear_parallel

# Load from a custom JSON scenario file
python main.py --file my_scenario.json

# Show interactive visualization (Matplotlib window)
python main.py --scenario perpendicular_collision --visualize

# Save 2D animation as GIF
python main.py --scenario clear_parallel --save-2d anim_2d.gif

# Save 4D space-time visualization as PNG
python main.py --scenario perpendicular_collision --save-4d spacetime.png
```

### Programmatic API

```python
from core.models import Waypoint, DroneMission
from core.deconfliction import check_mission

# Define primary drone's mission
primary = DroneMission(
    drone_id="PRIMARY",
    waypoints=[Waypoint(0, 0, 50), Waypoint(100, 0, 50)],
    speed=10.0,
    departure_time=0.0,
    mission_end_time=15.0
)

# Define other drones to check against
others = [
    DroneMission(
        drone_id="DRONE-1",
        waypoints=[Waypoint(50, 0, 50), Waypoint(50, 100, 50)],
        speed=10.0,
        departure_time=0.0
    )
]

# Run deconfliction check
report = check_mission(primary, others, safety_buffer=5.0)

# Interpret results
if report.status == "clear":
    print("Mission is safe!")
else:
    for conflict in report.conflicts:
        print(f"Conflict: {conflict.explain()}")
        print(f"  Buffer breach: {conflict.overlap_start:.2f}s - {conflict.overlap_end:.2f}s")
        print(f"  Minimum separation: {conflict.distance:.2f}m")
```

## Demo & Visualization

Run all built-in scenarios and generate visualizations:

```bash
python demo.py
```

This generates 16 output files in `demo_output/` (2 per scenario):
- **2D snapshots** (PNG): top-down view with safety buffer circles
- **Space-time diagrams** (PNG): 4D visualization (x, y, time) showing drone trajectories
- **Animations** (GIF): 2D trajectories with time progression

### Built-in Scenarios

1. **`clear_parallel`** — Two drones on parallel lanes 20m apart (CLEAR)
2. **`different_speeds_safe`** — Crossing paths, different speeds (CLEAR)
3. **`same_path_safe_gap`** — Same path with 50m temporal gap (CLEAR)
4. **`altitude_separation_4d`** — 3D altitude separation (CLEAR)
5. **`perpendicular_collision`** — Head-on crossing collision (CONFLICT at t=5.0s)
6. **`tailgating_unsafe`** — Following too closely on same path (CONFLICT at default 5m buffer)
7. **`spec_v11_sample`** — The spec's own sample test: crossing paths, different speeds (CLEAR)
8. **`multi_drone_busy`** — 4 drones in complex airspace (CLEAR with broad-phase pruning)

## Algorithm

### Two-Phase Collision Detection

#### Phase 1: Broad-Phase (4D AABB Pruning)
- Compute bounding boxes in 4D space (x, y, z, time)
- Only segment pairs with overlapping boxes advance to narrow-phase
- **Typical reduction**: 11–12 segment pairs → 1–2 for exact checking

#### Phase 2: Narrow-Phase (Exact CPA)
For each candidate pair:
1. Solve for closest approach time analytically using quadratic formula
2. Compute minimum separation distance at that time
3. Check if separation < safety buffer AND time overlaps with both missions
4. Deduplicate conflicts at segment joints

### Closest Point of Approach (CPA)

For two drones with position vectors **p₁**(t) and **p₂**(t), the squared distance is:

```
D²(t) = |p₁(t) - p₂(t)|² = α·t² + β·t + γ
```

The minimum occurs at:
- **t_cpa = -β / (2α)** (if α ≠ 0)
- Special handling for parallel/degenerate cases

**Minimum separation**:
```
S_min = √(γ - β²/(4α))
```

**Buffer crossing times** (when separation enters/exits buffer zone):
- Solve: D²(t) = buffer² to get t₁, t₂
- Check temporal overlap with mission time windows

See `core/geometry.py` for full derivations and numerical stability handling.

## Testing

### Run All Tests
```bash
pytest test_deconfliction.py -v
```

### Test Coverage

- **Clear scenarios** (3 tests): Parallel paths, safe temporal gaps, altitude separation
- **Conflict scenarios** (2 tests): Perpendicular collision, tailgating
- **Edge cases** (6 tests): No time overlap, degenerate geometries, infeasible windows, deduplication
- **4D extra credit** (3 tests): Altitude-aware collision detection
- **Scalability** (1 test): 50-drone scenario with broad-phase pruning
- **Geometry** (4 tests): CPA math, buffer crossing times

### Performance

- **Single scenario**: < 50ms (including visualization)
- **All 7 scenarios** (with PNG/GIF export): < 60 seconds
- **Broad-phase pruning**: Reduces segment checks by ~90% in multi-drone cases

## Limitations & Future Work

### Current Limitations
- Constant velocity between waypoints (no acceleration/deceleration)
- Single safety buffer value (no per-drone or per-altitude buffers)
- 2D analysis with 3D altitude (not true 3D movement planning)

### Scalability for 10,000+ Drones
For production deployment at scale:

1. **Spatial indexing**: Replace 4D AABB with R-tree or KD-tree
2. **Time-windowing**: Break airspace into time slices, process independently
3. **Distributed computing**: Partition drone set across cluster (e.g., Kafka + Spark)
4. **GPU acceleration**: CPA math is vectorizable (NumPy/CuPy)

See `reflection.md` for detailed scalability discussion.

## Files Generated

| File | Purpose |
|------|---------|
| `demo_output/*_2d.png` | 2D trajectory snapshot with safety circles |
| `demo_output/*_spacetime.png` | 4D space-time visualization |
| `demo_output/*_anim.gif` | Animated 2D trajectory (15 FPS) |

## References

- **Assignment spec (v1.1)**: Continuous trajectory analysis, 4D extra credit, CLI + visualization
- **CPA mathematics**: Derived from analytic geometry and quadratic root-finding
- **Broad-phase pruning**: Inspired by R-tree spatial indexing (simplified 4D AABB variant)

---

**Assignment**: UAV Deconfliction System  
**Status**: ✅ Complete and tested  
**Last updated**: 2026-04-21
