# UAV Strategic Deconfliction in Shared Airspace

**Is my drone mission safe to fly right now?**

This project answers that in continuous time using exact math (not time-step simulation), then exposes the result through:
1. a **Streamlit dashboard** for non-technical users,
2. a **FastAPI REST API** for integration,
3. a **CLI** for quick technical workflows.

---

## Why this project exists

Before takeoff, operators need to know if planned trajectories will violate minimum separation in shared airspace.  
This system checks mission overlap in **x, y, z, and time** and returns:

- **clear** (safe to execute), or
- **conflict detected** with exact details: who, when, where, how close, and for how long.

No sampling. No skipped events between time steps. Exact CPA-based detection.

---

## What is included

### 1. Streamlit dashboard (primary human interface)

Run:

```bash
streamlit run dashboard.py
```

Open:

```text
http://localhost:8501
```

### Dashboard capabilities

| Capability | What it gives you |
|---|---|
| Scenario explorer | One-click built-in mission sets (safe, conflict, busy airspace) |
| Upload JSON | Validate your own mission data directly |
| Plain-English status | Clear / conflict summaries for non-technical users |
| Time scrubber | Inspect live drone positions at any timestamp |
| Mission replay | Animated playback with speed control and conflict cueing |
| Advanced analytics | Separation-vs-time and 3D airspace view |
| Conflict cards | Human-readable explanation + exact conflict windows |
| Resolver suggestion | Minimum departure delay to clear conflicts |
| Builder mode | Create custom routes and test against preset traffic |
| Export tools | TXT report, PDF report, PNG map snapshot, scenario JSON |
| Session history | Reload recent analyses quickly |

### Recent Streamlit performance + UX upgrades

- Cached tone generation for faster audio playback (`@lru_cache`).
- Non-blocking audio cues (removed blocking sleep in `_play_sound`).
- Replay now triggers audio on **exact conflict-time crossing**.
- Replay frame cap + adaptive step size for smooth rendering on low-power machines.
- Performance profiles:
  - **Fast (Recommended)**: quickest feedback,
  - **Balanced**,
  - **High Detail**.
- Optional 3D panel toggle to reduce heavy rendering when needed.
- Time-series chart sample count now profile-driven.
- Resolver now runs only for real conflicts (not near-miss-only outputs).
- Export work moved behind an expander; heavy PNG generation is opt-in.
- One-click **Reset current analysis** for simpler usage.

---

## 2. FastAPI REST API (integration interface)

Run:

```bash
python -m uvicorn api:app --host 0.0.0.0 --port 8000
```

Open Swagger docs:

```text
http://localhost:8000/docs
```

### API endpoints

| Method | Endpoint | Purpose |
|---|---|---|
| GET | `/` | Redirects to `/docs` |
| GET | `/health` | Service liveness probe |
| POST | `/deconflict` | Core safety check (primary vs others) |
| POST | `/resolve` | Find minimum departure offset to clear conflicts |

### `/deconflict` capabilities

- Continuous-time analytical collision detection.
- Optional near-miss reporting (`include_near_misses`).
- Returns:
  - `status` (`clear` or `conflict_detected`)
  - conflict count,
  - exact conflict details (time, location, distance),
  - exact breach window (`t_entry_s`, `t_exit_s`, `duration_s`),
  - broad-phase metrics (`temporal_candidates_checked`, `broad_phase_prunes`).

Example request:

```json
{
  "primary": {
    "drone_id": "PRIMARY",
    "waypoints": [{"x": 0, "y": 50, "z": 30}, {"x": 100, "y": 50, "z": 30}],
    "speed": 10.0,
    "departure_time": 0.0,
    "mission_end_time": 15.0
  },
  "others": [
    {
      "drone_id": "DRONE-1",
      "waypoints": [{"x": 50, "y": 0, "z": 30}, {"x": 50, "y": 100, "z": 30}],
      "speed": 10.0,
      "departure_time": 0.0
    }
  ],
  "safety_buffer": 5.0,
  "include_near_misses": false
}
```

Example response (trimmed):

```json
{
  "status": "conflict_detected",
  "conflict_count": 1,
  "safety_buffer_m": 5.0,
  "temporal_candidates_checked": 1,
  "broad_phase_prunes": 0,
  "conflicts": [
    {
      "other_drone": "DRONE-1",
      "severity": "collision",
      "time_s": 5.0,
      "distance_m": 0.0,
      "location": {"x": 50.0, "y": 50.0, "z": 30.0},
      "breach_window": {
        "t_entry_s": 4.5,
        "t_exit_s": 5.5,
        "duration_s": 1.0
      }
    }
  ]
}
```

### `/resolve` capabilities

- Searches departure offsets and returns the **minimum safe delay**.
- Useful when `/deconflict` reports conflicts and you need an actionable fix.

Example request:

```json
{
  "primary": { "...": "same mission shape as /deconflict" },
  "others": [ { "...": "traffic missions" } ],
  "safety_buffer": 5.0,
  "max_delay_seconds": 300.0,
  "step_seconds": 0.25
}
```

Example response:

```json
{
  "resolved": true,
  "offset_s": 7.5,
  "new_departure_s": 7.5,
  "checks_performed": 31,
  "reason": null
}
```

---

## 3. CLI (technical / scripting interface)

Run a built-in scenario:

```bash
python main.py --scenario perpendicular_collision
```

Useful CLI options:

```bash
python main.py --help
python main.py --scenario clear_parallel --include-near-misses
python main.py --scenario multi_drone_busy --resolve
python main.py --file my_scenario.json --buffer 8.0
```

---

## Quick start (recommended order)

### Step 1: Install

```bash
pip install -r requirements.txt
```

### Step 2: Verify tests

```bash
pytest -q
```

Expected baseline:

```text
30 passed
```

### Step 3: Launch your preferred interface

- Dashboard: `streamlit run dashboard.py`
- API: `python -m uvicorn api:app --host 0.0.0.0 --port 8000`
- CLI: `python main.py --scenario multi_drone_busy`

---

## Docker run

Start dashboard + API together:

```bash
docker-compose up --build
```

Then open:

- Dashboard: `http://localhost:8501`
- API docs: `http://localhost:8000/docs`

---

## Built-in scenarios

1. `clear_parallel` - two drones on parallel safe lanes  
2. `different_speeds_safe` - crossing geometry, safe timing  
3. `same_path_safe_gap` - same route, sufficient temporal separation  
4. `altitude_separation_4d` - 2D crossing but safe altitude split  
5. `perpendicular_collision` - exact crossing-time collision  
6. `tailgating_unsafe` - same path, unsafe following gap  
7. `spec_v11_sample` - assignment sample scenario  
8. `multi_drone_busy` - denser mixed traffic case

---

## Project structure

```text
.
├── core/
│   ├── models.py
│   ├── geometry.py
│   ├── deconfliction.py
│   └── resolver.py
├── data/
│   ├── scenarios.py
│   └── loader.py
├── visualization/
│   ├── viz_2d.py
│   ├── viz_3d.py
│   └── viz_3d_interactive.py
├── dashboard.py            # Streamlit UI
├── api.py                  # FastAPI service
├── main.py                 # CLI entry point
├── demo.py                 # Batch scenario runner
├── test_deconfliction.py   # Test suite
├── requirements.txt
├── Dockerfile
└── docker-compose.yml
```

---

## Algorithm summary

Three-stage pipeline in `core/deconfliction.py`:

1. **Temporal pre-filtering** with interval overlap.
2. **Spatial broad-phase** with 4D AABB pruning.
3. **Exact narrow-phase** with closed-form CPA and breach window math.

This keeps detection exact while scaling better than naive segment pair scans.

---

## Performance notes

- Core analytical checks are fast for assignment-scale scenarios.
- Broad-phase pruning avoids unnecessary narrow-phase computations.
- Dashboard includes runtime controls to keep UI responsive:
  - replay frame cap,
  - optional 3D panel,
  - adaptive replay stepping,
  - configurable detail profiles.

---

## Quality status

- Test suite passing (`pytest -q`).
- API documented via Swagger/OpenAPI.
- Dashboard supports both non-technical and advanced technical workflows.
- Exports and resolver workflow integrated for assignment demonstration.

---

## Troubleshooting

### Streamlit audio does not play

- Ensure browser tab is not muted.
- Enable **Audio cues** in sidebar.
- Try Chrome/Edge if browser blocks autoplay.

### PNG/PDF export fails

- Install Kaleido (already in `requirements.txt`):

```bash
pip install kaleido
```

### API not reachable

- Confirm Uvicorn is running on port `8000`.
- Open `http://localhost:8000/health` first, then `http://localhost:8000/docs`.

---

## Submission note

This repo is structured to satisfy assignment needs:
- exact continuous-time deconfliction,
- explainable conflict output,
- human-friendly dashboard,
- developer-friendly API and CLI,
- test-backed implementation.

