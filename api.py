"""
UAV Strategic Deconfliction — REST API
=======================================
Run locally:  uvicorn api:app --reload --port 8000
Docker:       included in docker-compose.yml

Endpoints
---------
GET  /           Redirect to /docs
GET  /health     Liveness probe
POST /deconflict Pre-flight safety check (primary mission vs. other traffic)
POST /resolve    Find minimum departure-time delay to clear all conflicts

All responses use consistent JSON.  Errors return RFC-7807 "problem detail"
format so any client (Python, JavaScript, cURL) can handle them uniformly.
"""
from __future__ import annotations

from typing import List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import RedirectResponse
from pydantic import BaseModel, Field, field_validator

from core.deconfliction import check_mission
from core.models import DroneMission, Waypoint
from core.resolver import find_safe_departure_offset

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------

app = FastAPI(
    title="UAV Deconfliction API",
    version="2.0",
    description=(
        "Pre-flight airspace safety check for UAV missions. "
        "Detects spatio-temporal conflicts in continuous time using closed-form "
        "quadratic CPA mathematics — no discrete time-stepping."
    ),
    contact={"name": "Gautham Anil", "email": "hello@flytbase.com"},
    license_info={"name": "MIT"},
)


# ---------------------------------------------------------------------------
# Request / Response models  (Pydantic v2)
# ---------------------------------------------------------------------------

class WaypointIn(BaseModel):
    x: float = Field(..., description="East coordinate in metres")
    y: float = Field(..., description="North coordinate in metres")
    z: float = Field(0.0, description="Altitude in metres (default 0 = 2-D mode)")

    model_config = {"json_schema_extra": {"example": {"x": 0.0, "y": 50.0, "z": 30.0}}}


class MissionIn(BaseModel):
    drone_id: str = Field(..., description="Unique identifier for this drone")
    waypoints: List[WaypointIn] = Field(..., min_length=1, description="Ordered list of waypoints")
    speed: float = Field(..., gt=0, description="Constant cruising speed in m/s")
    departure_time: float = Field(0.0, description="Absolute departure time in seconds")
    mission_end_time: Optional[float] = Field(
        None, description="Hard deadline by which the mission must complete (seconds)"
    )

    @field_validator("speed")
    @classmethod
    def speed_positive(cls, v: float) -> float:
        if v <= 0:
            raise ValueError("speed must be > 0")
        return v

    model_config = {
        "json_schema_extra": {
            "example": {
                "drone_id": "UAV-1",
                "waypoints": [{"x": 0, "y": 50, "z": 30}, {"x": 100, "y": 50, "z": 30}],
                "speed": 10.0,
                "departure_time": 0.0,
                "mission_end_time": 15.0,
            }
        }
    }


class DeconflictRequest(BaseModel):
    primary: MissionIn
    others: List[MissionIn] = Field(default_factory=list, description="All other active missions")
    safety_buffer: float = Field(
        5.0, gt=0, description="Minimum safe separation in metres (default 5 m)"
    )
    include_near_misses: bool = Field(
        False, description="Also flag passes within 1.5× the safety buffer"
    )


class ResolveRequest(BaseModel):
    primary: MissionIn
    others: List[MissionIn] = Field(default_factory=list)
    safety_buffer: float = Field(5.0, gt=0)
    max_delay_seconds: float = Field(300.0, ge=0, description="Search window in seconds")
    step_seconds: float = Field(0.25, gt=0, description="Search granularity in seconds")


# ---------------------------------------------------------------------------
# Response models
# ---------------------------------------------------------------------------

class ConflictOut(BaseModel):
    other_drone: str
    severity: str
    time_s: float = Field(..., description="Exact CPA time in seconds")
    distance_m: float = Field(..., description="Minimum separation at CPA in metres")
    location: dict
    breach_window: Optional[dict] = Field(
        None,
        description="Exact interval [t_entry, t_exit] during which both drones "
                    "are inside the safety buffer (ATC-quality output)",
    )


class DeconflictResponse(BaseModel):
    status: str = Field(..., description="'clear' or 'conflict_detected'")
    conflict_count: int
    safety_buffer_m: float
    temporal_candidates_checked: int
    broad_phase_prunes: int
    conflicts: List[ConflictOut]


class ResolveResponse(BaseModel):
    resolved: bool
    offset_s: Optional[float] = None
    new_departure_s: Optional[float] = None
    checks_performed: int
    reason: Optional[str] = None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _to_mission(m: MissionIn) -> DroneMission:
    """Convert Pydantic input model to internal DroneMission."""
    try:
        return DroneMission(
            drone_id=m.drone_id,
            waypoints=[Waypoint(w.x, w.y, w.z) for w in m.waypoints],
            speed=m.speed,
            departure_time=m.departure_time,
            mission_end_time=m.mission_end_time,
        )
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.get("/", include_in_schema=False)
def root() -> RedirectResponse:
    """Redirect bare root to interactive API docs."""
    return RedirectResponse(url="/docs")


@app.get(
    "/health",
    tags=["System"],
    summary="Liveness probe",
    description="Returns 200 OK when the service is up. Used by Docker health-checks and load-balancers.",
)
def health() -> dict:
    return {"status": "ok", "service": "uav-deconfliction", "version": "2.0"}


@app.post(
    "/deconflict",
    response_model=DeconflictResponse,
    tags=["Deconfliction"],
    summary="Pre-flight safety check",
    description=(
        "Check whether a primary drone's mission is safe to execute given all other "
        "active missions in shared airspace. Returns 'clear' or 'conflict_detected' "
        "with full conflict certificates including exact CPA time, location, minimum "
        "separation, and buffer breach window [t_entry, t_exit].\n\n"
        "**No discrete time-stepping** — all conflict times are computed analytically "
        "using closed-form quadratic CPA mathematics (see MATH_DERIVATION.md)."
    ),
)
def deconflict(request: DeconflictRequest) -> DeconflictResponse:
    primary = _to_mission(request.primary)
    others = [_to_mission(o) for o in request.others]

    report = check_mission(
        primary,
        others,
        safety_buffer=request.safety_buffer,
        include_near_misses=request.include_near_misses,
    )

    conflicts_out: List[ConflictOut] = []
    for c in report.conflicts:
        breach = None
        if c.t_entry is not None and c.t_exit is not None:
            breach = {
                "t_entry_s": round(c.t_entry, 6),
                "t_exit_s": round(c.t_exit, 6),
                "duration_s": round(c.t_exit - c.t_entry, 6),
                "description": (
                    f"Both drones inside the {request.safety_buffer:.1f} m safety buffer "
                    f"for {c.t_exit - c.t_entry:.3f} s"
                ),
            }
        conflicts_out.append(
            ConflictOut(
                other_drone=c.other_drone_id,
                severity=c.severity,
                time_s=round(c.time_of_conflict, 6),
                distance_m=round(c.distance, 6),
                location={
                    "x": round(c.primary_location.x, 3),
                    "y": round(c.primary_location.y, 3),
                    "z": round(c.primary_location.z, 3),
                },
                breach_window=breach,
            )
        )

    return DeconflictResponse(
        status="clear" if report.is_clear() else "conflict_detected",
        conflict_count=len(report.conflicts),
        safety_buffer_m=report.safety_buffer,
        temporal_candidates_checked=report.total_checks,
        broad_phase_prunes=report.broad_phase_prunes,
        conflicts=conflicts_out,
    )


@app.post(
    "/resolve",
    response_model=ResolveResponse,
    tags=["Deconfliction"],
    summary="Find safe departure-time offset",
    description=(
        "When /deconflict returns 'conflict_detected', call /resolve to find the "
        "**minimum departure-time delay** for the primary mission that eliminates all "
        "conflicts. Uses uniform-cost search (linear scan over quantised offsets).\n\n"
        "Returns the offset in seconds and the new absolute departure time."
    ),
)
def resolve(request: ResolveRequest) -> ResolveResponse:
    primary = _to_mission(request.primary)
    others = [_to_mission(o) for o in request.others]

    result = find_safe_departure_offset(
        primary,
        others,
        safety_buffer=request.safety_buffer,
        max_delay_seconds=request.max_delay_seconds,
        step_seconds=request.step_seconds,
    )

    return ResolveResponse(
        resolved=result["resolved"],
        offset_s=result.get("offset"),
        new_departure_s=result.get("new_departure"),
        checks_performed=result["checks_performed"],
        reason=result.get("reason"),
    )
