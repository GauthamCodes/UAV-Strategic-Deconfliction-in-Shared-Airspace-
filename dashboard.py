"""
UAV Strategic Deconfliction — Interactive Dashboard
Run with: streamlit run dashboard.py
"""
from __future__ import annotations

import io
import json
import math
import os
import struct
import tempfile
import time
import wave
from datetime import datetime
from pathlib import Path
from typing import Optional

import pandas as pd
import plotly.graph_objects as go
import streamlit as st

# Project imports
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.deconfliction import check_mission
from core.models import DroneMission, Waypoint
from core.resolver import find_safe_departure_offset, _shifted_mission
from benchmark_data import BENCHMARK_POINTS
from data.scenarios import SCENARIOS
from visualization.viz_3d_interactive import build_3d_plotly


HISTORY_DIR = Path("scenario_history")
HISTORY_DIR.mkdir(exist_ok=True)

SCENARIO_DESCRIPTIONS = {
    "clear_parallel": "Two drones flying parallel east-west lanes 20 m apart. Always safe.",
    "different_speeds_safe": "Crossing paths but the fast drone clears the intersection first.",
    "same_path_safe_gap": "Same route, lead drone departed 5 s earlier — 50 m gap maintained.",
    "altitude_separation_4d": "Crossing in 2D but 80 m apart in altitude — 3D analysis needed.",
    "perpendicular_collision": "Both drones reach (50, 50) at exactly t = 5 s. Direct collision.",
    "tailgating_unsafe": "Primary follows 3 m behind the lead — inside the safety buffer.",
    "spec_v11_sample": "Spec v1.1 sample: crossing paths, different speeds -> CLEAR.",
    "multi_drone_busy": "Five drones in complex airspace with at least one real conflict.",
}

OTHER_COLORS = ["#2ecc71", "#f39c12", "#9b59b6", "#1abc9c", "#e74c3c", "#3498db", "#e67e22"]
PRIMARY_COLOR = "#1f77b4"
CONFLICT_COLOR = "#f85149"
NEARMISS_COLOR = "#e3b341"


st.set_page_config(
    page_title="UAV Deconfliction",
    page_icon="✈",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown(
    """
<style>
[data-testid="stAppViewContainer"] {background:#0d1117;}
[data-testid="stSidebar"] {background:#161b22;border-right:1px solid #30363d;}
h1,h2,h3,h4,label,p,li {color:#e6edf3 !important;}
.stButton>button {background:#238636;color:#fff;border:none;border-radius:6px;font-weight:600;}
.stButton>button:hover {background:#2ea043;}
.status-clear {background:#0d4429;border:1px solid #238636;border-radius:8px;padding:16px 20px;font-size:1.2rem;font-weight:700;color:#3fb950;}
.status-conflict {background:#3d1a1a;border:1px solid #da3633;border-radius:8px;padding:16px 20px;font-size:1.2rem;font-weight:700;color:#f85149;}
.status-nearmiss {background:#2d2000;border:1px solid #d29922;border-radius:8px;padding:16px 20px;font-size:1.2rem;font-weight:700;color:#e3b341;}
.conflict-card {background:#1f2937;border-left:4px solid #f85149;border-radius:6px;padding:14px 18px;margin:8px 0;}
.nearmiss-card {background:#1f2937;border-left:4px solid #e3b341;border-radius:6px;padding:14px 18px;margin:8px 0;}
.card-title {font-size:1rem;font-weight:700;color:#f85149;margin-bottom:6px;}
.card-title-nm {font-size:1rem;font-weight:700;color:#e3b341;margin-bottom:6px;}
.card-body {font-size:0.88rem;color:#c9d1d9;line-height:1.6;}
.resolve-box {background:#0d2d1a;border:1px solid #238636;border-radius:5px;padding:8px 12px;margin-top:8px;font-size:0.85rem;color:#3fb950;}
.legend-item {display:inline-flex;align-items:center;margin:0 10px 4px 0;font-size:0.82rem;color:#8b949e;}
.legend-dot {width:12px;height:12px;border-radius:50%;display:inline-block;margin-right:5px;}
[data-testid="metric-container"] {background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px;}
</style>
""",
    unsafe_allow_html=True,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _severity_color(severity: str) -> str:
    return {
        "collision": CONFLICT_COLOR,
        "buffer_breach": CONFLICT_COLOR,
        "near_miss": NEARMISS_COLOR,
    }.get(severity, CONFLICT_COLOR)


def _make_summary_sentence(report, buffer_val: float) -> str:
    """Return a one-sentence plain-English summary of the most critical conflict."""
    if not report.conflicts:
        return ""
    conflict = min(report.conflicts, key=lambda item: item.distance)
    loc = conflict.primary_location
    if conflict.severity == "collision":
        verb = "will collide with"
    else:
        verb = "will breach the safety buffer near"
    return (
        f"{conflict.other_drone_id} {verb} your drone at "
        f"({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f}) m at T+{conflict.time_of_conflict:.2f} s. "
        f"Minimum separation: {conflict.distance:.2f} m (buffer: {buffer_val:.1f} m)."
    )


def _style_waypoint_preview(df: pd.DataFrame, invalid_mask: pd.DataFrame) -> pd.io.formats.style.Styler:
    """Highlight invalid waypoint cells in a validation preview table."""
    preview_df = df.copy()
    preview_df["Row Status"] = ["Invalid" if invalid_mask.loc[idx].any() else "Valid" for idx in preview_df.index]
    preview_df = preview_df[["X", "Y", "Alt", "Row Status"]]

    def _highlight_cells(value: object, row_idx: int, column: str) -> str:
        if invalid_mask.loc[row_idx, column]:
            return "background-color: #3d1a1a; color: #f85149; font-weight: 700;"
        return "background-color: #0f1720; color: #c9d1d9;"

    styler = preview_df.style
    for column in ["X", "Y", "Alt"]:
        styler = styler.map(lambda value, col=column: "", subset=[column])

    def _apply_row(row: pd.Series) -> list[str]:
        styles = []
        for column in row.index:
            if column in ("X", "Y", "Alt") and invalid_mask.loc[row.name, column]:
                styles.append("background-color: #3d1a1a; color: #f85149; font-weight: 700;")
            elif column in ("X", "Y", "Alt"):
                styles.append("background-color: #0f1720; color: #c9d1d9;")
            else:
                styles.append("background-color: #111827; color: #9ca3af; font-style: italic;")
        return styles

    return styler.apply(_apply_row, axis=1)


def _tone_wav_bytes(freq: float, duration_s: float, volume: float = 0.3) -> bytes:
    sample_rate = 22050
    n_samples = int(sample_rate * duration_s)
    frames = bytearray()
    for i in range(n_samples):
        value = int(volume * 32767 * math.sin(2.0 * math.pi * freq * (i / sample_rate)))
        frames += struct.pack("<h", value)

    buf = io.BytesIO()
    with wave.open(buf, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(bytes(frames))
    return buf.getvalue()


def _play_sound(sound_type: str) -> None:
    if sound_type == "clear":
        st.audio(_tone_wav_bytes(880.0, 0.13), format="audio/wav", autoplay=True)
    elif sound_type == "warning":
        st.audio(_tone_wav_bytes(660.0, 0.16), format="audio/wav", autoplay=True)
    elif sound_type == "conflict":
        st.audio(_tone_wav_bytes(330.0, 0.22), format="audio/wav", autoplay=True)


def _save_to_history(primary: DroneMission, others: list[DroneMission], report, buffer_val: float) -> None:
    payload = {
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "primary": {
            "drone_id": primary.drone_id,
            "waypoints": [[w.x, w.y, w.z] for w in primary.waypoints],
            "speed": primary.speed,
            "departure_time": primary.departure_time,
            "mission_end_time": primary.mission_end_time,
        },
        "others": [
            {
                "drone_id": d.drone_id,
                "waypoints": [[w.x, w.y, w.z] for w in d.waypoints],
                "speed": d.speed,
                "departure_time": d.departure_time,
                "mission_end_time": d.mission_end_time,
            }
            for d in others
        ],
        "safety_buffer": buffer_val,
        "status": report.status,
        "conflicts": len(report.conflicts),
    }
    filename = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{primary.drone_id}.json"
    (HISTORY_DIR / filename).write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _load_history(path: Path) -> tuple[DroneMission, list[DroneMission], float]:
    data = json.loads(path.read_text(encoding="utf-8"))

    def _wp(raw: list[float]) -> Waypoint:
        if len(raw) == 2:
            return Waypoint(raw[0], raw[1], 0.0)
        return Waypoint(raw[0], raw[1], raw[2])

    def _mission(raw: dict) -> DroneMission:
        return DroneMission(
            drone_id=str(raw["drone_id"]),
            waypoints=[_wp(w) for w in raw["waypoints"]],
            speed=float(raw["speed"]),
            departure_time=float(raw["departure_time"]),
            mission_end_time=raw.get("mission_end_time"),
        )

    primary = _mission(data["primary"])
    others = [_mission(item) for item in data.get("others", [])]
    return primary, others, float(data.get("safety_buffer", 5.0))


def _plain_english_explanation(c, safety_buffer: float, resolver_result: Optional[dict]) -> tuple[str, str]:
    drone_name = c.other_drone_id
    t = c.time_of_conflict
    dist = c.distance
    loc = c.primary_location
    sev = c.severity.replace("_", " ").title()

    mins = int(t // 60)
    secs = t % 60
    time_str = f"{mins}m {secs:.1f}s" if mins else f"{secs:.1f}s"

    if c.severity == "collision":
        base = (
            f"{sev}: <b>{drone_name}</b> will be at the exact same point "
            f"as your drone at <b>T + {time_str}</b>. Separation = 0.0 m."
        )
    else:
        base = (
            f"{sev}: <b>{drone_name}</b> comes within <b>{dist:.1f} m</b> "
            f"(minimum safe: {safety_buffer:.0f} m) at <b>T + {time_str}</b>, "
            f"near ({loc.x:.0f}, {loc.y:.0f})."
        )

    if c.t_entry is not None and c.t_exit is not None:
        duration = c.t_exit - c.t_entry
        base += f" Danger window: <b>{duration:.1f} s</b> ({c.t_entry:.1f}s -> {c.t_exit:.1f}s)."

    resolve_html = ""
    if resolver_result and resolver_result.get("resolved"):
        delay = resolver_result["offset"]
        delay_str = f"{delay:.1f} s" if delay < 1 else f"{delay:.0f} s"
        resolve_html = (
            f'<div class="resolve-box">Suggested fix: Delay departure by <b>{delay_str}</b> '
            "to clear all conflicts.</div>"
        )

    return base, resolve_html


def _build_plotly_map(
    primary: DroneMission,
    others: list[DroneMission],
    report,
    safety_buffer: float,
    t_slider: Optional[float] = None,
) -> go.Figure:
    fig = go.Figure()

    def _add_path(drone: DroneMission, color: str, dash: str, name: str) -> None:
        xs = [w.x for w in drone.waypoints]
        ys = [w.y for w in drone.waypoints]
        hover_parts = [f"({x:.1f}, {y:.1f})" for x, y in zip(xs, ys)]
        fig.add_trace(
            go.Scatter(
                x=xs,
                y=ys,
                mode="lines+markers",
                name=name,
                line=dict(color=color, width=3, dash=dash),
                marker=dict(size=7, color=color),
                hovertemplate=(
                    f"<b>{name}</b><br>"
                    "Waypoint: %{customdata}<br>"
                    f"Speed: {drone.speed:.1f} m/s<br>"
                    f"Departure: T + {drone.departure_time:.1f}s"
                    "<extra></extra>"
                ),
                customdata=hover_parts,
            )
        )

    _add_path(primary, PRIMARY_COLOR, "solid", f"PRIMARY ({primary.drone_id})")
    for i, drone in enumerate(others):
        _add_path(drone, OTHER_COLORS[i % len(OTHER_COLORS)], "dash", drone.drone_id)

    for conflict in report.conflicts:
        col = _severity_color(conflict.severity)
        breach_info = ""
        if conflict.t_entry is not None and conflict.t_exit is not None:
            breach_info = f"<br>Danger window: {conflict.t_entry:.1f}s -> {conflict.t_exit:.1f}s"
        fig.add_trace(
            go.Scatter(
                x=[conflict.primary_location.x],
                y=[conflict.primary_location.y],
                mode="markers",
                name=f"Conflict @ t={conflict.time_of_conflict:.1f}s",
                marker=dict(size=22, color=col, symbol="x", line=dict(width=3, color=col)),
                hovertemplate=(
                    f"<b>{conflict.severity.replace('_', ' ').upper()}</b><br>"
                    f"With: {conflict.other_drone_id}<br>"
                    f"Time: T + {conflict.time_of_conflict:.2f}s<br>"
                    f"Separation: {conflict.distance:.2f}m<br>"
                    f"Min safe: {safety_buffer:.1f}m{breach_info}<extra></extra>"
                ),
                showlegend=True,
            )
        )

    if t_slider is not None:
        live_conflict = any(
            c.t_entry is not None
            and c.t_exit is not None
            and c.t_entry <= t_slider <= c.t_exit
            and c.severity in ("collision", "buffer_breach")
            for c in report.conflicts
        )

        def _add_live(drone: DroneMission, base_color: str, label: str) -> None:
            pos = drone.position_at(t_slider)
            if pos is None:
                return
            color = CONFLICT_COLOR if (live_conflict and drone is primary) else base_color
            angles = [2 * math.pi * k / 60 for k in range(61)]
            bx = [pos.x + safety_buffer * math.cos(a) for a in angles]
            by = [pos.y + safety_buffer * math.sin(a) for a in angles]
            fig.add_trace(
                go.Scatter(
                    x=bx,
                    y=by,
                    mode="lines",
                    line=dict(color=color, width=1, dash="dot"),
                    showlegend=False,
                    hoverinfo="skip",
                )
            )
            fig.add_trace(
                go.Scatter(
                    x=[pos.x],
                    y=[pos.y],
                    mode="markers+text",
                    text=[label],
                    textposition="top center",
                    marker=dict(size=14, color=color, line=dict(width=2, color="#ffffff")),
                    name=f"{label} @ t={t_slider:.1f}s",
                    hovertemplate=(
                        f"<b>{label}</b><br>"
                        f"Position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.0f}m alt)<br>"
                        f"t = {t_slider:.2f}s<extra></extra>"
                    ),
                    showlegend=False,
                )
            )

        _add_live(primary, PRIMARY_COLOR, "PRIMARY")
        for i, drone in enumerate(others):
            _add_live(drone, OTHER_COLORS[i % len(OTHER_COLORS)], drone.drone_id)

    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0d1117",
        plot_bgcolor="#0d1117",
        title=dict(text=f"Airspace View — S_min = {safety_buffer:.1f} m", font=dict(size=15, color="#e6edf3")),
        xaxis=dict(title="X position (m)", gridcolor="#21262d", zeroline=False, showgrid=True),
        yaxis=dict(title="Y position (m)", gridcolor="#21262d", zeroline=False, scaleanchor="x", showgrid=True),
        legend=dict(bgcolor="#161b22", bordercolor="#30363d", borderwidth=1, font=dict(size=11, color="#c9d1d9")),
        hovermode="closest",
        margin=dict(l=50, r=20, t=50, b=40),
        height=500,
    )
    fig.add_annotation(
        x=0.98,
        y=0.98,
        xref="paper",
        yref="paper",
        text="N ↑",
        showarrow=False,
        font=dict(color="#c9d1d9", size=14),
    )
    return fig


def _is_conflict_live(report, t_value: float) -> bool:
    """Return True if any real conflict is active at the provided time."""
    for conflict in report.conflicts:
        if conflict.severity not in ("collision", "buffer_breach"):
            continue
        if conflict.t_entry is not None and conflict.t_exit is not None:
            if conflict.t_entry <= t_value <= conflict.t_exit:
                return True
        elif abs(conflict.time_of_conflict - t_value) <= 0.1:
            # Fallback for events without an explicit breach window.
            return True
    return False


def _build_time_series(primary: DroneMission, others: list[DroneMission], safety_buffer: float) -> go.Figure:
    t_start = min([primary.departure_time] + [d.departure_time for d in others])
    t_end = max([primary.arrival_time] + [d.arrival_time for d in others])
    times = [t_start + ((t_end - t_start) * i / 299) for i in range(300)]

    fig = go.Figure()
    for i, drone in enumerate(others):
        seps = []
        for t in times:
            pp = primary.position_at(t)
            op = drone.position_at(t)
            if pp and op:
                dist = math.sqrt((pp.x - op.x) ** 2 + (pp.y - op.y) ** 2 + (pp.z - op.z) ** 2)
                seps.append(dist)
            else:
                seps.append(None)

        fig.add_trace(
            go.Scatter(
                x=times,
                y=seps,
                mode="lines",
                name=drone.drone_id,
                line=dict(color=OTHER_COLORS[i % len(OTHER_COLORS)], width=2),
            )
        )

    fig.add_hline(
        y=safety_buffer,
        line_dash="dash",
        line_color=CONFLICT_COLOR,
        annotation_text=f"Safety buffer ({safety_buffer:.0f}m)",
        annotation_font_color=CONFLICT_COLOR,
    )
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0d1117",
        plot_bgcolor="#0d1117",
        title=dict(text="Separation Over Time (PRIMARY vs each drone)", font=dict(size=13, color="#e6edf3")),
        xaxis=dict(title="Time (s)", gridcolor="#21262d"),
        yaxis=dict(title="Separation (m)", gridcolor="#21262d"),
        height=280,
        margin=dict(l=50, r=20, t=40, b=40),
    )
    return fig


def _export_text_report(primary: DroneMission, others: list[DroneMission], report, safety_buffer: float, resolver_result: Optional[dict]) -> str:
    lines = [
        "=" * 60,
        "  UAV STRATEGIC DECONFLICTION REPORT",
        "=" * 60,
        f"  Primary drone  : {primary.drone_id}",
        f"  Speed          : {primary.speed:.1f} m/s",
        f"  Departure      : T + {primary.departure_time:.1f}s",
        f"  Waypoints      : {len(primary.waypoints)}",
        f"  Safety buffer  : {safety_buffer:.1f} m",
        f"  Other drones   : {len(others)}",
        "",
        f"  STATUS: {report.status.upper()}",
        "",
    ]

    if report.conflicts:
        lines.append(f"  {len(report.conflicts)} conflict(s) found:")
        for i, c in enumerate(report.conflicts, 1):
            lines += [
                "",
                f"  [{i}] {c.severity.replace('_', ' ').upper()}",
                f"      With drone   : {c.other_drone_id}",
                f"      Time         : T + {c.time_of_conflict:.2f}s",
                f"      Separation   : {c.distance:.3f}m",
                f"      Location     : ({c.primary_location.x:.1f}, {c.primary_location.y:.1f}, {c.primary_location.z:.0f}m alt)",
            ]
            if c.t_entry is not None and c.t_exit is not None:
                lines.append(
                    f"      Danger window: {c.t_entry:.1f}s -> {c.t_exit:.1f}s ({c.t_exit - c.t_entry:.1f}s duration)"
                )
        lines.append("")
        if resolver_result and resolver_result.get("resolved"):
            lines += [
                "  SUGGESTED FIX:",
                f"  Delay departure by {resolver_result['offset']:.1f}s",
                f"  New departure time: T + {resolver_result['new_departure']:.1f}s",
            ]
    else:
        lines.append("  No conflicts detected. Mission is safe to execute.")

    lines += ["", "=" * 60]
    return "\n".join(lines)


def _build_map_png_bytes(fig: go.Figure) -> tuple[Optional[bytes], Optional[str]]:
    """Render a Plotly figure to PNG bytes when kaleido is available."""
    try:
        return fig.to_image(format="png", scale=2), None
    except Exception:
        return None, "Map image export needs kaleido. Install with: pip install kaleido"


def _build_pdf_report_bytes(
    primary: DroneMission,
    others: list[DroneMission],
    report,
    safety_buffer: float,
    resolver_result: Optional[dict],
    map_png_bytes: Optional[bytes] = None,
) -> tuple[Optional[bytes], Optional[str]]:
    """Build a compact PDF report with optional embedded map image."""
    try:
        from fpdf import FPDF
    except Exception:
        return None, "PDF export needs fpdf2. Install with: pip install fpdf2"

    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=12)
    pdf.add_page()
    pdf.set_font("Helvetica", "B", 14)
    pdf.cell(0, 8, "UAV Strategic Deconfliction Report", ln=1)
    pdf.set_font("Helvetica", size=10)
    pdf.cell(0, 6, f"Primary: {primary.drone_id} | Drones: {len(others) + 1} | Buffer: {safety_buffer:.1f} m", ln=1)
    pdf.cell(0, 6, f"Status: {report.status}", ln=1)

    summary = _make_summary_sentence(report, safety_buffer)
    if summary:
        pdf.ln(2)
        pdf.multi_cell(0, 6, summary)

    if report.conflicts:
        pdf.ln(2)
        pdf.set_font("Helvetica", "B", 11)
        pdf.cell(0, 6, "Conflict Details", ln=1)
        pdf.set_font("Helvetica", size=10)
        for idx, conflict in enumerate(report.conflicts[:10], 1):
            pdf.multi_cell(
                0,
                6,
                (
                    f"{idx}. {conflict.severity} vs {conflict.other_drone_id} at "
                    f"T+{conflict.time_of_conflict:.2f}s, separation {conflict.distance:.2f}m"
                ),
            )

    if resolver_result and resolver_result.get("resolved"):
        pdf.ln(2)
        pdf.set_font("Helvetica", "B", 11)
        pdf.cell(0, 6, "Resolver Suggestion", ln=1)
        pdf.set_font("Helvetica", size=10)
        pdf.multi_cell(
            0,
            6,
            (
                f"Delay departure by {resolver_result['offset']:.2f}s "
                f"(new departure T+{resolver_result['new_departure']:.2f}s)."
            ),
        )

    temp_map_path: Optional[str] = None
    if map_png_bytes:
        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".png") as tmp:
                tmp.write(map_png_bytes)
                temp_map_path = tmp.name
            pdf.ln(3)
            pdf.set_font("Helvetica", "B", 11)
            pdf.cell(0, 6, "Map Snapshot", ln=1)
            pdf.image(temp_map_path, w=180)
        except Exception:
            pass
        finally:
            if temp_map_path and os.path.exists(temp_map_path):
                os.unlink(temp_map_path)

    raw = pdf.output(dest="S")
    if isinstance(raw, bytes):
        return raw, None
    if isinstance(raw, bytearray):
        return bytes(raw), None
    return str(raw).encode("latin-1", errors="ignore"), None


def _parse_uploaded_json(file_bytes: bytes) -> tuple[DroneMission, list[DroneMission], float]:
    data = json.loads(file_bytes)

    def _wp(raw: list[float]) -> Waypoint:
        if len(raw) == 2:
            return Waypoint(raw[0], raw[1], 0.0)
        return Waypoint(raw[0], raw[1], raw[2])

    def _mission(raw: dict) -> DroneMission:
        return DroneMission(
            drone_id=str(raw["drone_id"]),
            waypoints=[_wp(w) for w in raw["waypoints"]],
            speed=float(raw["speed"]),
            departure_time=float(raw["departure_time"]),
            mission_end_time=raw.get("mission_end_time"),
        )

    primary = _mission(data["primary"])
    others = [_mission(o) for o in data.get("others", [])]
    return primary, others, float(data.get("safety_buffer", 5.0))


def _run_resolver_with_progress(primary: DroneMission, others: list[DroneMission], safety_buffer: float) -> Optional[dict]:
    if not others:
        return None

    bar = st.progress(0.0, text="Searching departure offsets...")
    result = find_safe_departure_offset(
        primary,
        others,
        safety_buffer,
        max_delay_seconds=120,
        step_seconds=0.5,
        progress_callback=lambda p: bar.progress(min(max(p, 0.0), 1.0), text="Searching departure offsets..."),
    )
    bar.empty()
    return result


def _run_check(primary: DroneMission, others: list[DroneMission], buffer_val: float) -> tuple:
    t0 = time.perf_counter()
    with st.spinner(f"Analyzing {len(others) + 1} drone trajectories..."):
        report = check_mission(primary, others, buffer_val)
    elapsed_ms = (time.perf_counter() - t0) * 1000
    st.session_state["analysis_ms"] = elapsed_ms

    resolver_result = None
    if report.conflicts:
        with st.spinner("Finding minimum departure delay..."):
            resolver_result = _run_resolver_with_progress(primary, others, buffer_val)

    _save_to_history(primary, others, report, buffer_val)
    return primary, others, report, buffer_val, resolver_result


if hasattr(st, "fragment"):

    @st.fragment
    def _scrubber_fragment(primary: DroneMission, others: list[DroneMission], report, buffer_val: float) -> float:
        t_start_all = min([primary.departure_time] + [d.departure_time for d in others])
        t_end_all = max([primary.arrival_time] + [d.arrival_time for d in others])
        t_slider = st.slider(
            "Time Scrubber",
            min_value=float(t_start_all),
            max_value=float(t_end_all),
            value=float(t_start_all),
            step=0.1,
            format="%.1f s",
            help="Drag to scrub through mission time and inspect live drone positions.",
        )
        st.plotly_chart(
            _build_plotly_map(primary, others, report, buffer_val, t_slider),
            use_container_width=True,
            config={"displayModeBar": True},
        )
        return t_slider

else:

    def _scrubber_fragment(primary: DroneMission, others: list[DroneMission], report, buffer_val: float) -> float:
        map_placeholder = st.empty()
        t_start_all = min([primary.departure_time] + [d.departure_time for d in others])
        t_end_all = max([primary.arrival_time] + [d.arrival_time for d in others])
        t_slider = st.slider(
            "Time Scrubber",
            min_value=float(t_start_all),
            max_value=float(t_end_all),
            value=float(t_start_all),
            step=0.1,
            format="%.1f s",
            help="Drag to scrub through mission time and inspect live drone positions.",
        )
        with map_placeholder:
            st.plotly_chart(
                _build_plotly_map(primary, others, report, buffer_val, t_slider),
                use_container_width=True,
                config={"displayModeBar": True},
            )
        return t_slider


# ---------------------------------------------------------------------------
# Sidebar
# ---------------------------------------------------------------------------
if "result" not in st.session_state:
    st.session_state["result"] = None
if "pending_sound" not in st.session_state:
    st.session_state["pending_sound"] = None
if "tutorial_shown" not in st.session_state:
    st.session_state["tutorial_shown"] = False
if "seen_onboarding" not in st.session_state:
    st.session_state["seen_onboarding"] = False

with st.sidebar:
    st.markdown("## UAV Deconfliction")
    st.markdown("---")

    mode = st.radio(
        "View Mode",
        ["Basic", "Advanced"],
        help="Basic hides technical details. Advanced reveals raw conflict data and analytics.",
    )
    enable_audio = st.toggle("Audio cues", value=False, help="Play short cues after checks.")
    if enable_audio and st.button("Test conflict sound", use_container_width=True):
        _play_sound("conflict")

    st.markdown("---")
    source = st.radio("Data Source", ["Preset Scenario", "Upload JSON"])

    error_msg = None
    if source == "Preset Scenario":
        scenario_key = st.selectbox(
            "Choose Scenario",
            list(SCENARIOS.keys()),
            format_func=lambda k: k.replace("_", " ").title(),
            help="Each scenario is a pre-built airspace situation to quickly test safety behavior.",
        )
        st.caption(f"_{SCENARIO_DESCRIPTIONS.get(scenario_key, '')}_")
        buffer_val = st.slider(
            "Safety Buffer (m)",
            1.0,
            50.0,
            5.0,
            0.5,
            help=(
                "Minimum safe distance in meters between any two drones. "
                "Typical: 5 m for small UAVs, larger for cargo/urban missions."
            ),
        )
        if st.button("Run Deconfliction Check", use_container_width=True):
            try:
                primary, others = SCENARIOS[scenario_key]()
                st.session_state["result"] = _run_check(primary, others, buffer_val)
                report = st.session_state["result"][2]
                st.session_state["pending_sound"] = "clear" if report.is_clear() else "conflict"
                st.rerun()
            except Exception as exc:
                st.session_state["result"] = None
                error_msg = str(exc)
    else:
        uploaded = st.file_uploader("Upload scenario JSON", type=["json"])
        template = json.dumps(
            {
                "primary": {
                    "drone_id": "PRIMARY",
                    "waypoints": [[0, 0, 50], [100, 0, 50]],
                    "speed": 10.0,
                    "departure_time": 0.0,
                    "mission_end_time": 20.0,
                },
                "others": [
                    {
                        "drone_id": "DRONE-A",
                        "waypoints": [[50, -20, 50], [50, 80, 50]],
                        "speed": 10.0,
                        "departure_time": 0.0,
                    }
                ],
                "safety_buffer": 5.0,
            },
            indent=2,
        )
        st.download_button(
            "Download Example Template",
            template,
            "example_scenario.json",
            "application/json",
            use_container_width=True,
        )

        upload_buffer = st.slider(
            "Safety Buffer (m)",
            1.0,
            50.0,
            5.0,
            0.5,
            help="Fallback buffer if JSON does not include safety_buffer.",
        )
        if uploaded and st.button("Run Deconfliction Check", use_container_width=True):
            try:
                primary, others, parsed_buffer = _parse_uploaded_json(uploaded.read())
                effective_buffer = parsed_buffer if parsed_buffer > 0 else upload_buffer
                st.session_state["result"] = _run_check(primary, others, effective_buffer)
                report = st.session_state["result"][2]
                st.session_state["pending_sound"] = "clear" if report.is_clear() else "conflict"
                st.rerun()
            except Exception as exc:
                st.session_state["result"] = None
                msg = str(exc)
                if "cannot complete" in msg:
                    error_msg = "The drone cannot reach the last waypoint in time. Increase mission duration or shorten the route."
                elif "zero speed" in msg.lower() or "non-zero path" in msg.lower():
                    error_msg = "Speed must be greater than 0 m/s for moving drones."
                elif "waypoints" in msg.lower():
                    error_msg = "Each drone needs at least two waypoints to define a route."
                else:
                    error_msg = f"Could not load scenario: {msg}"

    if st.session_state.get("analysis_ms") is not None:
        st.caption(
            f"⚡ Analysis: {st.session_state['analysis_ms']:.1f} ms | "
            f"{st.session_state.get('result')[2].total_checks if st.session_state.get('result') else 0} segment-pairs checked"
        )

    st.markdown("---")
    history_files = sorted(HISTORY_DIR.glob("*.json"), reverse=True)[:10]
    if history_files:
        with st.expander("Recent Sessions", expanded=False):
            for hist_file in history_files:
                data = json.loads(hist_file.read_text(encoding="utf-8"))
                status = data.get("status", "unknown")
                label = f"{data['primary']['drone_id']} - {status}"
                if st.button(label, key=f"hist_{hist_file.name}", use_container_width=True):
                    p, o, b = _load_history(hist_file)
                    st.session_state["result"] = _run_check(p, o, b)
                    st.session_state["pending_sound"] = "clear" if st.session_state["result"][2].is_clear() else "warning"
                    st.rerun()

    st.markdown("---")
    st.markdown("**Legend**")
    st.markdown(
        """
<div>
  <span class='legend-item'><span class='legend-dot' style='background:#1f77b4'></span>Primary drone</span><br>
  <span class='legend-item'><span class='legend-dot' style='background:#2ecc71'></span>Other drones</span><br>
  <span class='legend-item'><span class='legend-dot' style='background:#f85149'></span>Conflict / collision</span><br>
  <span class='legend-item'><span class='legend-dot' style='background:#e3b341'></span>Near-miss warning</span><br>
  <span class='legend-item'><span class='legend-dot' style='background:#888;border-radius:0'></span>Safety buffer zone</span>
</div>
""",
        unsafe_allow_html=True,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
st.markdown("## UAV Strategic Deconfliction System")
st.caption("Pre-flight safety verification with continuous 4D trajectory analysis (x, y, z + time)")

if not st.session_state["seen_onboarding"]:
    with st.expander("👋 How to use this tool", expanded=True):
        st.markdown(
            """
1. Pick a scenario from the sidebar or upload JSON.
2. Adjust the safety buffer and run the check.
3. Review the summary banner, conflict cards, and resolver suggestion.
            """
        )
        if st.button("Got it", use_container_width=True):
            st.session_state["seen_onboarding"] = True
            st.rerun()

if not st.session_state["tutorial_shown"]:
    st.info("Welcome! Start from Scenario Explorer or use Build Custom Mission to create your own route.")
    st.session_state["tutorial_shown"] = True

if enable_audio and st.session_state.get("pending_sound"):
    _play_sound(st.session_state["pending_sound"])
    st.session_state["pending_sound"] = None

if error_msg:
    st.error(error_msg)

result = st.session_state.get("result")

tab1, tab2 = st.tabs(["Scenario Explorer", "Build Custom Mission"])

with tab1:
    if result is None:
        st.markdown("### Get Started")
        c1, c2, c3 = st.columns(3)
        c1.info("1. Choose a scenario in the sidebar or upload JSON.")
        c2.info("2. Adjust the safety buffer.")
        c3.info("3. Run the check to inspect risks and fixes.")

        st.markdown("#### Quick Demo Scenarios")
        quick_cols = st.columns(4)
        demos = [
            ("perpendicular_collision", "Head-On Collision"),
            ("multi_drone_busy", "Busy Airspace"),
            ("clear_parallel", "Parallel Safe"),
            ("altitude_separation_4d", "Altitude Separation"),
        ]
        for col, (key, label) in zip(quick_cols, demos):
            with col:
                if st.button(label, key=f"demo_{key}", use_container_width=True):
                    p, o = SCENARIOS[key]()
                    st.session_state["result"] = _run_check(p, o, 5.0)
                    st.session_state["pending_sound"] = "clear" if st.session_state["result"][2].is_clear() else "conflict"
                    st.rerun()
    else:
        primary, others, report, buffer_val, resolver_result = result
        real_conflicts = [c for c in report.conflicts if c.severity != "near_miss"]
        near_misses = [c for c in report.conflicts if c.severity == "near_miss"]

        with st.container():
            if real_conflicts:
                st.error(
                    f"🚨 CONFLICT DETECTED — {len(real_conflicts)} conflict(s) require immediate action"
                )
            elif near_misses:
                st.warning(
                    f"⚠️ NEAR-MISS — {len(near_misses)} event(s) are close to the safety boundary"
                )
            else:
                st.success("✅ MISSION CLEAR — Safe to execute")

        summary_sentence = _make_summary_sentence(report, buffer_val)
        if summary_sentence:
            if real_conflicts:
                st.error(summary_sentence)
            elif near_misses:
                st.warning(summary_sentence)
            else:
                st.info(summary_sentence)

        elapsed_ms = st.session_state.get("analysis_ms")
        if elapsed_ms is not None:
            st.caption(
                f"⚡ Analysis: {elapsed_ms:.1f} ms | {report.total_checks} segment-pairs checked"
            )

        if report.conflicts:
            st.caption("Quick conflict filters")
            q1, q2, q3 = st.columns([1, 1, 1])
            with q1:
                if st.button(
                    "Collision-only",
                    key="quick_filter_collision",
                    use_container_width=True,
                    disabled=not any(c.severity in ("collision", "buffer_breach") for c in report.conflicts),
                ):
                    st.session_state["conflict_filter"] = "Collision/Buffer Breach"
                    st.rerun()
            with q2:
                if st.button(
                    "Near-miss-only",
                    key="quick_filter_nearmiss",
                    use_container_width=True,
                    disabled=not any(c.severity == "near_miss" for c in report.conflicts),
                ):
                    st.session_state["conflict_filter"] = "Near-Miss"
                    st.rerun()
            with q3:
                if st.button("Show all", key="quick_filter_all", use_container_width=True):
                    st.session_state["conflict_filter"] = "All"
                    st.rerun()

        m1, m2, m3, m4 = st.columns(4)
        m1.metric("Total Drones", len(others) + 1)
        m2.metric("Conflicts", len(real_conflicts), delta="action needed" if real_conflicts else None, delta_color="inverse")
        m3.metric(
            "Nearest Pass",
            f"{min((c.distance for c in report.conflicts), default=float('inf')):.1f} m" if report.conflicts else "-",
        )
        m4.metric("Safety Buffer", f"{buffer_val:.1f} m")

        left_col, right_col = st.columns([3, 2])
        with left_col:
            _scrubber_fragment(primary, others, report, buffer_val)

            st.markdown("#### Mission Replay")
            r1, r2, r3 = st.columns([1, 1, 1])
            replay_step = r1.number_input("Step (s)", min_value=0.1, max_value=2.0, value=0.2, step=0.1)
            replay_speed = r2.number_input("Speed (x)", min_value=0.5, max_value=5.0, value=1.5, step=0.5)
            replay_start = r3.button("Play", use_container_width=True)

            if replay_start:
                t_start_all = min([primary.departure_time] + [d.departure_time for d in others])
                t_end_all = max([primary.arrival_time] + [d.arrival_time for d in others])
                replay_placeholder = st.empty()
                audio_armed = True
                t_now = float(t_start_all)
                while t_now <= float(t_end_all) + 1e-9:
                    replay_placeholder.plotly_chart(
                        _build_plotly_map(primary, others, report, buffer_val, t_now),
                        use_container_width=True,
                        config={"displayModeBar": False},
                    )
                    live_now = _is_conflict_live(report, t_now)
                    if enable_audio and live_now and audio_armed:
                        _play_sound("conflict")
                        audio_armed = False
                    if not live_now:
                        audio_armed = True
                    time.sleep(max(0.02, replay_step / max(replay_speed, 0.1)))
                    t_now = round(t_now + replay_step, 6)

            if mode == "Advanced":
                st.plotly_chart(_build_time_series(primary, others, buffer_val), use_container_width=True)
                show_3d_legend = st.toggle(
                    "Show 3D legend",
                    value=True,
                    key="show_3d_legend",
                    help="Toggle legend visibility for the 3D airspace panel.",
                )
                fig3d = build_3d_plotly(primary, others, report, buffer_val)
                fig3d.update_layout(showlegend=show_3d_legend)
                st.plotly_chart(fig3d, use_container_width=True)

            if resolver_result and resolver_result.get("resolved"):
                st.markdown("#### Resolved Mission Preview")
                resolved_primary = _shifted_mission(primary, float(resolver_result["offset"]))
                resolved_report = check_mission(resolved_primary, others, buffer_val)
                st.plotly_chart(
                    _build_plotly_map(resolved_primary, others, resolved_report, buffer_val),
                    use_container_width=True,
                    config={"displayModeBar": False},
                )

        with right_col:
            st.markdown("#### Mission Summary")
            summary = {
                "Primary drone": primary.drone_id,
                "Speed": f"{primary.speed:.1f} m/s",
                "Departure": f"T + {primary.departure_time:.1f}s",
                "Waypoints": len(primary.waypoints),
                "Est. arrival": f"T + {primary.arrival_time:.1f}s",
            }
            if mode == "Advanced":
                summary["Path length"] = f"{primary.total_path_length():.1f} m"
                summary["Temporal candidates"] = report.total_checks
                summary["AABB pruned"] = report.broad_phase_prunes
            for k, v in summary.items():
                cols = st.columns([2, 3])
                cols[0].markdown(f"**{k}**")
                cols[1].markdown(str(v))

            st.markdown("---")
            st.markdown("#### What-If Delay")
            delay_what_if = st.slider(
                "Shift departure by (s)",
                0.0,
                60.0,
                0.0,
                0.5,
                help="Slide to test how delaying departure affects conflicts. Does not save.",
                key="delay_what_if",
            )
            if delay_what_if > 0:
                shifted = _shifted_mission(primary, delay_what_if)
                wi_report = check_mission(shifted, others, buffer_val)
                if wi_report.is_clear():
                    st.success(f"✅ Delay of {delay_what_if:.1f}s clears all conflicts!")
                else:
                    st.warning(f"Still {len(wi_report.conflicts)} conflict(s) at +{delay_what_if:.1f}s delay.")

            if mode == "Advanced":
                st.markdown("---")
                st.markdown("#### Performance Benchmark")
                benchmark_df = pd.DataFrame(BENCHMARK_POINTS).set_index("drone_count")
                st.line_chart(benchmark_df[["latency_ms"]])
                st.caption("Precomputed latency growth for representative fleet sizes.")

            st.markdown("---")
            if report.conflicts:
                st.markdown("#### Conflict Details")
                conflict_filter = st.radio(
                    "Conflict Filter",
                    ["All", "Collision/Buffer Breach", "Near-Miss"],
                    horizontal=True,
                    key="conflict_filter",
                    help="Filter conflict cards for evaluator demos.",
                )

                if conflict_filter == "Collision/Buffer Breach":
                    visible_conflicts = [
                        c for c in report.conflicts if c.severity in ("collision", "buffer_breach")
                    ]
                elif conflict_filter == "Near-Miss":
                    visible_conflicts = [c for c in report.conflicts if c.severity == "near_miss"]
                else:
                    visible_conflicts = list(report.conflicts)

                if not visible_conflicts:
                    st.info("No conflicts match the selected filter.")

                for c in visible_conflicts:
                    is_nm = c.severity == "near_miss"
                    card_class = "nearmiss-card" if is_nm else "conflict-card"
                    title_class = "card-title-nm" if is_nm else "card-title"
                    title = "Near-Miss Warning" if is_nm else c.severity.replace("_", " ").title()

                    body, resolve_html = _plain_english_explanation(c, buffer_val, resolver_result)
                    if mode == "Advanced":
                        body += (
                            f"<br><small style='color:#8b949e'>"
                            f"CPA t={c.time_of_conflict:.4f}s | d={c.distance:.4f}m | "
                            f"Seg {c.primary_segment_index} vs {c.other_segment_index}</small>"
                        )

                    st.markdown(
                        f'<div class="{card_class}"><div class="{title_class}">{title} - {c.other_drone_id}</div>'
                        f'<div class="card-body">{body}</div>{resolve_html}</div>',
                        unsafe_allow_html=True,
                    )
            else:
                st.success("No conflicts detected. All drone pairs maintain safe separation.")

            if real_conflicts:
                st.markdown("---")
                st.markdown("#### Conflict Resolver")
                with st.expander("Find minimum delay to clear all conflicts", expanded=True):
                    if resolver_result and resolver_result.get("resolved"):
                        st.success(
                            f"Delay departure by {resolver_result['offset']:.1f}s -> new departure T + {resolver_result['new_departure']:.1f}s\n\n"
                            f"{resolver_result['checks_performed']} time slots tested"
                        )
                    elif resolver_result:
                        st.warning(resolver_result.get("reason", "No clear slot found within search range."))

            st.markdown("---")
            report_text = _export_text_report(primary, others, report, buffer_val, resolver_result)
            st.download_button(
                "Export Report (.txt)",
                report_text,
                file_name=f"deconfliction_report_{primary.drone_id}.txt",
                mime="text/plain",
                use_container_width=True,
            )

            export_fig = _build_plotly_map(primary, others, report, buffer_val)
            map_png_bytes, map_export_error = _build_map_png_bytes(export_fig)
            if map_png_bytes:
                st.download_button(
                    "Export Map Snapshot (.png)",
                    map_png_bytes,
                    file_name=f"map_snapshot_{primary.drone_id}.png",
                    mime="image/png",
                    use_container_width=True,
                )
            elif map_export_error:
                st.caption(map_export_error)

            pdf_bytes, pdf_error = _build_pdf_report_bytes(
                primary,
                others,
                report,
                buffer_val,
                resolver_result,
                map_png_bytes=map_png_bytes,
            )
            if pdf_bytes:
                st.download_button(
                    "Export Report (.pdf)",
                    pdf_bytes,
                    file_name=f"deconfliction_report_{primary.drone_id}.pdf",
                    mime="application/pdf",
                    use_container_width=True,
                )
            elif pdf_error:
                st.caption(pdf_error)

            scenario_json = json.dumps(
                {
                    "primary": {
                        "drone_id": primary.drone_id,
                        "waypoints": [[w.x, w.y, w.z] for w in primary.waypoints],
                        "speed": primary.speed,
                        "departure_time": primary.departure_time,
                        "mission_end_time": primary.mission_end_time,
                    },
                    "others": [
                        {
                            "drone_id": d.drone_id,
                            "waypoints": [[w.x, w.y, w.z] for w in d.waypoints],
                            "speed": d.speed,
                            "departure_time": d.departure_time,
                            "mission_end_time": d.mission_end_time,
                        }
                        for d in others
                    ],
                    "safety_buffer": buffer_val,
                },
                indent=2,
            )
            st.download_button(
                "Export Scenario (.json)",
                scenario_json,
                file_name=f"scenario_{primary.drone_id}.json",
                mime="application/json",
                use_container_width=True,
            )

        if mode == "Advanced" and report.conflicts:
            st.markdown("---")
            st.markdown("#### Raw Conflict Data (Advanced)")
            conflict_filter = st.session_state.get("conflict_filter", "All")
            if conflict_filter == "Collision/Buffer Breach":
                visible_conflicts = [
                    c for c in report.conflicts if c.severity in ("collision", "buffer_breach")
                ]
            elif conflict_filter == "Near-Miss":
                visible_conflicts = [c for c in report.conflicts if c.severity == "near_miss"]
            else:
                visible_conflicts = list(report.conflicts)

            rows = []
            for c in visible_conflicts:
                rows.append(
                    {
                        "Other Drone": c.other_drone_id,
                        "Severity": c.severity,
                        "CPA Time (s)": round(c.time_of_conflict, 4),
                        "Distance (m)": round(c.distance, 4),
                        "X (m)": round(c.primary_location.x, 2),
                        "Y (m)": round(c.primary_location.y, 2),
                        "Z (m)": round(c.primary_location.z, 2),
                        "Buffer Entry": round(c.t_entry, 3) if c.t_entry is not None else "N/A",
                        "Buffer Exit": round(c.t_exit, 3) if c.t_exit is not None else "N/A",
                    }
                )
            if rows:
                st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)
            else:
                st.info("No raw rows for the selected conflict filter.")

with tab2:
    st.markdown("### Build Your Own Mission")
    st.caption("Define the primary drone route and validate it against selectable traffic presets.")

    builder_buffer = st.slider(
        "Builder Safety Buffer (m)",
        1.0,
        50.0,
        5.0,
        0.5,
        key="builder_buffer",
        help="Minimum distance maintained between your custom drone and selected traffic.",
    )

    c_speed, c_dep, c_end = st.columns(3)
    custom_speed = c_speed.number_input("Speed (m/s)", 1.0, 50.0, 10.0, key="builder_speed")
    custom_dep = c_dep.number_input("Departure time (s)", 0.0, 300.0, 0.0, key="builder_dep")
    custom_end = c_end.number_input("Mission window end (s)", 1.0, 600.0, 30.0, key="builder_end")

    n_wp = st.number_input("Number of waypoints", 2, 8, 3, key="builder_nwp")
    wp_rows = []
    for i in range(int(n_wp)):
        c1, c2, c3 = st.columns(3)
        x = c1.number_input(f"WP{i+1} X (m)", key=f"builder_x_{i}", value=float(i * 50))
        y = c2.number_input(f"WP{i+1} Y (m)", key=f"builder_y_{i}", value=0.0)
        z = c3.number_input(f"WP{i+1} Alt (m)", key=f"builder_z_{i}", value=50.0)
        wp_rows.append({"X": x, "Y": y, "Alt": z})

    st.markdown("#### Edit Waypoints as Table (What-if Sandbox)")
    st.caption("Edit coordinates directly; route preview updates instantly.")
    edited = st.data_editor(
        pd.DataFrame(wp_rows),
        num_rows="dynamic",
        use_container_width=True,
        key="builder_editor",
    )

    edited_df = edited.copy()
    required_cols = ["X", "Y", "Alt"]
    custom_wps: list[Waypoint] = []
    parse_error: Optional[str] = None
    if not all(col in edited_df.columns for col in required_cols):
        parse_error = "Waypoint table is missing required columns: X, Y, Alt."
    else:
        for col in required_cols:
            edited_df[col] = pd.to_numeric(edited_df[col], errors="coerce")

        invalid_rows = edited_df[required_cols].isna().any(axis=1)
        if invalid_rows.any():
            parse_error = (
                f"Ignored {int(invalid_rows.sum())} invalid row(s). "
                "Fill X, Y, Alt with numeric values."
            )

        valid_df = edited_df.loc[~invalid_rows, required_cols]
        custom_wps = [Waypoint(float(row.X), float(row.Y), float(row.Alt)) for row in valid_df.itertuples(index=False)]

    if parse_error:
        st.warning(parse_error)
    if len(custom_wps) < 2:
        st.info("Add at least two valid waypoints to build a mission route.")

    if not edited_df.empty and all(col in edited_df.columns for col in required_cols):
        st.markdown("#### Validation Preview")
        st.caption("Rows flagged in red contain missing or non-numeric waypoint values.")
        preview_valid = edited_df.copy()
        for col in required_cols:
            preview_valid[col] = pd.to_numeric(preview_valid[col], errors="coerce")
        invalid_mask_preview = preview_valid[required_cols].isna()
        st.dataframe(
            _style_waypoint_preview(preview_valid[required_cols], invalid_mask_preview),
            use_container_width=True,
            hide_index=True,
        )

    traffic_preset = st.selectbox(
        "Traffic preset",
        list(SCENARIOS.keys()),
        key="builder_traffic",
        help="Select existing background traffic to check your custom primary route against.",
    )

    st.markdown("#### Live Route Preview")
    try:
        if len(custom_wps) >= 2:
            preview_primary = DroneMission(
                drone_id="MY-DRONE",
                waypoints=custom_wps,
                speed=float(custom_speed),
                departure_time=float(custom_dep),
                mission_end_time=float(custom_end),
            )
            preview_report = type("PreviewReport", (), {"conflicts": tuple()})()
            st.plotly_chart(
                _build_plotly_map(preview_primary, [], preview_report, builder_buffer, t_slider=None),
                use_container_width=True,
                config={"displayModeBar": False},
            )
    except Exception as exc:
        st.info(f"Preview not available yet: {exc}")

    if st.button(
        "Check My Mission",
        key="builder_submit",
        use_container_width=True,
        disabled=len(custom_wps) < 2,
    ):
        try:
            _, traffic_others = SCENARIOS[traffic_preset]()
            custom_primary = DroneMission(
                drone_id="MY-DRONE",
                waypoints=custom_wps,
                speed=float(custom_speed),
                departure_time=float(custom_dep),
                mission_end_time=float(custom_end),
            )
            st.session_state["result"] = _run_check(custom_primary, traffic_others, builder_buffer)
            report = st.session_state["result"][2]
            st.session_state["pending_sound"] = "clear" if report.is_clear() else "conflict"
            st.success("Mission evaluated. Open Scenario Explorer tab to inspect full results.")
            st.rerun()
        except ValueError as exc:
            msg = str(exc)
            if "cannot complete" in msg:
                st.error("Your drone cannot reach the last waypoint in time. Increase mission window or shorten the route.")
            elif "zero speed" in msg.lower():
                st.error("Speed must be greater than 0.")
            else:
                st.error(f"{msg}")
        except Exception as exc:
            st.error(f"Unexpected builder error: {exc}")

st.markdown("---")
st.caption(
    "UAV Deconfliction System · Analytical CPA engine · 4D space-time conflict detection · "
    "Continuous trajectory analysis (no discrete time stepping)"
)
