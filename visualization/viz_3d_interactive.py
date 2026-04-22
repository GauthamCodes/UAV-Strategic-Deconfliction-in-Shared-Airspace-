"""Interactive 3D Plotly airspace view for the Streamlit dashboard."""
from __future__ import annotations

import plotly.graph_objects as go

from core.models import DroneMission, ConflictReport


def build_3d_plotly(
    primary: DroneMission,
    others: list[DroneMission],
    report: ConflictReport,
    safety_buffer: float,
) -> go.Figure:
    fig = go.Figure()

    def add_3d_path(drone: DroneMission, color: str, name: str) -> None:
        xs = [w.x for w in drone.waypoints]
        ys = [w.y for w in drone.waypoints]
        zs = [w.z for w in drone.waypoints]
        fig.add_trace(
            go.Scatter3d(
                x=xs,
                y=ys,
                z=zs,
                mode="lines+markers",
                name=name,
                line=dict(color=color, width=4),
                marker=dict(size=5, color=color),
                hovertemplate=(
                    f"<b>{name}</b><br>"
                    "x=%{x:.1f}, y=%{y:.1f}, alt=%{z:.0f}m"
                    "<extra></extra>"
                ),
            )
        )

    add_3d_path(primary, "#1f77b4", f"PRIMARY ({primary.drone_id})")
    colors = ["#2ecc71", "#f39c12", "#9b59b6", "#1abc9c", "#e74c3c", "#3498db"]
    for i, drone in enumerate(others):
        add_3d_path(drone, colors[i % len(colors)], drone.drone_id)

    for conflict in report.conflicts:
        fig.add_trace(
            go.Scatter3d(
                x=[conflict.primary_location.x],
                y=[conflict.primary_location.y],
                z=[conflict.primary_location.z],
                mode="markers",
                marker=dict(size=12, color="#f85149", symbol="x"),
                name=f"Conflict @ t={conflict.time_of_conflict:.1f}s",
                hovertemplate=(
                    "<b>CONFLICT</b><br>"
                    f"t={conflict.time_of_conflict:.2f}s<br>"
                    f"sep={conflict.distance:.2f}m<br>"
                    f"buffer={safety_buffer:.2f}m"
                    "<extra></extra>"
                ),
            )
        )

    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0d1117",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Altitude (m)",
            bgcolor="#0d1117",
        ),
        height=480,
        title="3D Airspace View",
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig
