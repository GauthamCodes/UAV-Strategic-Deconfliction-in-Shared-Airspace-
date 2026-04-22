"""
2D top-down animation of a deconfliction run using matplotlib.

Shows:
    - static trajectory lines for all drones
    - moving dots as drones fly
    - safety-buffer circles around each drone
    - red X markers at conflict points, with time-of-conflict labels
    - a live clock and a persistent status banner

Design inspired by the Pygame animation in my A* factory-nav project,
but using matplotlib's FuncAnimation for portability and easier saving
as GIF/MP4 for the demo video.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

from core.models import DroneMission, ConflictReport


_OTHER_COLORS = [
    "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b",
    "#17becf", "#e377c2", "#7f7f7f", "#bcbd22",
]


def _time_window(primary: DroneMission, others: list[DroneMission]) -> tuple[float, float]:
    t_start = min([primary.departure_time] + [d.departure_time for d in others])
    t_end = max([primary.arrival_time] + [d.arrival_time for d in others])
    return t_start, t_end


def _set_bounds(ax, primary: DroneMission, others: list[DroneMission], padding: float = 15.0):
    all_xs, all_ys = [], []
    for drone in [primary, *others]:
        for w in drone.waypoints:
            all_xs.append(w.x)
            all_ys.append(w.y)
    if not all_xs:
        return
    ax.set_xlim(min(all_xs) - padding, max(all_xs) + padding)
    ax.set_ylim(min(all_ys) - padding, max(all_ys) + padding)


def animate_2d(
    primary: DroneMission,
    others: list[DroneMission],
    report: ConflictReport,
    safety_buffer: float,
    save_path: Optional[str] = None,
    fps: int = 20,
    title_suffix: str = "",
):
    """
    Create a 2D top-down animation. Returns (fig, anim).

    If `save_path` is given (e.g. "scenario.gif"), the animation is saved.
    """
    fig, ax = plt.subplots(figsize=(11, 8))

    # ---- static trajectories ----
    def _plot_path(drone, color, linestyle):
        xs = [w.x for w in drone.waypoints]
        ys = [w.y for w in drone.waypoints]
        ax.plot(xs, ys, color=color, linestyle=linestyle, alpha=0.35, linewidth=1.5)
        ax.scatter(xs, ys, color=color, s=25, alpha=0.7, zorder=3)

    _plot_path(primary, "#1f77b4", "-")
    for i, d in enumerate(others):
        _plot_path(d, _OTHER_COLORS[i % len(_OTHER_COLORS)], "--")

    # ---- conflict markers ----
    for c in report.conflicts:
        ax.scatter(
            [c.primary_location.x], [c.primary_location.y],
            color="red", s=250, marker="X", edgecolor="black",
            linewidth=1.5, zorder=10,
        )
        ax.annotate(
            f"t={c.time_of_conflict:.2f}s\nd={c.distance:.1f}m",
            (c.primary_location.x, c.primary_location.y),
            xytext=(10, 10), textcoords="offset points",
            fontsize=8, color="darkred",
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="red", alpha=0.8),
        )

    # ---- dynamic actors ----
    primary_dot, = ax.plot([], [], "o", color="#1f77b4", markersize=13,
                           label=f"PRIMARY ({primary.drone_id})", zorder=5)
    primary_buffer = patches.Circle((0, 0), safety_buffer, fill=False,
                                     edgecolor="#1f77b4", linewidth=1.2, alpha=0.7)
    ax.add_patch(primary_buffer)

    other_dots, other_buffers = [], []
    for i, d in enumerate(others):
        col = _OTHER_COLORS[i % len(_OTHER_COLORS)]
        dot, = ax.plot([], [], "o", color=col, markersize=11, label=d.drone_id, zorder=5)
        other_dots.append(dot)
        buf = patches.Circle((0, 0), safety_buffer, fill=False,
                             edgecolor=col, linewidth=1.0, alpha=0.5)
        ax.add_patch(buf)
        other_buffers.append(buf)

    time_text = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, fontsize=12,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85),
    )
    status_color = "lightcoral" if "conflict" in report.status else "lightgreen"
    ax.text(
        0.98, 0.98, report.summary(),
        transform=ax.transAxes, fontsize=9,
        verticalalignment="top", horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor=status_color, alpha=0.85),
    )

    # ---- axes & style ----
    _set_bounds(ax, primary, others)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(
        f"2D UAV Deconfliction — S_min = {safety_buffer} m{title_suffix}"
    )
    ax.legend(loc="lower left", fontsize=8, framealpha=0.85)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    # ---- animation frames ----
    t_start, t_end = _time_window(primary, others)
    n_frames = max(2, int((t_end - t_start) * fps) + 1)
    times = np.linspace(t_start, t_end, n_frames)

    _OFF = -1e9  # offscreen anchor for inactive drones

    def _update(frame: int):
        t = times[frame]
        time_text.set_text(f"t = {t:.2f}s")

        p = primary.position_at(t)
        if p is not None:
            primary_dot.set_data([p.x], [p.y])
            primary_buffer.center = (p.x, p.y)
        else:
            primary_dot.set_data([], [])
            primary_buffer.center = (_OFF, _OFF)

        for i, d in enumerate(others):
            pos = d.position_at(t)
            if pos is not None:
                other_dots[i].set_data([pos.x], [pos.y])
                other_buffers[i].center = (pos.x, pos.y)
            else:
                other_dots[i].set_data([], [])
                other_buffers[i].center = (_OFF, _OFF)

        # Flash the primary red if a conflict is "live" at this time.
        # Prefer exact breach window [t_entry, t_exit]; fall back to CPA flash
        # only when no breach window is available.
        conflict_live = any(
            (
                c.t_entry is not None
                and c.t_exit is not None
                and c.t_entry <= t <= c.t_exit
            )
            or (
                c.t_entry is None
                and c.t_exit is None
                and abs(c.time_of_conflict - t) < 0.5
            )
            for c in report.conflicts
            if c.severity in ("collision", "buffer_breach")
        )
        primary_dot.set_markerfacecolor("red" if conflict_live else "#1f77b4")

        return [primary_dot, primary_buffer, time_text] + other_dots + other_buffers

    anim = FuncAnimation(
        fig, _update, frames=n_frames, interval=1000 // fps,
        blit=False, repeat=True,
    )

    if save_path:
        anim.save(save_path, fps=fps, writer="pillow")
        print(f"Saved 2D animation to {save_path}")

    return fig, anim


def plot_2d_static(
    primary: DroneMission,
    others: list[DroneMission],
    report: ConflictReport,
    safety_buffer: float,
    save_path: Optional[str] = None,
):
    """Static top-down snapshot (no animation). Useful for the report."""
    fig, ax = plt.subplots(figsize=(11, 8))

    def _plot_path(drone, color, linestyle, label):
        xs = [w.x for w in drone.waypoints]
        ys = [w.y for w in drone.waypoints]
        ax.plot(xs, ys, color=color, linestyle=linestyle, linewidth=2, label=label)
        ax.scatter(xs, ys, color=color, s=35, zorder=3)

    _plot_path(primary, "#1f77b4", "-", f"PRIMARY ({primary.drone_id})")
    for i, d in enumerate(others):
        _plot_path(d, _OTHER_COLORS[i % len(_OTHER_COLORS)], "--", d.drone_id)

    for c in report.conflicts:
        ax.scatter([c.primary_location.x], [c.primary_location.y],
                   color="red", s=300, marker="X", edgecolor="black",
                   linewidth=2, zorder=10)
        ax.annotate(
            f"{c.severity}\nt={c.time_of_conflict:.2f}s\nd={c.distance:.2f}m",
            (c.primary_location.x, c.primary_location.y),
            xytext=(12, 12), textcoords="offset points", fontsize=9,
            color="darkred",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="red"),
        )

    _set_bounds(ax, primary, others)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"2D Trajectory Plan — {report.summary()}")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    if save_path:
        fig.savefig(save_path, dpi=130, bbox_inches="tight")
        print(f"Saved static 2D plot to {save_path}")

    return fig
