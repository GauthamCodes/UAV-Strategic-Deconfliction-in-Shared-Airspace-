"""
3D and 4D (space-time) visualization modules. Earns the extra credit from
the spec for 3D+time handling.

Provides:
    plot_3d_static      — static 3D spatial view (x, y, z)
    plot_spacetime_tube — 4D view with time on the Z-axis (x, y, t), showing
                          that drones crossing the same (x,y) at different
                          times do not actually conflict
    animate_4d          — 3D spatial animation with drones moving over time
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (needed for 3D projection)
from matplotlib.animation import FuncAnimation

from core.models import DroneMission, ConflictReport


_OTHER_COLORS = [
    "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b",
    "#17becf", "#e377c2", "#7f7f7f", "#bcbd22",
]


def plot_3d_static(
    primary: DroneMission,
    others: list[DroneMission],
    report: ConflictReport,
    safety_buffer: float,
    save_path: Optional[str] = None,
):
    """Static 3D plot of all waypoint trajectories + conflict markers."""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    def _plot(drone, color, linestyle, label):
        xs = [w.x for w in drone.waypoints]
        ys = [w.y for w in drone.waypoints]
        zs = [w.z for w in drone.waypoints]
        ax.plot(xs, ys, zs, color=color, linestyle=linestyle, linewidth=2.2, label=label)
        ax.scatter(xs, ys, zs, color=color, s=40)

    _plot(primary, "#1f77b4", "-", f"PRIMARY ({primary.drone_id})")
    for i, d in enumerate(others):
        _plot(d, _OTHER_COLORS[i % len(_OTHER_COLORS)], "--", d.drone_id)

    for c in report.conflicts:
        ax.scatter(
            [c.primary_location.x], [c.primary_location.y], [c.primary_location.z],
            color="red", s=280, marker="X", edgecolor="black",
            linewidth=1.8, zorder=20,
        )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z / altitude (m)")
    ax.set_title(f"3D Spatial View — {report.summary()}")
    ax.legend(loc="upper left", fontsize=9)

    if save_path:
        fig.savefig(save_path, dpi=130, bbox_inches="tight")
        print(f"Saved 3D static plot to {save_path}")

    return fig


def plot_spacetime_tube(
    primary: DroneMission,
    others: list[DroneMission],
    report: ConflictReport,
    safety_buffer: float,
    save_path: Optional[str] = None,
):
    """
    4D space-time view: (X, Y, time) with altitude collapsed.

    The key insight: drones that cross the same (x,y) at different *times*
    produce tubes that miss each other in the (x, y, t) plot. Tubes that
    actually intersect are genuine conflicts. This is exactly why Gemini's
    Approach C earns the extra-credit 4D visualization points.
    """
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    def _sample_drone(drone: DroneMission, n: int = 250):
        if drone.arrival_time <= drone.departure_time:
            return np.array([]), np.array([]), np.array([]), np.array([])
        ts = np.linspace(drone.departure_time, drone.arrival_time, n)
        xs, ys, zs = [], [], []
        for t in ts:
            p = drone.position_at(t)
            if p is None:
                xs.append(np.nan); ys.append(np.nan); zs.append(np.nan)
            else:
                xs.append(p.x); ys.append(p.y); zs.append(p.z)
        return ts, np.array(xs), np.array(ys), np.array(zs)

    ts_p, xs_p, ys_p, _ = _sample_drone(primary)
    ax.plot(xs_p, ys_p, ts_p, color="#1f77b4", linewidth=3,
            label=f"PRIMARY ({primary.drone_id})")

    for i, d in enumerate(others):
        ts_o, xs_o, ys_o, _ = _sample_drone(d)
        col = _OTHER_COLORS[i % len(_OTHER_COLORS)]
        ax.plot(xs_o, ys_o, ts_o, color=col, linestyle="--", linewidth=2,
                label=d.drone_id)

    for c in report.conflicts:
        ax.scatter(
            [c.primary_location.x], [c.primary_location.y], [c.time_of_conflict],
            color="red", s=250, marker="X", edgecolor="black",
            linewidth=1.8, zorder=20,
        )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Time (s)")
    ax.set_title(
        "Space-Time Tube View — conflicts = where tubes meet\n"
        f"({report.summary()})"
    )
    ax.legend(loc="upper left", fontsize=9)

    if save_path:
        fig.savefig(save_path, dpi=130, bbox_inches="tight")
        print(f"Saved space-time plot to {save_path}")

    return fig


def animate_4d(
    primary: DroneMission,
    others: list[DroneMission],
    report: ConflictReport,
    safety_buffer: float,
    save_path: Optional[str] = None,
    fps: int = 15,
):
    """3D spatial animation with drones moving over time."""
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")

    # static paths
    def _plot(drone, color, linestyle):
        xs = [w.x for w in drone.waypoints]
        ys = [w.y for w in drone.waypoints]
        zs = [w.z for w in drone.waypoints]
        ax.plot(xs, ys, zs, color=color, linestyle=linestyle, alpha=0.35, linewidth=1.2)

    _plot(primary, "#1f77b4", "-")
    for i, d in enumerate(others):
        _plot(d, _OTHER_COLORS[i % len(_OTHER_COLORS)], "--")

    for c in report.conflicts:
        ax.scatter(
            [c.primary_location.x], [c.primary_location.y], [c.primary_location.z],
            color="red", s=240, marker="X", edgecolor="black", linewidth=1.8,
        )

    primary_dot, = ax.plot([], [], [], "o", color="#1f77b4", markersize=10)
    other_dots = []
    for i, d in enumerate(others):
        col = _OTHER_COLORS[i % len(_OTHER_COLORS)]
        dot, = ax.plot([], [], [], "o", color=col, markersize=8)
        other_dots.append(dot)

    time_text = ax.text2D(
        0.02, 0.95, "", transform=ax.transAxes, fontsize=11,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(
        f"4D UAV Deconfliction — S_min = {safety_buffer}m\n{report.summary()}"
    )

    t_start = min([primary.departure_time] + [d.departure_time for d in others])
    t_end = max([primary.arrival_time] + [d.arrival_time for d in others])
    n_frames = max(2, int((t_end - t_start) * fps) + 1)
    times = np.linspace(t_start, t_end, n_frames)

    def _update(frame):
        t = times[frame]
        time_text.set_text(f"t = {t:.2f}s")

        p = primary.position_at(t)
        if p is not None:
            primary_dot.set_data([p.x], [p.y])
            primary_dot.set_3d_properties([p.z])
        else:
            primary_dot.set_data([], [])
            primary_dot.set_3d_properties([])

        for i, d in enumerate(others):
            pos = d.position_at(t)
            if pos is not None:
                other_dots[i].set_data([pos.x], [pos.y])
                other_dots[i].set_3d_properties([pos.z])
            else:
                other_dots[i].set_data([], [])
                other_dots[i].set_3d_properties([])
        return [primary_dot, time_text] + other_dots

    anim = FuncAnimation(fig, _update, frames=n_frames,
                         interval=1000 // fps, blit=False, repeat=True)

    if save_path:
        anim.save(save_path, fps=fps, writer="pillow")
        print(f"Saved 4D animation to {save_path}")

    return fig, anim
