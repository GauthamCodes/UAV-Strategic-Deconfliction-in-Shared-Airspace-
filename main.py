"""
CLI entry point for the UAV deconfliction service.

Usage examples
--------------
    # Run the default scenario with a 5 m buffer
    python main.py

    # Run a specific scenario and show plots
    python main.py --scenario perpendicular_collision --visualize

    # Load from a JSON scenario file
    python main.py --file scenarios/my_scenario.json --visualize

    # Save animations to disk (for the demo video)
    python main.py --scenario multi_drone_busy --save-2d anim.gif --save-4d anim4d.gif
"""
from __future__ import annotations

import argparse
import sys

from core.deconfliction import check_mission
from core.models import DroneMission
from core.resolver import find_safe_departure_offset, _shifted_mission
from data.scenarios import SCENARIOS
from data.loader import load_scenario


def _banner(text: str, char: str = "=", width: int = 72) -> None:
    print(char * width)
    print(text)
    print(char * width)


def print_report(report) -> None:
    _banner(f"STATUS: {report.status.upper()}")
    print(f"  Primary drone   : {report.primary_drone_id}")
    print(f"  Safety buffer   : {report.safety_buffer} m")
    print(f"  Temporal candidates checked: {report.total_checks} "
          f"(broad-phase pruned {report.broad_phase_prunes})")
    print()

    if not report.conflicts:
        print("  [OK] No conflicts detected. Mission is safe to execute.")
    else:
        print(f"  {len(report.conflicts)} conflict(s) found:")
        for i, c in enumerate(report.conflicts, 1):
            print(f"    [{i}] {c.explain()}")
            if c.t_entry is not None and c.t_exit is not None:
                print(
                    f"         Buffer breach : {c.t_entry:.3f}s -> "
                    f"{c.t_exit:.3f}s (duration {c.t_exit - c.t_entry:.3f}s)"
                )
            else:
                print(
                    f"         Temporal overlap: "
                    f"{c.overlap_start:.2f}s to {c.overlap_end:.2f}s"
                )
    print("=" * 72)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="uav_deconfliction",
        description="UAV strategic deconfliction — pre-flight safety check "
                    "against shared airspace.",
    )
    src = parser.add_mutually_exclusive_group()
    src.add_argument(
        "--scenario",
        choices=list(SCENARIOS.keys()),
        default="multi_drone_busy",
        help="Built-in scenario to run.",
    )
    src.add_argument("--file", type=str, help="Load a scenario from a JSON file.")
    parser.add_argument("--buffer", type=float, default=5.0,
                        help="Safety buffer in meters (default 5.0).")
    parser.add_argument("--include-near-misses", action="store_true",
                        help="Also report passes within 1.5x the buffer.")
    parser.add_argument("--visualize", action="store_true",
                        help="Open matplotlib windows showing 2D + 3D views.")
    parser.add_argument("--pygame", action="store_true",
                        help="Launch interactive Pygame viewer.")
    parser.add_argument("--resolve", action="store_true",
                        help="Run departure-time resolver when conflicts are detected.")
    parser.add_argument("--resolve-max-delay", type=float, default=300.0,
                        help="Maximum delay (seconds) searched by resolver.")
    parser.add_argument("--resolve-step", type=float, default=0.25,
                        help="Resolver search step in seconds (default 0.25).")
    parser.add_argument("--save-2d", type=str, help="Save 2D animation to this path (e.g. out.gif).")
    parser.add_argument("--save-4d", type=str, help="Save 4D animation to this path.")
    parser.add_argument("--save-static", type=str, help="Save a static 2D plot to this path.")
    parser.add_argument("--save-spacetime", type=str, help="Save space-time tube plot.")
    args = parser.parse_args(argv)

    # 1. Load the scenario
    if args.file:
        primary, others, buffer_override = load_scenario(args.file)
        buffer_value = buffer_override
        print(f"Loaded scenario from {args.file!r}")
    else:
        primary, others = SCENARIOS[args.scenario]()
        buffer_value = args.buffer
        print(f"Running built-in scenario: {args.scenario!r}")

    print(
        f"Primary: {primary.drone_id} | "
        f"{len(primary.waypoints)} waypoints | "
        f"speed = {primary.speed} m/s | "
        f"window = [{primary.departure_time}, {primary.mission_end_time}]"
    )
    print(f"Other drones: {len(others)}")
    print()

    # 2. Run the deconfliction check
    report = check_mission(
        primary,
        others,
        safety_buffer=buffer_value,
        include_near_misses=args.include_near_misses,
    )
    print_report(report)

    resolver_cleared = False
    if args.resolve and not report.is_clear():
        print("Running departure-time resolver...")
        result = find_safe_departure_offset(
            primary,
            others,
            safety_buffer=buffer_value,
            max_delay_seconds=args.resolve_max_delay,
            step_seconds=args.resolve_step,
        )
        if result.get("resolved"):
            offset = float(result["offset"])
            new_departure = float(result["new_departure"])
            print(
                f"  [RESOLVED] Delay primary by {offset:.2f}s "
                f"(new departure {new_departure:.2f}s)."
            )
            shifted_primary = _shifted_mission(primary, offset)
            resolved_report = check_mission(
                shifted_primary,
                others,
                safety_buffer=buffer_value,
                include_near_misses=args.include_near_misses,
            )
            print_report(resolved_report)
            resolver_cleared = resolved_report.is_clear()
        else:
            print(f"  [UNRESOLVED] {result.get('reason', 'No solution found.')}")

    if args.pygame:
        from visualization.viz_pygame import run_pygame_visualizer

        run_pygame_visualizer(primary, others, report, buffer_value)

    # 3. Visualize (optional)
    need_viz = args.visualize or any([
        args.save_2d, args.save_4d, args.save_static, args.save_spacetime,
    ])
    if need_viz:
        import matplotlib
        if not args.visualize:
            matplotlib.use("Agg")  # headless when only saving
        from visualization.viz_2d import animate_2d, plot_2d_static
        from visualization.viz_3d import plot_3d_static, plot_spacetime_tube, animate_4d
        import matplotlib.pyplot as plt

        if args.save_static:
            plot_2d_static(primary, others, report, buffer_value, save_path=args.save_static)

        if args.save_spacetime:
            plot_spacetime_tube(primary, others, report, buffer_value, save_path=args.save_spacetime)

        fig2d = anim2d = None
        if args.visualize or args.save_2d:
            fig2d, anim2d = animate_2d(primary, others, report, buffer_value, save_path=args.save_2d)

        fig4d = anim4d = None
        if args.visualize or args.save_4d:
            fig4d, anim4d = animate_4d(primary, others, report, buffer_value, save_path=args.save_4d)

        if args.visualize:
            plot_3d_static(primary, others, report, buffer_value)
            plot_spacetime_tube(primary, others, report, buffer_value)
            plt.show()

    return 0 if (report.is_clear() or resolver_cleared) else 1


if __name__ == "__main__":
    sys.exit(main())
