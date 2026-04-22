"""
Demo script — runs every built-in scenario and saves static plots + animations.

Useful for:
    - shooting the demo video
    - regression-checking every scenario at once

Outputs go to ./demo_output/.
"""
from __future__ import annotations

import os
import matplotlib
matplotlib.use("Agg")  # Headless so this works on CI / remote machines

from core.deconfliction import check_mission
from data.scenarios import SCENARIOS
from visualization.viz_2d import plot_2d_static, animate_2d
from visualization.viz_3d import plot_spacetime_tube


def _safe_name(s: str) -> str:
    return "".join(ch if ch.isalnum() else "_" for ch in s)


def main():
    out_dir = "demo_output"
    os.makedirs(out_dir, exist_ok=True)
    print(f"Writing demo outputs to {out_dir!r}\n")

    for scenario_name, factory in SCENARIOS.items():
        print(f"=== {scenario_name} ===")
        try:
            primary, others = factory()
            report = check_mission(primary, others, safety_buffer=5.0)
            print(f"  {report.summary()}")
            for c in report.conflicts:
                print(f"    - {c.explain()}")

            # Static 2D snapshot
            base = _safe_name(scenario_name)
            plot_2d_static(
                primary, others, report, safety_buffer=5.0,
                save_path=os.path.join(out_dir, f"{base}_2d.png"),
            )

            # Space-time tube view (4D)
            plot_spacetime_tube(
                primary, others, report, safety_buffer=5.0,
                save_path=os.path.join(out_dir, f"{base}_spacetime.png"),
            )

            # GIF animation (commented out due to performance)
            # Uncomment below if you need GIF animations for the video
            # animate_2d(
            #     primary, others, report, safety_buffer=5.0,
            #     save_path=os.path.join(out_dir, f"{base}_anim.gif"),
            #     fps=15,
            # )
        except Exception as exc:
            print(f"  ERROR: scenario '{scenario_name}' failed: {exc}")
        finally:
            import matplotlib.pyplot as plt

            plt.close("all")
            print()

    print("Demo run complete.")


if __name__ == "__main__":
    main()
