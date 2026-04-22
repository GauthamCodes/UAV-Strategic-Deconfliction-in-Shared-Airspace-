"""
Pygame-based 2D visualizer for UAV deconfliction scenarios.

This complements the Matplotlib visualizer with a real-time interactive view.
"""
from __future__ import annotations

from typing import Sequence

import pygame

from core.models import DroneMission, ConflictReport


COLORS = {
    "background": (15, 15, 25),
    "primary": (30, 144, 255),
    "conflict": (220, 20, 60),
    "other": [
        (0, 200, 0),
        (255, 165, 0),
        (148, 0, 211),
        (255, 215, 0),
        (0, 191, 255),
    ],
    "hud_ok": (50, 205, 50),
    "hud_bad": (220, 20, 60),
}


class UAVPygameVisualizer:
    def __init__(
        self,
        primary: DroneMission,
        others: Sequence[DroneMission],
        report: ConflictReport,
        safety_buffer: float,
        width: int = 1000,
        height: int = 800,
        fps: int = 30,
    ) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("UAV Deconfliction - Pygame View")
        self.clock = pygame.time.Clock()

        self.primary = primary
        self.others = list(others)
        self.report = report
        self.safety_buffer = safety_buffer
        self.width = width
        self.height = height
        self.fps = fps

        self._compute_transform()

    def _compute_transform(self, padding: int = 60) -> None:
        all_x = [w.x for d in [self.primary, *self.others] for w in d.waypoints]
        all_y = [w.y for d in [self.primary, *self.others] for w in d.waypoints]

        if not all_x or not all_y:
            self.scale = 1.0
            self.offset_x = 0.0
            self.offset_y = 0.0
            return

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        dx = max(max_x - min_x, 1.0)
        dy = max(max_y - min_y, 1.0)

        self.scale = min((self.width - 2 * padding) / dx, (self.height - 2 * padding) / dy)
        self.offset_x = padding - min_x * self.scale
        self.offset_y = padding - min_y * self.scale

    def _world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        sx = int(x * self.scale + self.offset_x)
        sy = int(y * self.scale + self.offset_y)
        # Flip y so world y+ appears upward on screen.
        return sx, self.height - sy

    def _draw_paths(self) -> None:
        def draw_path(drone: DroneMission, color: tuple[int, int, int], width: int) -> None:
            pts = [self._world_to_screen(w.x, w.y) for w in drone.waypoints]
            if len(pts) > 1:
                pygame.draw.lines(self.screen, color, False, pts, width)
            for pt in pts:
                pygame.draw.circle(self.screen, color, pt, 4)

        draw_path(self.primary, COLORS["primary"], 3)
        for i, drone in enumerate(self.others):
            color = COLORS["other"][i % len(COLORS["other"])]
            draw_path(drone, color, 2)

    def _draw_conflict_markers(self) -> None:
        for conflict in self.report.conflicts:
            x, y = self._world_to_screen(conflict.primary_location.x, conflict.primary_location.y)
            size = 10
            pygame.draw.line(self.screen, COLORS["conflict"], (x - size, y - size), (x + size, y + size), 3)
            pygame.draw.line(self.screen, COLORS["conflict"], (x + size, y - size), (x - size, y + size), 3)

    def _draw_drone(self, drone: DroneMission, color: tuple[int, int, int], t: float, radius: int = 8) -> None:
        pos = drone.position_at(t)
        if pos is None:
            return

        sx, sy = self._world_to_screen(pos.x, pos.y)
        buffer_px = max(1, int(self.safety_buffer * self.scale))
        pygame.draw.circle(self.screen, color, (sx, sy), buffer_px, 1)
        pygame.draw.circle(self.screen, color, (sx, sy), radius)

    def _is_conflict_live(self, t: float, dt: float) -> bool:
        for conflict in self.report.conflicts:
            if conflict.severity not in ("collision", "buffer_breach"):
                continue
            if conflict.t_entry is not None and conflict.t_exit is not None:
                if conflict.t_entry <= t <= conflict.t_exit:
                    return True
            elif abs(conflict.time_of_conflict - t) <= dt:
                return True
        return False

    def _draw_hud(self, t: float, t_start: float, t_end: float) -> None:
        font = pygame.font.SysFont("consolas", 16)
        status = self.report.status.upper()
        status_color = COLORS["hud_bad"] if "CONFLICT" in status else COLORS["hud_ok"]
        text = font.render(f"t={t:.2f}s | {status}", True, status_color)
        self.screen.blit(text, (10, 10))

        progress = (t - t_start) / max(t_end - t_start, 1e-9)
        progress = max(0.0, min(1.0, progress))
        pygame.draw.rect(self.screen, (80, 80, 80), (10, 36, 260, 8))
        pygame.draw.rect(self.screen, status_color, (10, 36, int(260 * progress), 8))

    def run(self) -> None:
        t_start = min([self.primary.departure_time] + [d.departure_time for d in self.others])
        t_end = max([self.primary.arrival_time] + [d.arrival_time for d in self.others])
        t = t_start
        dt = (t_end - t_start) / max(self.fps * 10, 1)

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                    t = t_start

            self.screen.fill(COLORS["background"])
            self._draw_paths()
            self._draw_conflict_markers()

            primary_color = COLORS["conflict"] if self._is_conflict_live(t, dt) else COLORS["primary"]
            self._draw_drone(self.primary, primary_color, t, radius=10)
            for i, drone in enumerate(self.others):
                self._draw_drone(drone, COLORS["other"][i % len(COLORS["other"])], t)

            self._draw_hud(t, t_start, t_end)
            pygame.display.flip()

            t = t_start + ((t - t_start + dt) % max(t_end - t_start, 1e-9))
            self.clock.tick(self.fps)

        pygame.quit()


def run_pygame_visualizer(
    primary: DroneMission,
    others: Sequence[DroneMission],
    report: ConflictReport,
    safety_buffer: float,
    width: int = 1000,
    height: int = 800,
    fps: int = 30,
) -> None:
    """Convenience wrapper to launch the interactive Pygame viewer."""
    viewer = UAVPygameVisualizer(
        primary=primary,
        others=others,
        report=report,
        safety_buffer=safety_buffer,
        width=width,
        height=height,
        fps=fps,
    )
    viewer.run()
