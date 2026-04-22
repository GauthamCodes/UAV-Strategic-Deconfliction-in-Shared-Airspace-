"""Data access helpers and built-in scenario registry."""

from .scenarios import SCENARIOS
from .loader import load_scenario, save_scenario

__all__ = ["SCENARIOS", "load_scenario", "save_scenario"]
