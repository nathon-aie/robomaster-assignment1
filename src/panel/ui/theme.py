#!/usr/bin/env python3
"""Colours and fonts for the mission control dashboard.

Dark instrument-panel palette: low-glare background, one accent per meaning,
and hard-to-miss colours for anything safety related.
"""

import pygame

from ..robot_state import RobotStatus

# ------------------------------------------------------------------ palette
BG = (14, 17, 22)
PANEL = (22, 26, 33)
PANEL_ALT = (28, 33, 41)
PANEL_EDGE = (48, 56, 68)
HEADER = (18, 22, 29)

TEXT = (226, 232, 240)
TEXT_DIM = (140, 152, 168)
TEXT_FAINT = (92, 102, 116)

ACCENT = (56, 189, 248)        # cyan - primary interactive
ACCENT_DARK = (14, 116, 144)
OK = (34, 197, 94)
WARN = (250, 204, 21)
DANGER = (239, 68, 68)
DANGER_DARK = (127, 29, 29)
VIOLET = (167, 139, 250)
ORANGE = (251, 146, 60)

# ------------------------------------------------------------- map palette
GRID_LINE = (40, 47, 58)
CELL_UNKNOWN = (26, 30, 38)
CELL_FREE = (46, 55, 68)
CELL_WALL = (96, 106, 122)
CELL_OBSTACLE = (150, 60, 60)
WALL_EDGE = (232, 238, 246)
WALL_EDGE_UNKNOWN = (70, 78, 92)
CELL_START = (22, 101, 52)
CELL_GOAL = (153, 27, 27)
CELL_CHECKPOINT = (113, 63, 18)
CELL_OBJECT = (13, 88, 75)        # square the object to pick up stands on
OBJECT_AIM = (94, 234, 212)       # aim point inside that square
OBJECT_CARRIED = (34, 211, 238)   # the object while the gripper holds it
CELL_PLACE = (76, 29, 149)        # drop-off square for the carried object
PLACE_ARROW = (196, 181, 253)     # facing the object is released at
CELL_FRONTIER = (56, 189, 248)
GHOST_TRUTH = (60, 66, 80)

PATH_PLANNED = (56, 189, 248)
PATH_DONE = (34, 197, 94)
PATH_REMAINING = (125, 211, 252)
TRAIL = (250, 204, 21)
ROBOT = (248, 250, 252)
ROBOT_PLACED = (148, 163, 184)   # ghosted marker for the not-yet-live robot
ROBOT_BODY = (37, 99, 235)
SENSOR_RAY = (56, 189, 248)
SENSOR_HIT = (239, 68, 68)

#: Colour used for each robot status pill.
STATUS_COLORS = {
    RobotStatus.DISCONNECTED: TEXT_FAINT,
    RobotStatus.CONNECTING: WARN,
    RobotStatus.CONNECTED: ACCENT,
    RobotStatus.READY: OK,
    RobotStatus.RUNNING: OK,
    RobotStatus.MOVING: ACCENT,
    RobotStatus.PAUSED: WARN,
    RobotStatus.STOPPED: TEXT_DIM,
    RobotStatus.MAPPING: VIOLET,
    RobotStatus.NAVIGATING: ACCENT,
    RobotStatus.ERROR: DANGER,
    RobotStatus.EMERGENCY_STOP: DANGER,
    RobotStatus.TRACKING_LOST: ORANGE,
}


class Fonts(object):
    """Lazily built font set; falls back to the bundled default face."""

    def __init__(self):
        self.title = self._font(("consolas", "dejavusansmono", "couriernew"), 20, bold=True)
        self.h1 = self._font(("consolas", "dejavusansmono", "couriernew"), 15, bold=True)
        self.body = self._font(("consolas", "dejavusansmono", "couriernew"), 14)
        self.small = self._font(("consolas", "dejavusansmono", "couriernew"), 12)
        self.tiny = self._font(("consolas", "dejavusansmono", "couriernew"), 11)
        self.big = self._font(("consolas", "dejavusansmono", "couriernew"), 26, bold=True)

    @staticmethod
    def _font(names, size, bold=False):
        for name in names:
            try:
                font = pygame.font.SysFont(name, size, bold=bold)
                if font is not None:
                    return font
            except Exception:
                continue
        return pygame.font.Font(None, size + 4)


def status_color(status):
    return STATUS_COLORS.get(status, TEXT_DIM)


def blend(color_a, color_b, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(a + (b - a) * t) for a, b in zip(color_a, color_b))
