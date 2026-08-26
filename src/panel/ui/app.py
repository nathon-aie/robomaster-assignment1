#!/usr/bin/env python3
"""RoboMaster Mission Control - pygame dashboard.

    +--------------------------------------------------------------+
    | ROBOMASTER MISSION CONTROL      MODE / LINK / STATE  [E-STOP] |
    +-------------+------------------------------+-----------------+
    | MODE        |                              | SENSOR DEBUG    |
    | CONNECTION  |          LIVE MAP            | MAPPING         |
    | RUN CONTROL |                              | MISSION         |
    | ROBOT STATE |                              | EVENT LOG       |
    +-------------+------------------------------+-----------------+
    | SIZE TOOLS / EDIT ROTATE FILE / PLAN TURN VIEW SIM SPEED     |
    +--------------------------------------------------------------+
"""

import os
import sys
import time

import pygame

from ..geometry import DIR_LONG, heading_to_dir
from ..mission import (
    MODE_MOCK,
    MODE_REAL,
    MODE_SIM,
    MissionConfig,
    MissionController,
)
from ..occupancy import OccupancyGrid
from ..robot_state import RobotStatus
from ..simulation import SPEED_STEPS
from . import theme
from .map_view import TOOLS, MapView
from .place_dialog import DELIVERY_TARGET, OBJECT_TARGET, SubCellTargetDialog
from .widgets import Button, Modal, NumberInput, ProgressBar, draw_kv, draw_panel, draw_text

DEFAULT_MAP_PATH = os.path.join("data", "panel_map.json")

#: Cruise-speed presets for the real robot (label, m/s).
ROBOT_SPEEDS = (("SLOW", 0.10), ("NORMAL", 0.15), ("BRISK", 0.22))

HEADER_H = 52
LEFT_W = 296
RIGHT_W = 316
BOTTOM_H = 212
PAD = 10


#: Right-column panels: (preferred height, minimum height).
# The mapping panel needs 108 px for header + progress bar + two count
# lines; below that the bar overlaps the text.
RIGHT_STACK = ((214, 148), (118, 108), (150, 104))
LOG_MIN_H = 92


def _stack_heights(available, gap):
    """Splits the right column between the three panels and the event log.

    Panels take their preferred height when there is room; otherwise they give
    height back - largest surplus first - until the log reaches ``LOG_MIN_H``.
    """
    heights = [pref for pref, _ in RIGHT_STACK]
    usable = available - gap * len(RIGHT_STACK)
    log_h = usable - sum(heights)
    if log_h < LOG_MIN_H:
        deficit = LOG_MIN_H - log_h
        for i, (pref, minimum) in enumerate(RIGHT_STACK):
            if deficit <= 0:
                break
            take = min(pref - minimum, deficit)
            heights[i] -= take
            deficit -= take
        log_h = usable - sum(heights)
    return heights[0], heights[1], heights[2], max(0, log_h)


def fit_to_desktop(size, margin=(80, 120), minimum=(1180, 800)):
    """Shrinks the requested window so it always fits on the actual screen.

    A laptop panel is often smaller than the layout's comfortable size; opening
    larger than the desktop puts the toolbar off-screen where it cannot be
    reached.  Never goes below ``minimum`` - the layout needs that much room.
    """
    width, height = size
    try:
        pygame.display.init()
        info = pygame.display.Info()
        if info.current_w > 0 and info.current_h > 0:
            width = min(width, max(minimum[0], info.current_w - margin[0]))
            height = min(height, max(minimum[1], info.current_h - margin[1]))
    except Exception:
        pass
    return (int(width), int(height))


class _ToolbarRow(object):
    """Flow layout for one toolbar row, grouped under small captions.

    ``group()`` starts a new labelled cluster and drops a separator before it,
    so related controls read as a unit instead of one long strip of buttons.
    """

    GAP = 5
    GROUP_GAP = 16

    def __init__(self, app, x, y, height=26):
        self.app = app
        self.x = x
        self.y = y
        self.height = height
        self._first_group = True

    def group(self, label):
        if not self._first_group:
            self.app.toolbar_separators.append((self.x + self.GROUP_GAP // 2 - 2,
                                                self.y - 12, self.y + self.height))
            self.x += self.GROUP_GAP
        self._first_group = False
        if label:
            self.app.toolbar_labels.append((self.x, self.y - 14, label))

    def claim(self, width):
        """Reserves raw space (for non-Button widgets) and returns its x."""
        x = self.x
        self.x += width + self.GAP
        return x

    def button(self, label, on_click, style="normal", width=None, **kwargs):
        if width is None:
            width = 10 + self.app.fonts.small.size(label)[0] + 16
        button = Button((self.x, self.y, width, self.height), label, on_click, style, **kwargs)
        self.app.buttons.append(button)
        self.x += width + self.GAP
        return button


class MissionControlApp(object):
    MIN_SIZE = (1180, 800)

    def __init__(self, controller=None, map_path=DEFAULT_MAP_PATH, size=(1600, 1000)):
        pygame.init()
        pygame.display.set_caption("RoboMaster Mission Control")
        self.screen = pygame.display.set_mode(fit_to_desktop(size), pygame.RESIZABLE)
        self.fonts = theme.Fonts()
        self.clock = pygame.time.Clock()

        self.controller = controller or MissionController(OccupancyGrid(9, 9), MissionConfig())
        self.map_path = map_path
        self.map_view = MapView((0, 0, 10, 10))
        self.map_view.on_edit = self._on_map_edit
        self.modal = None
        self.place_dialog = None
        self.running = True
        self.status_message = ""
        self.status_until = 0.0
        self.sim_speed_index = 1
        self.robot_speed_index = self._nearest_speed_index(
            self.controller.config.base_speed_mps if controller else 0.15)

        self.widgets = []
        self.buttons = []
        self.toolbar_labels = []
        self.toolbar_separators = []
        self.width_input = NumberInput((0, 0, 56, 26), "WIDTH", self.controller.map.width)
        self.height_input = NumberInput((0, 0, 56, 26), "HEIGHT", self.controller.map.height)
        self.progress_bar = ProgressBar((0, 0, 10, 10), theme.VIOLET)
        self._build_layout()

    # ==================================================================
    # helpers
    # ==================================================================
    def notify(self, text, seconds=4.0):
        self.status_message = text
        self.status_until = time.time() + seconds

    def editable(self):
        return not self.controller.mission_active()

    def connected(self):
        robot = self.controller.robot
        return robot is not None and robot.is_connected()

    def _confirm(self, title, lines, on_confirm, label="CONFIRM", style="danger"):
        self.modal = Modal(title, lines, on_confirm, confirm_label=label, confirm_style=style)

    # ==================================================================
    # layout
    # ==================================================================
    def _build_layout(self):
        width, height = self.screen.get_size()
        self.buttons = []

        # ---- header ---------------------------------------------------
        self.estop_btn = Button(
            (width - 210, 8, 200, HEADER_H - 16), "!! EMERGENCY STOP", self.act_estop,
            "danger", font_key="body",
        )
        self.buttons.append(self.estop_btn)

        left_x = PAD
        y = HEADER_H + PAD

        # ---- mode selector -------------------------------------------
        self.mode_rect = pygame.Rect(left_x, y, LEFT_W, 88)
        mode_y = y + 30
        btn_w = (LEFT_W - 2 * 8 - 12) // 3
        for i, mode in enumerate((MODE_SIM, MODE_REAL, MODE_MOCK)):
            label = {"SIMULATION": "SIM", "REAL ROBOT": "REAL", "MOCK ROBOT": "MOCK"}[mode]
            self.buttons.append(Button(
                (left_x + 8 + i * (btn_w + 6), mode_y, btn_w, 34), label,
                (lambda m=mode: self.act_set_mode(m)),
                "danger" if mode == MODE_REAL else "accent",
                enabled=self.editable,
                active=(lambda m=mode: self.controller.mode == m),
            ))
        y += 88 + PAD

        # ---- connection ----------------------------------------------
        self.conn_rect = pygame.Rect(left_x, y, LEFT_W, 108)
        half = (LEFT_W - 16 - 6) // 2
        self.buttons.append(Button((left_x + 8, y + 30, half, 32), "CONNECT", self.act_connect,
                                   "ok", enabled=lambda: not self.connected()))
        self.buttons.append(Button((left_x + 14 + half, y + 30, half, 32), "DISCONNECT",
                                   self.act_disconnect, "ghost", enabled=self.connected))
        self.buttons.append(Button((left_x + 8, y + 68, half, 32), "ARM", self.act_arm, "warn",
                                   enabled=lambda: self.connected() and not self.controller.armed,
                                   active=lambda: self.controller.armed))
        self.buttons.append(Button((left_x + 14 + half, y + 68, half, 32), "DISARM",
                                   self.act_disarm, "ghost",
                                   enabled=lambda: self.controller.armed))
        y += 108 + PAD

        # ---- run control ---------------------------------------------
        self.run_rect = pygame.Rect(left_x, y, LEFT_W, 158)
        quarter = (LEFT_W - 16 - 18) // 4
        self.buttons.append(Button((left_x + 8, y + 30, quarter, 34), "> RUN", self.act_run, "ok",
                                   enabled=lambda: self.connected() and not self.controller.mission_active()))
        self.buttons.append(Button((left_x + 14 + quarter, y + 30, quarter, 34), "|| PAUSE",
                                   self.act_pause_resume, "warn",
                                   enabled=lambda: self.controller.mission_active()))
        self.buttons.append(Button((left_x + 20 + 2 * quarter, y + 30, quarter, 34), "[] STOP",
                                   self.act_stop, "ghost",
                                   enabled=lambda: self.controller.mission_active()))
        self.buttons.append(Button((left_x + 26 + 3 * quarter, y + 30, quarter, 34), "<> RESTART",
                                   self.act_restart, "ghost", enabled=self.connected))
        self.buttons.append(Button((left_x + 8, y + 70, half, 34), "AUTO MAP", self.act_automap,
                                   "accent", enabled=lambda: not self.controller.mission_active()))
        self.buttons.append(Button((left_x + 14 + half, y + 70, half, 34), "CLEAR E-STOP",
                                   self.act_clear_estop, "danger",
                                   enabled=lambda: self.connected() and self.controller.robot.emergency_stopped()))

        # Robot cruise speed - separate from SIM SPEED, which only scales time.
        speed_w = (LEFT_W - 16 - 12) // 3
        for i, (label, _mps) in enumerate(ROBOT_SPEEDS):
            self.buttons.append(Button(
                (left_x + 8 + i * (speed_w + 6), y + 122, speed_w, 28), label,
                (lambda idx=i: self.act_set_robot_speed(idx)), "normal",
                enabled=self.editable,
                active=(lambda idx=i: self.robot_speed_index == idx),
            ))
        y += 158 + PAD

        self.state_rect = pygame.Rect(left_x, y, LEFT_W, height - BOTTOM_H - y - PAD)

        # ---- right column --------------------------------------------
        # Heights adapt to the window: on a short screen the stacked panels
        # shrink toward their minimums so the event log still gets room,
        # instead of the last panel being squeezed to nothing.
        right_x = width - RIGHT_W - PAD
        ry = HEADER_H + PAD
        available = height - BOTTOM_H - ry - PAD
        sensor_h, mapping_h, mission_h, log_h = _stack_heights(available, PAD)

        self.sensor_rect = pygame.Rect(right_x, ry, RIGHT_W, sensor_h)
        ry += sensor_h + PAD
        self.mapping_rect = pygame.Rect(right_x, ry, RIGHT_W, mapping_h)
        self.progress_bar.rect = pygame.Rect(right_x + 10, ry + 52, RIGHT_W - 20, 14)
        ry += mapping_h + PAD
        self.mission_rect = pygame.Rect(right_x, ry, RIGHT_W, mission_h)
        ry += mission_h + PAD
        self.log_rect = pygame.Rect(right_x, ry, RIGHT_W, log_h)

        # ---- map canvas ----------------------------------------------
        map_x = left_x + LEFT_W + PAD
        self.map_view.rect = pygame.Rect(
            map_x, HEADER_H + PAD,
            right_x - map_x - PAD,
            height - BOTTOM_H - HEADER_H - 2 * PAD,
        )

        # ---- bottom toolbar ------------------------------------------
        self._build_toolbar(width, height)
        self.widgets = [self.width_input, self.height_input]

    def _build_toolbar(self, width, height):
        """Lays out the bottom bar as labelled groups: SIZE, TOOLS, EDIT,
        ROTATE, FILE, PLAN, VIEW, SIM SPEED."""
        bar_y = height - BOTTOM_H + 6
        self.toolbar_rect = pygame.Rect(PAD, bar_y - 6, width - 2 * PAD, BOTTOM_H - 8)
        self.toolbar_labels = []      # [(x, y, text)]
        self.toolbar_separators = []  # [(x, y_top, y_bottom)]

        row = _ToolbarRow(self, PAD + 12, bar_y + 22)
        row.group("")   # the WIDTH / HEIGHT field captions label this group
        self.width_input.rect = pygame.Rect(row.claim(52), row.y, 52, 26)
        self.height_input.rect = pygame.Rect(row.claim(52), row.y, 52, 26)
        row.button("RESIZE", self.act_resize, "accent", enabled=self.editable)
        row.group("TOOLS")
        for tool in TOOLS:
            row.button(tool, (lambda t=tool: self.act_set_tool(t)),
                       enabled=self.editable,
                       active=(lambda t=tool: self.map_view.tool == t))

        row = _ToolbarRow(self, PAD + 12, bar_y + 62)
        row.group("EDIT")
        row.button("CLEAR WALLS", self.act_clear, "ghost", enabled=self.editable)
        row.button("RESET MAP", self.act_reset, "ghost", enabled=self.editable)
        row.button("RANDOM", self.act_random, "ghost", enabled=self.editable)
        row.group("ROTATE MAP")
        row.button("<< CCW", lambda: self.act_rotate_map(-1), "warn", enabled=self.editable)
        row.button("CW >>", lambda: self.act_rotate_map(1), "warn", enabled=self.editable)
        row.group("ROTATE ROBOT")
        row.button("<< LEFT", lambda: self.act_rotate_robot(-1), "warn", enabled=self.editable)
        row.button("RIGHT >>", lambda: self.act_rotate_robot(1), "warn", enabled=self.editable)
        row.button("FACING: --", self.act_rotate_robot, "ghost",
                   enabled=self.editable, dynamic_label=self._robot_facing_label)
        row.button("SET ORIGIN", self.act_set_origin, "warn",
                   enabled=lambda: self.connected() and self.editable())
        row.group("FILE")
        row.button("SAVE", self.act_save, "ghost", enabled=self.editable)
        row.button("LOAD", self.act_load, "ghost", enabled=self.editable)
        row.button("USE DESIGN MAP", self.act_use_design, "ghost", enabled=self.editable)

        row = _ToolbarRow(self, PAD + 12, bar_y + 102)
        row.group("PLAN")
        row.button("A* PATH", self.act_plan, "accent", enabled=self.editable)
        row.button("AUTO MAP", self.act_automap, "accent",
                   enabled=lambda: not self.controller.mission_active())
        # The Place point itself is set with the "Place" tool in the TOOLS row.
        row.group("GRIPPER")
        row.button("OBJECT PLACE", self.act_pickup, "ok", enabled=self._can_pickup,
                   tooltip="Go to the object's square, sweep the ToF for it and grab it")
        row.button("DELIVERY", self.act_delivery, "ok", enabled=self._can_deliver,
                   tooltip="Carry what is held to the delivery point and release it")
        row.button("TARGET: OBJECT", self.act_object_target, "normal",
                   enabled=lambda: self.editable() and self.controller.map.object_cell is not None,
                   tooltip="Pick the exact sub-square the object stands on")
        row.button("TARGET: DELIVERY", self.act_place_target, "normal",
                   enabled=lambda: self.editable() and self.controller.map.delivery_cell is not None,
                   tooltip="Pick the exact sub-square to release the object on")
        row.button("BACK TO START", self.act_back_to_start, "warn",
                   enabled=self._can_return)
        row.group("TURN ROBOT NOW")
        row.button("TURN LEFT", lambda: self.act_jog_turn(-90.0), "ok", enabled=self._can_jog)
        row.button("TURN RIGHT", lambda: self.act_jog_turn(90.0), "ok", enabled=self._can_jog)
        row.button("TURN 180", lambda: self.act_jog_turn(180.0), "ok", enabled=self._can_jog)
        row = _ToolbarRow(self, PAD + 12, bar_y + 142)
        row.group("VIEW")
        row.button("TRAIL", lambda: self._toggle("show_trail"),
                   active=lambda: self.map_view.show_trail)
        row.button("CLR TRAIL", self.act_clear_trail)
        row.button("SENSORS", lambda: self._toggle("show_sensors"),
                   active=lambda: self.map_view.show_sensors)
        row.button("PATH", lambda: self._toggle("show_planned"),
                   active=lambda: self.map_view.show_planned)
        row.button("TRUTH", self.act_toggle_truth,
                   active=lambda: self.map_view.reveal_truth)
        row.group("SIM SPEED")
        for i, speed in enumerate(SPEED_STEPS):
            row.button("{:g}x".format(speed), (lambda idx=i: self.act_set_speed(idx)),
                       active=(lambda idx=i: self.sim_speed_index == idx),
                       enabled=lambda: self.controller.mode == MODE_SIM,
                       width=36)

    def _robot_facing_label(self):
        return "FACING: {}".format(DIR_LONG[self.controller.map.robot_dir % 4].upper())

    def _gripper_ready(self):
        controller = self.controller
        if not self.connected() or controller.mission_active():
            return False
        if controller.robot.emergency_stopped() or not controller.robot.has_gripper():
            return False
        return controller.armed or not controller.robot.is_physical

    def _can_pickup(self):
        controller = self.controller
        grid = controller.map
        return (self._gripper_ready() and not controller.robot.carrying
                and (grid.object_cell is not None or grid.goal is not None))

    def _can_deliver(self):
        controller = self.controller
        return (self._gripper_ready() and controller.robot.carrying
                and controller.map.delivery_cell is not None)

    def act_pickup(self):
        """Go to the object's square, sweep for it and grab it."""
        controller = self.controller
        robot = controller.robot
        if robot is not None and robot.is_physical:
            self._confirm(
                "OBJECT PLACE",
                ["The robot will drive to {},".format(controller.object_square()),
                 "turn in place sweeping the ToF for the object,",
                 "and close the gripper on it."],
                self._do_pickup, label="GO AND GRAB",
            )
        else:
            self._do_pickup()

    def _do_pickup(self):
        if self.controller.start_pickup_mission():
            self.notify("Object place: looking for the object")
        else:
            self.notify(self.controller.last_error or "Cannot start OBJECT PLACE")

    def act_delivery(self):
        """Carry the held object to the aimed sub-square and release it."""
        controller = self.controller
        grid = controller.map
        robot = controller.robot
        off_x, off_y = getattr(grid, "delivery_offset", (0.0, 0.0))
        cell_m = grid.cell_size_m or controller.config.cell_size_m
        if robot is not None and robot.is_physical:
            self._confirm(
                "DELIVERY",
                ["Carry the object to {} facing {},".format(
                    grid.delivery_cell, DIR_LONG[grid.delivery_dir % 4].upper()),
                 "releasing it {:+.0f} cm E {:+.0f} cm S inside that square.".format(
                     off_x * cell_m * 100, off_y * cell_m * 100)],
                self._do_delivery, label="DELIVER",
            )
        else:
            self._do_delivery()

    def _do_delivery(self):
        if self.controller.start_delivery_mission():
            self.notify("Delivering to {}".format(self.controller.map.delivery_cell))
        else:
            self.notify(self.controller.last_error or "Cannot start DELIVERY")

    def _can_return(self):
        controller = self.controller
        if not self.connected() or controller.mission_active():
            return False
        if controller.robot.emergency_stopped() or controller.map.start is None:
            return False
        return controller.armed or not controller.robot.is_physical

    def act_back_to_start(self):
        controller = self.controller
        robot = controller.robot
        if robot is not None and robot.is_physical:
            self._confirm(
                "RETURN TO START",
                ["The robot will drive back to {}.".format(controller.map.start)],
                self._do_back_to_start, label="GO",
            )
        else:
            self._do_back_to_start()

    def _do_back_to_start(self):
        if self.controller.start_return_to_start():
            self.notify("Returning to Start")
        else:
            self.notify(self.controller.last_error or "Cannot return to Start")

    def act_place_target(self):
        """Zoomed aiming window for the delivery square."""
        grid = self.controller.map
        if grid.delivery_cell is None:
            self.notify("Mark the delivery square first with the Delivery tool")
            return
        self._open_target(DELIVERY_TARGET)

    def act_object_target(self):
        """Zoomed aiming window for the object square."""
        grid = self.controller.map
        if grid.object_cell is None:
            self.notify("Mark the object square first with the Object tool")
            return
        self._open_target(OBJECT_TARGET)

    def _open_target(self, kind):
        grid = self.controller.map
        self.place_dialog = SubCellTargetDialog(
            grid, kind=kind, on_change=lambda: self._on_target_changed(kind),
            cell_size_m=grid.cell_size_m or self.controller.config.cell_size_m)

    def _on_target_changed(self, kind):
        grid = self.controller.map
        cell_m = grid.cell_size_m or self.controller.config.cell_size_m
        if kind == OBJECT_TARGET:
            off_x, off_y = grid.object_offset
            cell = grid.object_cell
        else:
            off_x, off_y = grid.delivery_offset
            cell = grid.delivery_cell
        self.notify("{} aim {:+.0f} cm E {:+.0f} cm S in cell {}".format(
            kind.capitalize(), off_x * cell_m * 100, off_y * cell_m * 100, cell))

    def _can_jog(self):
        controller = self.controller
        if not self.connected() or controller.mission_active():
            return False
        if controller.robot.emergency_stopped():
            return False
        return controller.armed or not controller.robot.is_physical

    def _toggle(self, attr):
        setattr(self.map_view, attr, not getattr(self.map_view, attr))

    # ==================================================================
    # actions
    # ==================================================================
    def act_set_mode(self, mode):
        if mode == self.controller.mode:
            return
        if mode == MODE_REAL:
            self._confirm(
                "SWITCH TO REAL ROBOT MODE",
                [
                    "Commands will be sent to the PHYSICAL RoboMaster.",
                    "Clear the field and keep the emergency stop within reach.",
                    "The robot still has to be CONNECTED and ARMED before it moves.",
                ],
                lambda: self._do_set_mode(mode),
                label="SWITCH TO REAL",
            )
        else:
            self._do_set_mode(mode)

    def _do_set_mode(self, mode):
        if self.controller.set_mode(mode):
            self.map_view.reset_smooth()
            self.notify("Mode: {}".format(mode))

    def act_connect(self):
        if self.controller.mode == MODE_REAL:
            self._confirm(
                "CONNECT TO PHYSICAL ROBOT",
                ["Connect to the RoboMaster EP over {}?".format(self.controller.config.conn_type.upper()),
                 "Sensor streaming will start immediately.",
                 "No motion happens until you ARM and RUN."],
                self._do_connect, label="CONNECT",
            )
        else:
            self._do_connect()

    def _do_connect(self):
        # Connecting to real hardware blocks for several seconds; paint a frame
        # first so the window shows CONNECTING instead of looking frozen.
        self.controller.tracker.set_status(RobotStatus.CONNECTING)
        self.notify("Connecting ({})...".format(self.controller.mode), seconds=30.0)
        self._draw()

        ok, reason = self.controller.connect()
        self.map_view.reset_smooth()
        self.notify(reason if not ok else "Connected ({})".format(self.controller.mode),
                    seconds=12.0 if not ok else 4.0)

    def act_disconnect(self):
        self.controller.disconnect()
        self.map_view.reset_smooth()
        self.notify("Disconnected")

    def act_arm(self):
        robot = self.controller.robot
        if robot is not None and robot.is_physical:
            self._confirm(
                "ARM PHYSICAL ROBOT",
                ["Arming allows motion commands to reach the hardware.",
                 "Make sure the field is clear."],
                lambda: self.controller.arm(), label="ARM",
            )
        else:
            self.controller.arm()

    def act_disarm(self):
        self.controller.disarm()

    def act_run(self):
        controller = self.controller
        if not controller.path_result.ok:
            controller.plan()
        if not controller.path_result.ok:
            self.notify("NO VALID PATH - nothing to run")
            return
        robot = controller.robot
        if robot is not None and robot.is_physical:
            self._confirm(
                "START REAL MISSION",
                ["The physical robot will drive {} cells.".format(controller.path_result.steps),
                 "Estimated duration ~{:.0f} s.".format(controller.path_result.est_time_s),
                 "Emergency stop stays available at all times."],
                self._do_run, label="RUN MISSION",
            )
        else:
            self._do_run()

    def _do_run(self):
        if self.controller.start_navigation():
            self.notify("Mission running")

    def act_pause_resume(self):
        controller = self.controller
        if controller.tracker.get_status() == RobotStatus.PAUSED:
            controller.resume_mission()
        else:
            controller.pause_mission()

    def act_stop(self):
        self.controller.stop_mission()
        self.notify("Mission stopped")

    def act_restart(self):
        robot = self.controller.robot
        if robot is not None and robot.is_physical:
            self._confirm(
                "RESTART RUN",
                ["This only re-zeroes odometry at the start cell.",
                 "The physical robot is NOT driven back - move it yourself first."],
                self._do_restart, label="RE-ZERO",
            )
        else:
            self._do_restart()

    def _do_restart(self):
        self.controller.restart()
        self.map_view.reset_smooth()
        self.notify("Restarted")

    def act_estop(self):
        self.controller.emergency_stop()
        self.notify("EMERGENCY STOP ENGAGED", 8.0)

    def act_clear_estop(self):
        self.controller.clear_emergency_stop()
        self.notify("Emergency stop cleared - robot is disarmed")

    def act_automap(self):
        controller = self.controller
        robot = controller.robot
        if controller.mode == MODE_REAL and (robot is None or not robot.is_physical or not controller.armed):
            if robot is None or not robot.is_connected():
                self.notify("Connect the robot first")
                return
            self._confirm(
                "AUTO-MAP WITH PHYSICAL ROBOT",
                ["The robot will explore the field on its own,",
                 "driving to unknown areas until it runs out of frontiers.",
                 "ARM the robot first; emergency stop stays available."],
                lambda: self._do_automap(), label="START AUTO MAP",
            )
            return
        self._do_automap()

    def _do_automap(self):
        if self.controller.begin_auto_mapping():
            self.map_view.reset_smooth()
            self.notify("Auto-mapping started")
        else:
            self.notify(self.controller.last_error or "Auto-mapping could not start")

    def act_plan(self):
        result = self.controller.plan()
        if result.ok:
            self.notify("Path: {} steps, {:.2f} m, {} turns".format(
                result.steps, result.distance_m, result.turns))
        else:
            self.notify(result.reason)

    def act_resize(self):
        grid = self.controller.map
        grid.resize(self.width_input.value, self.height_input.value, keep=True)
        grid.add_border()
        self.controller.set_map(grid)
        self.notify("Map resized to {} x {}".format(grid.width, grid.height))

    def act_clear(self):
        self.controller.map.clear_walls(keep_border=True)
        self.controller.path_result = self.controller.path_result.__class__()
        self.notify("Walls cleared")

    def act_reset(self):
        grid = self.controller.map
        grid.reset()
        grid.mark_all_known()
        self.controller.set_map(grid)
        self.notify("Map reset")

    def act_random(self):
        grid = self.controller.map
        grid.random_map()
        self.controller.set_map(grid)
        self.notify("Random map generated")

    def act_save(self):
        try:
            path = self.controller.map.save(self.map_path)
            self.notify("Saved {}".format(path))
        except Exception as exc:
            self.notify("Save failed: {}".format(exc))

    def act_load(self):
        try:
            grid = OccupancyGrid.load(self.map_path)
        except Exception as exc:
            self.notify("Load failed: {}".format(exc))
            return
        self.controller.set_map(grid)
        self.width_input.set_value(grid.width)
        self.height_input.set_value(grid.height)
        self.notify("Loaded {}".format(self.map_path))

    def act_use_design(self):
        self.controller.use_design_map()
        self.width_input.set_value(self.controller.map.width)
        self.height_input.set_value(self.controller.map.height)
        self.notify("Switched back to the design map")

    def act_rotate_map(self, quarter_turns=1):
        if self.controller.rotate_map(quarter_turns):
            grid = self.controller.map
            self.width_input.set_value(grid.width)
            self.height_input.set_value(grid.height)
            self.map_view.reset_smooth()
            self.notify("Map rotated ({} x {})".format(grid.width, grid.height))

    def act_set_origin(self):
        """Re-anchors the map frame to where the robot is physically standing."""
        robot = self.controller.robot
        cell = self.controller.map.robot_cell or self.controller.map.start
        facing = DIR_LONG[self.controller.map.robot_dir % 4]
        if robot is not None and robot.is_physical:
            self._confirm(
                "SET ORIGIN",
                ["Confirm the robot is physically standing on cell {},".format(cell),
                 "facing {}.".format(facing.upper()),
                 "Odometry is re-zeroed here. The robot is not moved."],
                self._do_set_origin, label="IT IS THERE", style="accent",
            )
        else:
            self._do_set_origin()

    def _do_set_origin(self):
        if self.controller.set_origin_here():
            self.map_view.reset_smooth()
            self.notify("Origin set at {} facing {}".format(
                self.controller.map.robot_cell,
                DIR_LONG[self.controller.map.robot_dir % 4]))

    def act_rotate_robot(self, quarter_turns=1):
        if self.controller.rotate_robot_start(quarter_turns):
            self.map_view.reset_smooth()
            self.notify("Robot start heading: {}".format(
                DIR_LONG[self.controller.map.robot_dir]))

    def act_jog_turn(self, degrees):
        """Turns the actual robot in place, right now."""
        robot = self.controller.robot
        if robot is not None and robot.is_physical:
            self._confirm(
                "TURN PHYSICAL ROBOT",
                ["The robot will turn {:+.0f} deg in place.".format(degrees),
                 "Make sure it has room to rotate."],
                lambda: self._do_jog(degrees), label="TURN",
            )
        else:
            self._do_jog(degrees)

    def _do_jog(self, degrees):
        if not self.controller.jog_turn(degrees):
            self.notify(self.controller.last_error or "Cannot turn the robot right now")

    def act_clear_trail(self):
        self.controller.tracker.clear_trail()
        self.notify("Trail cleared")

    def act_toggle_truth(self):
        self.map_view.reveal_truth = not self.map_view.reveal_truth
        if self.map_view.reveal_truth:
            self.notify("DEBUG: ground truth revealed")

    def act_set_tool(self, tool):
        self.map_view.tool = tool

    @staticmethod
    def _nearest_speed_index(mps):
        best, best_gap = 1, float("inf")
        for i, (_label, value) in enumerate(ROBOT_SPEEDS):
            gap = abs(value - mps)
            if gap < best_gap:
                best, best_gap = i, gap
        return best

    def act_set_robot_speed(self, index):
        self.robot_speed_index = index
        label, mps = ROBOT_SPEEDS[index]
        self.controller.set_robot_speed(mps)
        self.notify("Robot speed: {} ({:.2f} m/s)".format(label, mps))

    def act_set_speed(self, index):
        self.sim_speed_index = index
        if self.controller.sim_engine is not None:
            self.controller.sim_engine.set_speed(SPEED_STEPS[index])
        self.controller.config.sim_speed = SPEED_STEPS[index]

    def _on_map_edit(self, kind, payload):
        self.controller.path_result = self.controller.path_result.__class__()
        if kind in ("delivery", "delivery_dir"):
            grid = self.controller.map
            self.notify("Delivery square {} facing {} - use TARGET: DELIVERY to aim".format(
                grid.delivery_cell, DIR_LONG[grid.delivery_dir % 4]))
        elif kind == "object":
            self.notify("Object square {} - use TARGET: OBJECT to aim".format(
                self.controller.map.object_cell))
        if kind in ("robot", "robot_dir") and self.connected():
            self.notify("Robot placement changed - press SET ORIGIN once the "
                        "robot is physically there", seconds=8.0)
        if kind in ("robot", "start"):
            grid = self.controller.map
            if kind == "start" and grid.robot_cell is None:
                grid.robot_cell = payload

    # ==================================================================
    # event loop
    # ==================================================================
    def run(self):
        # The shutdown must happen even if the loop dies on an exception: it is
        # what brakes a physical robot that is mid-drive.
        try:
            while self.running:
                for event in pygame.event.get():
                    self._handle(event)
                self.controller.tick()
                self._draw()
                self.clock.tick(60)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.controller.shutdown()
            finally:
                pygame.quit()
        return 0

    def _handle(self, event):
        if event.type == pygame.QUIT:
            self.running = False
            return
        if event.type == pygame.VIDEORESIZE:
            # Below the minimum the side panels would overlap the map.
            width = max(self.MIN_SIZE[0], event.w)
            height = max(self.MIN_SIZE[1], event.h)
            self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
            self._build_layout()
            return

        if self.place_dialog is not None:
            self.place_dialog.layout(self.screen.get_rect())
            self.place_dialog.handle_event(event)
            if self.place_dialog.done:
                self.place_dialog = None
            return
        if self.modal is not None:
            self.modal.layout(self.screen.get_rect())
            self.modal.handle_event(event)
            if self.modal.done:
                self.modal = None
            return

        for widget in self.widgets:
            if widget.handle_event(event):
                return
        for button in self.buttons:
            if button.handle_event(event):
                return

        if event.type == pygame.KEYDOWN:
            if self._handle_key(event):
                return

        self.map_view.handle_event(event, self.controller.map, editable=self.editable())

    def _handle_key(self, event):
        mods = pygame.key.get_mods()
        if event.key == pygame.K_SPACE:
            self.act_plan()
            return True
        if event.key == pygame.K_F1:
            self.act_estop()
            return True
        if mods & pygame.KMOD_CTRL and event.key == pygame.K_s:
            self.act_save()
            return True
        if mods & pygame.KMOD_CTRL and event.key == pygame.K_l:
            self.act_load()
            return True
        if event.key == pygame.K_r:
            self.act_rotate_robot(-1 if mods & pygame.KMOD_SHIFT else 1)
            return True
        if event.key == pygame.K_LEFTBRACKET:
            self.act_rotate_map(-1)
            return True
        if event.key == pygame.K_RIGHTBRACKET:
            self.act_rotate_map(1)
            return True
        if pygame.K_1 <= event.key <= pygame.K_8:
            index = event.key - pygame.K_1
            if index < len(TOOLS):
                self.act_set_tool(TOOLS[index])
            return True
        return False

    # ==================================================================
    # drawing
    # ==================================================================
    def _draw(self):
        controller = self.controller
        state = controller.tracker.get()
        reading = controller.latest_reading()

        self.screen.fill(theme.BG)
        self._draw_header(state)
        self._draw_mode_panel()
        self._draw_connection_panel()
        self._draw_run_panel()
        self._draw_state_panel(state)
        self._draw_sensor_panel(reading, state)
        self._draw_mapping_panel()
        self._draw_mission_panel()
        self._draw_log_panel()
        self._draw_toolbar()

        self.map_view.draw(self.screen, self.fonts, {
            "grid": controller.map,
            "ground_truth": controller.ground_truth,
            "state": state,
            "trail": controller.tracker.trail(),
            "reading": reading,
            "path_result": controller.path_result,
            "executed_index": controller.executed_index,
            "transform": controller.transform,
            "cell_size_m": controller.map.cell_size_m,
            "tracking_ok": controller.tracker.tracking_ok(),
            "mapping": controller.mapping_status == "ACTIVE",
        })

        for button in self.buttons:
            button.draw(self.screen, self.fonts)
        for widget in self.widgets:
            widget.draw(self.screen, self.fonts)
        self.progress_bar.draw(self.screen, self.fonts)

        if self.status_message and time.time() < self.status_until:
            self._draw_toast(self.status_message)
        if self.modal is not None:
            self.modal.layout(self.screen.get_rect())
            self.modal.draw(self.screen, self.fonts)
        if self.place_dialog is not None:
            self.place_dialog.layout(self.screen.get_rect())
            self.place_dialog.draw(self.screen, self.fonts)
        pygame.display.flip()

    def _draw_toast(self, text):
        # Failure reasons can be long; wrap instead of running off the map.
        font = self.fonts.body
        max_width = max(240, self.map_view.rect.width - 40)
        lines = []
        line = ""
        for word in str(text).split():
            trial = (line + " " + word).strip()
            if font.size(trial)[0] > max_width and line:
                lines.append(line)
                line = word
            else:
                line = trial
        if line:
            lines.append(line)
        lines = lines[:4]

        line_h = font.get_height() + 2
        width = max(font.size(l)[0] for l in lines) + 28
        rect = pygame.Rect(0, 0, width, line_h * len(lines) + 16)
        rect.midbottom = (self.map_view.rect.centerx, self.map_view.rect.bottom - 8)
        pygame.draw.rect(self.screen, theme.PANEL_ALT, rect, border_radius=6)
        pygame.draw.rect(self.screen, theme.ACCENT, rect, 1, border_radius=6)
        for i, line in enumerate(lines):
            draw_text(self.screen, font, line,
                      (rect.centerx, rect.y + 8 + i * line_h), theme.TEXT, align="center")

    def _draw_header(self, state):
        width = self.screen.get_width()
        pygame.draw.rect(self.screen, theme.HEADER, (0, 0, width, HEADER_H))
        pygame.draw.line(self.screen, theme.PANEL_EDGE, (0, HEADER_H), (width, HEADER_H))
        draw_text(self.screen, self.fonts.title, "ROBOMASTER MISSION CONTROL", (PAD + 4, 15))

        x = 420
        mode = self.controller.mode
        mode_color = theme.DANGER if mode == MODE_REAL else (
            theme.VIOLET if mode == MODE_MOCK else theme.ACCENT)
        x = self._chip(x, "MODE", mode, mode_color)
        link = "ONLINE" if self.connected() else "OFFLINE"
        x = self._chip(x, "LINK", link, theme.OK if self.connected() else theme.TEXT_FAINT)
        x = self._chip(x, "STATE", state.status, theme.status_color(state.status))
        if self.controller.armed:
            x = self._chip(x, "SAFETY", "ARMED", theme.WARN)

    def _chip(self, x, label, value, color):
        text = "{} {}".format(label, value)
        width = self.fonts.small.size(text)[0] + 22
        rect = pygame.Rect(x, 12, width, 28)
        pygame.draw.rect(self.screen, theme.PANEL, rect, border_radius=14)
        pygame.draw.rect(self.screen, color, rect, 1, border_radius=14)
        pygame.draw.circle(self.screen, color, (rect.x + 12, rect.centery), 4)
        draw_text(self.screen, self.fonts.small, value, (rect.x + 22, rect.y + 6), color)
        draw_text(self.screen, self.fonts.tiny, label, (rect.x + 2, rect.y - 12), theme.TEXT_FAINT)
        return rect.right + 10

    def _draw_mode_panel(self):
        draw_panel(self.screen, self.mode_rect, "mode", self.fonts)
        note = "Simulation - hardware untouched"
        color = theme.TEXT_DIM
        if self.controller.mode == MODE_REAL:
            note = "REAL HARDWARE - commands go to the robot"
            color = theme.DANGER
        elif self.controller.mode == MODE_MOCK:
            note = "Mock robot - development only"
            color = theme.VIOLET
        draw_text(self.screen, self.fonts.tiny, note, (self.mode_rect.x + 10, self.mode_rect.bottom - 16), color)

    def _draw_connection_panel(self):
        draw_panel(self.screen, self.conn_rect, "connection", self.fonts)

    def _draw_run_panel(self):
        rect = self.run_rect
        draw_panel(self.screen, rect, "run control", self.fonts)
        draw_text(self.screen, self.fonts.tiny,
                  "ROBOT SPEED  {:.2f} m/s".format(self.controller.config.base_speed_mps),
                  (rect.x + 10, rect.y + 108), theme.TEXT_DIM)

    def _draw_state_panel(self, state):
        rect = self.state_rect
        draw_panel(self.screen, rect, "robot state", self.fonts)
        x = rect.x + 12
        y = rect.y + 34
        width = rect.width - 24

        color = theme.status_color(state.status)
        pygame.draw.rect(self.screen, theme.blend(color, theme.PANEL, 0.75),
                         (x, y, width, 30), border_radius=5)
        pygame.draw.rect(self.screen, color, (x, y, width, 30), 1, border_radius=5)
        draw_text(self.screen, self.fonts.h1, state.status, (x + 10, y + 7), color)
        y += 40

        controller = self.controller
        heading_dir = DIR_LONG[heading_to_dir(state.map_heading)] if state.valid else "--"
        rows = [
            ("X", "{:+.2f} m".format(state.x) if state.valid else "--"),
            ("Y", "{:+.2f} m".format(state.y) if state.valid else "--"),
            ("HEADING", "{:+.1f} deg ({})".format(state.map_heading, heading_dir) if state.valid else "--"),
            ("SPEED", "{:.2f} m/s".format(state.velocity) if state.valid else "--"),
            ("CELL", str(state.cell) if state.valid else "--"),
            ("TARGET", str(state.current_target) if state.current_target else "--"),
            ("CHECKPOINT", str(state.current_checkpoint) if state.current_checkpoint else "--"),
            ("MISSION", controller.mission_kind),
            ("NAVIGATION", controller.navigation_status),
            ("MAPPING", controller.mapping_status),
            ("SENSORS", "LIVE" if controller.sensor_source is not None else "NONE"),
            ("GRIPPER", self._gripper_text()),
        ]
        limit = rect.bottom - 26
        for key, value in rows:
            if y > limit:
                break
            y = draw_kv(self.screen, self.fonts, x, y, key, value, width)

        age = controller.tracker.age()
        age_text = "{:.1f} s ago".format(age) if age != float("inf") else "no data"
        ok = controller.tracker.tracking_ok()
        if y <= limit:
            y = draw_kv(self.screen, self.fonts, x, y, "LAST UPDATE", age_text, width,
                        theme.OK if ok else theme.DANGER)

        if not ok and state.valid and y + 44 <= rect.bottom - 8:
            y += 6
            pygame.draw.rect(self.screen, theme.blend(theme.ORANGE, theme.PANEL, 0.7),
                             (x, y, width, 44), border_radius=5)
            draw_text(self.screen, self.fonts.small, "! ROBOT TRACKING LOST", (x + 8, y + 5), theme.ORANGE)
            draw_text(self.screen, self.fonts.tiny,
                      "last X={:.2f} Y={:.2f}".format(state.x, state.y),
                      (x + 8, y + 24), theme.TEXT_DIM)
            y += 52

        if controller.last_error and y + 40 <= rect.bottom - 8:
            y += 6
            pygame.draw.rect(self.screen, theme.blend(theme.DANGER, theme.PANEL, 0.72),
                             (x, y, width, 40), border_radius=5)
            draw_text(self.screen, self.fonts.tiny, "ERROR", (x + 8, y + 5), theme.DANGER)
            self._wrap_text(controller.last_error, x + 8, y + 20, width - 16, self.fonts.tiny,
                            theme.TEXT, max_lines=1)

    def _gripper_text(self):
        robot = self.controller.robot
        if robot is None or not robot.is_connected():
            return "--"
        if not robot.has_gripper():
            return "NONE"
        return "CARRYING" if robot.carrying else "EMPTY"

    def _wrap_text(self, text, x, y, width, font, color, max_lines=3):
        words = str(text).split()
        line = ""
        lines = []
        for word in words:
            trial = (line + " " + word).strip()
            if font.size(trial)[0] > width and line:
                lines.append(line)
                line = word
            else:
                line = trial
        if line:
            lines.append(line)
        for i, line in enumerate(lines[:max_lines]):
            draw_text(self.screen, font, line, (x, y + i * (font.get_height() + 1)), color)
        return y + min(len(lines), max_lines) * (font.get_height() + 1)

    def _draw_sensor_panel(self, reading, state):
        rect = self.sensor_rect
        draw_panel(self.screen, rect, "sensor debug", self.fonts)
        x = rect.x + 12
        y = rect.y + 34
        width = rect.width - 24

        if reading is None:
            draw_text(self.screen, self.fonts.small, "no sensor data", (x, y), theme.TEXT_FAINT)
            return
        limit = rect.bottom - 20
        for label, value in reading.as_display():
            color = theme.TEXT
            if value in ("--", "invalid"):
                color = theme.TEXT_FAINT
            y = draw_kv(self.screen, self.fonts, x, y, label, value, width, color)
        y = draw_kv(self.screen, self.fonts, x, y, "Source",
                    "REAL ROBOT" if reading.source == "real" else "SIMULATED", width,
                    theme.OK if reading.source == "real" else theme.VIOLET)
        y += 6
        pygame.draw.line(self.screen, theme.PANEL_EDGE, (x, y), (rect.right - 12, y))
        y += 6
        for key, value in (
            ("X", "{:+.3f} m".format(state.x)),
            ("Y", "{:+.3f} m".format(state.y)),
            ("Yaw (robot)", "{:+.1f} deg".format(state.heading)),
            ("Heading (map)", "{:+.1f} deg".format(state.map_heading)),
            ("Frame", str(reading.frame_index)),
        ):
            if y > limit:
                break
            y = draw_kv(self.screen, self.fonts, x, y, key, value, width)

    def _draw_mapping_panel(self):
        rect = self.mapping_rect
        draw_panel(self.screen, rect, "auto mapping", self.fonts)
        stats = self.controller.mapping_progress()
        x = rect.x + 12
        status = self.controller.mapping_status
        color = theme.VIOLET if status == "ACTIVE" else (
            theme.OK if status == "COMPLETE" else theme.TEXT_DIM)
        draw_text(self.screen, self.fonts.small, "Mapping: {}".format(status), (x, rect.y + 30), color)
        self.progress_bar.value = stats["progress"]
        draw_text(self.screen, self.fonts.small, "{:.0f}%".format(stats["progress"] * 100),
                  (rect.right - 12, rect.y + 30), theme.TEXT, align="right")
        # Keep both count lines inside the panel even when it has been shrunk.
        y = min(rect.y + 74, rect.bottom - 34)
        draw_text(self.screen, self.fonts.tiny,
                  "Discovered {} / {} cells".format(stats["known"], stats["total"]),
                  (x, y), theme.TEXT_DIM)
        draw_text(self.screen, self.fonts.tiny,
                  "free {}   walls {}   unknown {}   obstacles {}".format(
                      stats["free"], stats["wall_edges"], stats["unknown"], stats["obstacle"]),
                  (x, y + 16), theme.TEXT_DIM)

    def _draw_mission_panel(self):
        rect = self.mission_rect
        draw_panel(self.screen, rect, "mission", self.fonts)
        summary = self.controller.mission_summary()
        x = rect.x + 12
        y = rect.y + 32
        marks = {"done": ("[x]", theme.OK), "active": ("[>]", theme.ACCENT),
                 "pending": ("[ ]", theme.TEXT_FAINT)}
        if not summary["waypoints"]:
            draw_text(self.screen, self.fonts.small, "no mission planned", (x, y), theme.TEXT_FAINT)
        for label, cell, status in summary["waypoints"][:5]:
            mark, color = marks.get(status, marks["pending"])
            draw_text(self.screen, self.fonts.small, "{} {}".format(mark, label), (x, y), color)
            draw_text(self.screen, self.fonts.tiny, str(cell), (rect.right - 12, y + 1),
                      theme.TEXT_DIM, align="right")
            y += 18
        y = max(y, rect.bottom - 56)
        width = rect.width - 24
        y = draw_kv(self.screen, self.fonts, x, y, "Steps remaining",
                    "{} / {}".format(summary["steps_remaining"], summary["steps_total"]), width)
        y = draw_kv(self.screen, self.fonts, x, y, "Distance remaining",
                    "{:.2f} m".format(summary["distance_remaining_m"]), width)
        color = theme.WARN if summary["deviation"] else theme.TEXT
        y = draw_kv(self.screen, self.fonts, x, y, "Navigation",
                    summary["navigation"] + (" (replans {})".format(summary["replans"])
                                             if summary["replans"] else ""), width, color)

    def _draw_log_panel(self):
        rect = self.log_rect
        if rect.height < 46:
            return
        draw_panel(self.screen, rect, "event log", self.fonts)
        events = list(self.controller.events)[-((rect.height - 34) // 15):]
        y = rect.y + 30
        for stamp, level, text in events:
            color = {"warn": theme.WARN, "error": theme.DANGER}.get(level, theme.TEXT_DIM)
            line = "{} {}".format(stamp, text)
            if self.fonts.tiny.size(line)[0] > rect.width - 20:
                while line and self.fonts.tiny.size(line + "...")[0] > rect.width - 20:
                    line = line[:-1]
                line += "..."
            draw_text(self.screen, self.fonts.tiny, line, (rect.x + 10, y), color)
            y += 15

    def _draw_toolbar(self):
        draw_panel(self.screen, self.toolbar_rect, None, self.fonts, fill=theme.PANEL)
        for x, y_top, y_bottom in self.toolbar_separators:
            pygame.draw.line(self.screen, theme.PANEL_EDGE, (x, y_top), (x, y_bottom))
        for x, y, label in self.toolbar_labels:
            draw_text(self.screen, self.fonts.tiny, label, (x, y), theme.TEXT_FAINT)


def run(map_file=None, mode=None, conn_type="ap", cell_size=0.60, size=(1600, 1000)):
    """Entry point used by ``main.py panel``."""
    config = MissionConfig(cell_size_m=cell_size, conn_type=conn_type)
    grid = None
    if map_file and os.path.exists(map_file):
        try:
            grid = OccupancyGrid.load(map_file)
        except Exception as exc:
            print("[panel] Could not load {}: {}".format(map_file, exc))
    if grid is None:
        grid = OccupancyGrid(9, 9)
        grid.add_border()
        grid.mark_all_known()
        grid.start = (0, 0)
        grid.goal = (grid.width - 1, grid.height - 1)
        grid.robot_cell = grid.start
    grid.cell_size_m = cell_size

    controller = MissionController(grid, config)
    if mode:
        controller.mode = mode
    app = MissionControlApp(controller, map_path=map_file or DEFAULT_MAP_PATH, size=size)
    app.width_input.set_value(grid.width)
    app.height_input.set_value(grid.height)
    return app.run()


if __name__ == "__main__":
    sys.exit(run())
