#!/usr/bin/env python3
"""Robot interface abstraction.

    UI -> MissionController -> RobotInterface -> RoboMaster SDK / simulation

Three implementations share one surface:

    RealRobotInterface  physical RoboMaster EP through the existing
                        ``RobotSystem`` (Thread 1 sensors + Thread 2 PID grid
                        navigation).  Nothing is faked here.
    MockRobotInterface  the project's existing ``MockRobotActuators`` for
                        development when no hardware is present.
    SimRobotInterface   the kinematic simulator with simulated sensors.

Turn convention across all three: ``turn(+90)`` is a right/clockwise turn on
the map, matching the yaw convention in ``geometry.py``.
"""

import atexit
import math
import signal
import threading
import time

from .robot_state import RobotStatus
from .sensors import RealSensorInterface


class _ChassisSafetyNet(object):
    """Stops the physical chassis whenever this process is going away.

    ``drive_speed`` is continuous: the wheels hold the last commanded velocity
    until a new command arrives.  If the controlling program exits mid-drive
    without braking, the robot keeps going until it hits something.  The SDK's
    own watchdog timer runs *in this process*, so it dies with it and cannot
    help here.

    Covers normal exit, unhandled exceptions, Ctrl+C and SIGTERM.  It cannot
    cover SIGKILL / "End task" - nothing in-process can - so the operator must
    still keep the physical robot switch within reach.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._stoppers = []
        self._installed = False

    def register(self, stopper):
        with self._lock:
            if stopper not in self._stoppers:
                self._stoppers.append(stopper)
            self._install()

    def unregister(self, stopper):
        with self._lock:
            if stopper in self._stoppers:
                self._stoppers.remove(stopper)

    def _install(self):
        if self._installed:
            return
        self._installed = True
        atexit.register(self.stop_all)
        for sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None),
                    getattr(signal, "SIGBREAK", None)):
            if sig is None:
                continue
            try:
                previous = signal.getsignal(sig)
                signal.signal(sig, self._make_handler(sig, previous))
            except (ValueError, OSError, RuntimeError):
                # Not on the main thread, or the platform refuses this signal.
                pass

    def _make_handler(self, sig, previous):
        def handler(signum, frame):
            self.stop_all()
            if callable(previous):
                previous(signum, frame)
            elif previous == signal.SIG_DFL:
                signal.signal(sig, signal.SIG_DFL)
                os_kill_self(signum)
        return handler

    def stop_all(self):
        with self._lock:
            stoppers = list(self._stoppers)
        for stopper in stoppers:
            try:
                stopper()
            except Exception:
                pass


def os_kill_self(signum):
    import os
    try:
        os.kill(os.getpid(), signum)
    except Exception:
        pass


#: Process-wide safety net; every connected physical robot registers with it.
CHASSIS_SAFETY_NET = _ChassisSafetyNet()


class RobotCommandResult(object):
    """Outcome of a motion command.  Success is only ever reported by the
    interface that actually performed it."""

    __slots__ = ("ok", "reason")

    def __init__(self, ok, reason=""):
        self.ok = bool(ok)
        self.reason = reason

    def __bool__(self):
        return self.ok

    __nonzero__ = __bool__

    def __repr__(self):
        return "<RobotCommandResult ok={} {!r}>".format(self.ok, self.reason)


OK = RobotCommandResult(True, "OK")


def _load_robot_system():
    """Imports ``RobotSystem`` whether the package is used as ``src.panel`` or ``panel``."""
    try:
        from ..robot_system import RobotSystem
    except (ImportError, ValueError):
        from robot_system import RobotSystem
    return RobotSystem


def _sdk_problem():
    """Returns why the DJI SDK cannot be used, or ``None`` when it is fine.

    ``RobotSystem.connect_robot()`` only returns a bool and falls back to mock,
    so a missing SDK and an unreachable robot look identical from outside.  They
    need very different fixes, so check the import separately and say which it is.
    """
    try:
        try:
            from ..calibrate import load_robot_sdk
        except (ImportError, ValueError):
            from calibrate import load_robot_sdk
    except Exception as exc:
        return "cannot load the project SDK helper: {}".format(exc)

    try:
        load_robot_sdk()
    except Exception as exc:
        return str(exc)
    return None


#: Shown when the SDK is present but the robot does not answer.
UNREACHABLE_HINT = (
    "Robot did not respond over {mode}. Check: (1) the PC is joined to the "
    "robot's Wi-Fi (SSID RMEP-xxxxxx) - not a campus/home network; "
    "(2) the robot's connection switch is set to the matching mode; "
    "(3) 192.168.2.1 answers a ping in AP mode."
)


class RobotInterface(object):
    """Common surface for every robot backend."""

    kind = "abstract"
    is_physical = False

    def __init__(self):
        self.on_status = None
        self._estop = False
        # A stop request latches.  Without this the *next* motion command would
        # silently resume driving, so a STOP pressed between two steps of a
        # mission would be lost.
        self._stop_event = threading.Event()

    def stop_requested(self):
        return self._stop_event.is_set()

    def clear_stop(self):
        """Explicitly lifts a previous stop so motion may be commanded again."""
        self._stop_event.clear()

    def _motion_guard(self):
        """Common refusal check run before every motion command."""
        if not self.is_connected():
            return RobotCommandResult(False, "Not connected")
        if self._estop:
            return RobotCommandResult(False, "Emergency stop engaged")
        if self._stop_event.is_set():
            return RobotCommandResult(False, "Stop requested")
        return None

    # ------------------------------------------------------------- connection
    def connect(self):
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError

    def is_connected(self):
        return False

    # ------------------------------------------------------------------ motion
    def move_cells(self, cells=1):
        raise NotImplementedError

    def turn(self, degrees):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def emergency_stop(self):
        self._estop = True
        self.stop()

    def clear_emergency_stop(self):
        self._estop = False
        self._stop_event.clear()

    def emergency_stopped(self):
        return self._estop

    def pause(self):
        pass

    def resume(self):
        pass

    # ------------------------------------------------------------------ gripper
    #: True once something is held, so the UI and mission can show/queue it.
    carrying = False
    #: How far in front of the robot's centre this backend actually releases
    #: an object, in metres.  Negative means behind - the real drop sequence
    #: reverses the chassis before opening, so the object ends up back there.
    release_offset_m = 0.25

    def has_gripper(self):
        """Whether this backend can pick things up at all."""
        return False

    def pick(self):
        """Grabs the object in front of the robot."""
        return RobotCommandResult(False, "No gripper on this backend")

    def place(self, offset_xy=None):
        """Puts the carried object down.

        ``offset_xy`` is an optional (forward, right) nudge in metres, applied
        before releasing so the object lands on the aimed sub-cell spot rather
        than wherever the robot happened to stop.
        """
        return RobotCommandResult(False, "No gripper on this backend")

    # ------------------------------------------------------------------ sensors
    def sensors(self):
        raise NotImplementedError

    def zero_odometry(self, cell, direction):
        """Re-zeroes odometry so the given map cell/heading becomes the origin."""

    def _notify(self, status, message=""):
        if self.on_status:
            try:
                self.on_status(status, message)
            except Exception:
                pass


# --------------------------------------------------------------------------
# Physical RoboMaster EP
# --------------------------------------------------------------------------

class RealRobotInterface(RobotInterface):
    """Drives the physical robot through the existing ``RobotSystem`` stack.

    Thread 1 (sensor collection + filtering) runs as usual.  Thread 2's own
    command loop is *not* started - its PID grid-navigation methods are called
    directly, one cell / one turn at a time, so the mission controller keeps
    control and can replan between steps.
    """

    kind = "real"
    is_physical = True

    def __init__(self, conn_type="ap", calibration_file="calibration_output/calibration.json",
                 sensor_rate_hz=20.0, base_speed=0.15, nominal_side_mm=140.0,
                 cell_size_m=0.60, allow_mock_fallback=False, turn_speed_dps=30.0,
                 place_backoff_cm=50.0, gripper_reach_m=0.25):
        RobotInterface.__init__(self)
        # main's field-tuned drop sequence reverses the chassis before opening
        # the gripper, so the object is released behind where the robot stood.
        self.place_backoff_cm = place_backoff_cm
        self.gripper_reach_m = gripper_reach_m
        # drop() reverses by place_backoff_cm and *then* lowers the arm, so the
        # object lands that much further back than the arm's own reach.
        self.release_offset_m = gripper_reach_m - place_backoff_cm / 100.0
        self.conn_type = conn_type
        self.calibration_file = calibration_file
        self.sensor_rate_hz = sensor_rate_hz
        self.base_speed = base_speed
        self.turn_speed_dps = turn_speed_dps
        self.nominal_side_mm = nominal_side_mm
        self.cell_size_m = cell_size_m
        self.allow_mock_fallback = allow_mock_fallback
        self.system = None
        self._sensors = None
        self._connected = False
        self._gripper_ctrl = None
        self._lock = threading.RLock()

    # ------------------------------------------------------------- connection
    def connect(self):
        RobotSystem = _load_robot_system()  # deferred: pulls in the SDK

        with self._lock:
            if self._connected:
                return RobotCommandResult(True, "Already connected")
            self._notify(RobotStatus.CONNECTING, "Connecting via {}".format(self.conn_type.upper()))

            problem = _sdk_problem()
            if problem is not None:
                reason = "RoboMaster SDK not usable: {}".format(problem)
                self._notify(RobotStatus.ERROR, reason)
                return RobotCommandResult(False, reason)

            try:
                system = RobotSystem(
                    calibration_file=self.calibration_file,
                    sensor_rate_hz=self.sensor_rate_hz,
                    mock_mode=False,
                    conn_type=self.conn_type,
                )
            except Exception as exc:
                self._notify(RobotStatus.ERROR, "SDK unavailable: {}".format(exc))
                return RobotCommandResult(False, "SDK unavailable: {}".format(exc))

            ok = False
            try:
                ok = system.connect_robot()
            except Exception as exc:
                self._notify(RobotStatus.ERROR, "Connect failed: {}".format(exc))
                return RobotCommandResult(False, "Connect failed: {}".format(exc))

            # RobotSystem silently falls back to mock on failure - refuse that here.
            if not ok or system.mock_mode or system.robot is None:
                if not self.allow_mock_fallback:
                    reason = UNREACHABLE_HINT.format(mode=self.conn_type.upper())
                    self._notify(RobotStatus.ERROR, reason)
                    return RobotCommandResult(False, reason)

            system.setup_threads()
            controller = system.thread_2_controller
            controller.grid_size_m = self.cell_size_m
            controller.base_speed = self.base_speed
            controller.wall_pid.nominal_side_dist_mm = self.nominal_side_mm
            # Arm the controller's motion primitives without starting its own
            # command loop - the mission controller sequences the steps.
            controller._running.set()
            controller._pause_event.set()

            system.thread_1_sensor.start_collecting()
            self.system = system
            self._sensors = RealSensorInterface(system.sensor_hub)
            self._connected = True
            self._estop = False
            self._stop_event.clear()
            if self.is_physical:
                CHASSIS_SAFETY_NET.register(self._panic_stop)
            self._notify(RobotStatus.CONNECTED, "Connected to RoboMaster EP")
            return RobotCommandResult(True, "Connected")

    def _panic_stop(self):
        """Last-ditch brake, called while the process is exiting.

        Talks to the chassis directly - the controller threads may already be
        gone by the time interpreter shutdown reaches us.
        """
        system = self.system
        if system is None or system.robot is None:
            return
        try:
            system.thread_2_controller._running.clear()
        except Exception:
            pass
        for _ in range(3):
            try:
                system.robot.chassis.drive_speed(x=0, y=0, z=0)
            except Exception:
                break

    def disconnect(self):
        CHASSIS_SAFETY_NET.unregister(self._panic_stop)
        with self._lock:
            if self.system is not None:
                self._panic_stop()
                try:
                    self.system.thread_2_controller.stop_running()
                except Exception:
                    pass
                try:
                    self.system.shutdown(save_telemetry=True, run_analysis=False)
                except Exception as exc:
                    print("[RealRobotInterface] shutdown warning: {}".format(exc))
            self.system = None
            self._sensors = None
            self._connected = False
            self._notify(RobotStatus.DISCONNECTED, "Disconnected")
            return OK

    def is_connected(self):
        return self._connected and self.system is not None

    # ------------------------------------------------------------------ motion
    def _controller(self):
        if not self.is_connected():
            return None
        return self.system.thread_2_controller

    def move_cells(self, cells=1):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        controller = self._controller()
        if controller is None:
            return RobotCommandResult(False, "Not connected")
        try:
            controller.move_forward_grid(cells=int(cells))
        except Exception as exc:
            return RobotCommandResult(False, "Move failed: {}".format(exc))
        if self._estop or not controller._running.is_set():
            return RobotCommandResult(False, "Motion aborted")
        return OK

    def turn(self, degrees):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        controller = self._controller()
        if controller is None:
            return RobotCommandResult(False, "Not connected")
        speed = self.turn_speed_dps
        try:
            if abs(abs(degrees) - 180.0) < 1.0:
                controller.turn_around(speed=speed)
            elif degrees > 0:
                controller.turn_right(abs(degrees), speed=speed)
            elif degrees < 0:
                controller.turn_left(abs(degrees), speed=speed)
        except Exception as exc:
            return RobotCommandResult(False, "Turn failed: {}".format(exc))
        if self._estop or not controller._running.is_set():
            return RobotCommandResult(False, "Motion aborted")
        return OK

    def stop(self):
        self._stop_event.set()
        controller = self._controller()
        if controller is not None:
            try:
                controller.stop_chassis()
            except Exception as exc:
                return RobotCommandResult(False, "Stop failed: {}".format(exc))
        return OK

    def emergency_stop(self):
        self._estop = True
        controller = self._controller()
        if controller is not None:
            try:
                controller._running.clear()   # breaks out of any running motion loop
                controller.emergency_stop()
            except Exception as exc:
                return RobotCommandResult(False, "E-stop error: {}".format(exc))
        self._notify(RobotStatus.EMERGENCY_STOP, "EMERGENCY STOP")
        return OK

    def clear_emergency_stop(self):
        self._estop = False
        self._stop_event.clear()
        controller = self._controller()
        if controller is not None:
            controller._running.set()
            controller.wall_pid.reset()

    def pause(self):
        controller = self._controller()
        if controller is not None:
            controller.pause()
        self.stop()

    def resume(self):
        self._stop_event.clear()
        controller = self._controller()
        if controller is not None:
            controller.resume()

    # ----------------------------------------------------------------- sensors
    def set_speed(self, metres_per_second):
        """Applies a new cruise speed to the running controller."""
        self.base_speed = max(0.05, float(metres_per_second))
        controller = self._controller()
        if controller is not None:
            controller.base_speed = self.base_speed

    def set_turn_speed(self, degrees_per_second):
        self.turn_speed_dps = max(5.0, float(degrees_per_second))

    def sensors(self):
        return self._sensors

    # ------------------------------------------------------------------ gripper
    def _gripper(self):
        """Lazily builds the arm/gripper controller ported from the gripper branch."""
        if self._gripper_ctrl is None and self.is_connected():
            try:
                try:
                    from ..gripper_controller import SimpleGripperController
                except (ImportError, ValueError):
                    from gripper_controller import SimpleGripperController
            except Exception as exc:
                self._notify(RobotStatus.ERROR, "Gripper module unavailable: {}".format(exc))
                return None
            self._gripper_ctrl = SimpleGripperController(self.system.robot)
        return self._gripper_ctrl

    def has_gripper(self):
        if not self.is_connected():
            return False
        robot = self.system.robot
        return getattr(robot, "gripper", None) is not None

    def pick(self):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        controller = self._gripper()
        if controller is None:
            return RobotCommandResult(False, "Gripper unavailable")
        try:
            controller.pick()
        except Exception as exc:
            return RobotCommandResult(False, "Pick failed: {}".format(exc))
        self.carrying = True
        return RobotCommandResult(True, "Picked up")

    def place(self, offset_xy=None):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        controller = self._gripper()
        if controller is None:
            return RobotCommandResult(False, "Gripper unavailable")
        try:
            chassis = self.system.robot.chassis if self.system.robot else None
            if chassis is not None and offset_xy and any(offset_xy):
                # Line the chassis up with the aimed spot inside the cell.
                forward, right = offset_xy
                action = chassis.move(x=forward, y=right, z=0, xy_speed=0.3)
                if hasattr(action, "wait_for_completed"):
                    action.wait_for_completed()
            controller.drop(chassis=chassis, back_cm=self.place_backoff_cm)
        except Exception as exc:
            return RobotCommandResult(False, "Place failed: {}".format(exc))
        self.carrying = False
        return RobotCommandResult(True, "Placed")

    def zero_odometry(self, cell, direction):
        if not self.is_connected():
            return
        collector = self.system.thread_1_sensor
        collector.reset_position_zero()
        collector.reset_heading_zero()
        controller = self._controller()
        if controller is not None:
            controller.target_heading_deg = 0.0


class MockRobotInterface(RealRobotInterface):
    """Development backend: the project's own ``MockRobotActuators``.

    Behaves like the real interface but never touches hardware.  Its sensor
    values are synthetic, so it is presented in the UI as its own mode and is
    never mistaken for Real Robot mode.
    """

    kind = "mock"
    is_physical = False

    def connect(self):
        RobotSystem = _load_robot_system()

        with self._lock:
            if self._connected:
                return RobotCommandResult(True, "Already connected")
            self._notify(RobotStatus.CONNECTING, "Starting mock robot")
            system = RobotSystem(
                calibration_file=self.calibration_file,
                sensor_rate_hz=self.sensor_rate_hz,
                mock_mode=True,
            )
            system.connect_robot()
            system.setup_threads()
            controller = system.thread_2_controller
            controller.grid_size_m = self.cell_size_m
            controller.base_speed = self.base_speed
            controller._running.set()
            controller._pause_event.set()
            system.thread_1_sensor.start_collecting()
            self.system = system
            self._sensors = RealSensorInterface(system.sensor_hub)
            self._connected = True
            self._estop = False
            self._stop_event.clear()
            self._notify(RobotStatus.CONNECTED, "Mock robot ready (no hardware)")
            return RobotCommandResult(True, "Mock robot ready")


# --------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------

class SimRobotInterface(RobotInterface):
    """Commands the kinematic simulator; blocks until each move completes."""

    kind = "sim"
    is_physical = False

    def __init__(self, sim_robot, sensor_interface, engine=None, cell_size_m=0.60,
                 gripper_reach_m=0.25):
        RobotInterface.__init__(self)
        # The simulator sets the object down in front of itself; it does not
        # model the real drop sequence's reverse.
        self.release_offset_m = gripper_reach_m
        self.clearance_m = 0.16
        #: Where the object was last let go, in map cell coordinates.
        self.last_release_point = None
        self.robot = sim_robot
        self._sensors = sensor_interface
        self.engine = engine
        self.cell_size_m = cell_size_m
        self._connected = False

    def connect(self):
        self._connected = True
        self._estop = False
        self._stop_event.clear()
        self._notify(RobotStatus.CONNECTED, "Simulator ready")
        return OK

    def disconnect(self):
        self._connected = False
        self.robot.stop()
        self._notify(RobotStatus.DISCONNECTED, "Simulator stopped")
        return OK

    def is_connected(self):
        return self._connected

    def _wait_idle(self, timeout=120.0):
        deadline = time.monotonic() + timeout
        while self.robot.is_busy():
            if self._estop or self._stop_event.is_set():
                self.robot.stop()
                return RobotCommandResult(False, "Motion aborted")
            if time.monotonic() > deadline:
                self.robot.stop()
                return RobotCommandResult(False, "Motion timeout")
            time.sleep(0.01)
        if self.robot.blocked():
            return RobotCommandResult(False, "Blocked by obstacle")
        return OK

    def move_cells(self, cells=1):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        for _ in range(int(cells)):
            self.robot.command_move(self.cell_size_m)
            result = self._wait_idle()
            if not result.ok:
                return result
        return OK

    def turn(self, degrees):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        self.robot.command_turn(degrees)
        return self._wait_idle(timeout=60.0)

    def stop(self):
        self._stop_event.set()
        self.robot.stop()
        return OK

    def emergency_stop(self):
        self._estop = True
        self.stop()
        self._notify(RobotStatus.EMERGENCY_STOP, "EMERGENCY STOP (simulation)")
        return OK

    def clear_emergency_stop(self):
        self._estop = False
        self._stop_event.clear()

    def resume(self):
        self._stop_event.clear()
        if self.engine:
            self.engine.resume()

    def pause(self):
        if self.engine:
            self.engine.pause()

    def set_speed(self, metres_per_second):
        self.robot.base_speed = max(0.05, float(metres_per_second))

    def has_gripper(self):
        return True

    def pick(self):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        if self.carrying:
            return RobotCommandResult(False, "Already carrying something")
        target = self._object_in_front()
        if target is None:
            return RobotCommandResult(False, "Nothing within reach to pick up")
        time.sleep(0.4)
        self.robot.ground_truth.objects.discard(target)
        self.carrying = True
        # The payload now blocks the front beam, exactly as on the real robot.
        if self._sensors is not None:
            self._sensors.payload_distance_m = 0.12
        return RobotCommandResult(True, "Picked up (simulated)")

    def _object_in_front(self):
        """The simulated object the gripper could close on, if any."""
        from .geometry import DIR_VECTORS, heading_to_dir

        truth = getattr(self.robot, "ground_truth", None)
        if truth is None or not getattr(truth, "objects", None):
            return None
        map_pose = self.robot.transform.robot_to_map(self.robot.pose())
        cell = map_pose.cell
        d_col, d_row = DIR_VECTORS[heading_to_dir(map_pose.heading_deg)]
        for candidate in ((cell[0] + d_col, cell[1] + d_row), cell):
            if candidate in truth.objects:
                return candidate
        return None

    def place(self, offset_xy=None):
        refusal = self._motion_guard()
        if refusal is not None:
            return refusal
        if not self.carrying:
            return RobotCommandResult(False, "Nothing to place")
        time.sleep(0.4)
        if offset_xy:
            # Line up on the aim point exactly, the way chassis.move() does.
            self.robot.nudge(offset_xy[0], offset_xy[1],
                             clearance_m=self.clearance_m)
        truth = getattr(self.robot, "ground_truth", None)
        if truth is not None:
            # The object leaves the gripper in front of the robot, not under it.
            pose = self.robot.pose()
            map_pose = self.robot.transform.robot_to_map(pose)
            cell_m = self.robot.transform.cell_size_m or 0.60
            angle = math.radians(map_pose.heading_deg)
            reach = self.release_offset_m / cell_m
            self.last_release_point = (map_pose.col + math.sin(angle) * reach,
                                       map_pose.row - math.cos(angle) * reach)
            truth.objects.add((int(math.floor(self.last_release_point[0] + 0.5)),
                               int(math.floor(self.last_release_point[1] + 0.5))))
        self.carrying = False
        if self._sensors is not None:
            self._sensors.payload_distance_m = None
        return RobotCommandResult(True, "Placed (simulated)")

    def sensors(self):
        return self._sensors

    def zero_odometry(self, cell, direction):
        self.robot.place(cell, direction)
