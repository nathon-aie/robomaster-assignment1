#!/usr/bin/env python3
"""RoboMaster EP Autonomous Grid Navigation System - Master CLI.

Quick Commands:
    ./run sim               # Run dry-run simulation
    ./run map               # Open interactive Map Planner GUI
    ./run step 1            # Test move forward 1 cell (60 cm)
    ./run turn right        # Test turn right (+90 deg)
    ./run mon               # Live stream sensor telemetry
    ./run ana               # Analyze latest telemetry run log
    ./run run               # Run autonomous navigation on live robot
"""

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import List, Optional

# Ensure src is in sys.path
_SRC_DIR = Path(__file__).resolve().parent / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from src.config_loader import load_settings
from src.gripper_controller import SimpleGripperController
from src.robot_system import RobotSystem
from src.telemetry import TelemetryAnalyzer


# ---------------------------------------------------------------------------
# Utility Functions
# ---------------------------------------------------------------------------

def parse_custom_commands(cmd_input: str) -> List[str]:
    """Parses arbitrary command string or JSON list into robot controller commands."""
    if not cmd_input:
        return []
    s = cmd_input.strip()
    if s.startswith("["):
        try:
            return json.loads(s)
        except Exception:
            pass

    raw_items = [c.strip() for c in s.replace(";", ",").split(",") if c.strip()]
    parsed = []
    for item in raw_items:
        low = item.lower()
        if any(w in low for w in ("fwd", "forward", "move", "cell")):
            nums = [int(tok) for tok in item.split() if tok.isdigit()]
            cells = nums[0] if nums else 1
            parsed.append(f"Move Forward: {cells} cells")
        elif "left" in low:
            parsed.append("Turn Left (90 deg)")
        elif "right" in low:
            parsed.append("Turn Right (90 deg)")
        elif "around" in low or "180" in low:
            parsed.append("Turn Around (180 deg)")
        elif "open" in low:
            parsed.append("Gripper Open")
        elif "close" in low:
            parsed.append("Gripper Close")
        else:
            parsed.append(item)
    return parsed


def confirm_action(prompt: str, auto_yes: bool = False) -> bool:
    """Prompts user for confirmation [y/N], or bypasses if auto_yes is True."""
    if auto_yes:
        return True
    try:
        answer = input(f"{prompt} [y/N]: ").strip().lower()
        if answer not in ("y", "yes"):
            print("[main] Operation cancelled.")
            return False
        return True
    except (EOFError, KeyboardInterrupt):
        return False


def resolve_telemetry_target(target: Optional[str] = None) -> str:
    """Resolves telemetry target path or finds the latest run automatically."""
    logs_dir = Path("telemetry_logs")
    if not target or target.strip() == "":
        if logs_dir.exists():
            runs = [d for d in logs_dir.iterdir() if d.is_dir() and d.name.startswith("run")]
            if runs:
                runs.sort(key=lambda d: d.stat().st_mtime)
                latest = runs[-1]
                print(f"[main] Auto-selected latest run: {latest}")
                return str(latest)
            json_files = list(logs_dir.glob("*.json"))
            if json_files:
                json_files.sort(key=lambda f: f.stat().st_mtime)
                return str(json_files[-1])
        return "telemetry_logs"

    if target.isdigit():
        candidate = logs_dir / f"run{target}"
        if candidate.exists():
            return str(candidate)

    if (logs_dir / target).exists():
        return str(logs_dir / target)

    return target


def run_pick_and_wait_for_navigation(sys_runner: RobotSystem, args) -> Optional[bool]:
    """Orchestrates Pick -> Map Follow -> Drop workflow."""
    gripper_ctrl = SimpleGripperController(
        ep_robot=sys_runner.robot,
        dry_run=sys_runner.mock_mode,
    )
    auto_yes = getattr(args, "yes", False)

    # 1. Pick sequence
    if not getattr(args, "skip_pick", False):
        if not confirm_action("Start pick now?", auto_yes=auto_yes):
            return None
        gripper_ctrl.pick(
            extend_cm=getattr(args, "extend_cm", 7.0),
            lift_cm=getattr(args, "lift_cm", 10.0),
        )
        if not confirm_action("Pick finished. Start following the map now?", auto_yes=auto_yes):
            return None
    else:
        if not confirm_action("Start following the map now?", auto_yes=auto_yes):
            return None

    # 2. Setup threads and command queue
    sys_runner.setup_threads(plan_file=args.plan)
    if hasattr(args, "commands") and args.commands and sys_runner.thread_2_controller:
        custom_cmds = parse_custom_commands(args.commands)
        sys_runner.thread_2_controller.set_commands(custom_cmds)
    elif getattr(args, "backward_cm", None) is not None and sys_runner.thread_2_controller:
        sys_runner.thread_2_controller.set_commands([f"Move Backward: {args.backward_cm} cm"])

    if sys_runner.thread_2_controller:
        sys_runner.thread_2_controller.base_speed = getattr(args, "speed", 0.25)
        sys_runner.thread_2_controller.wall_pid.nominal_side_dist_mm = getattr(args, "nominal_side", 140.0)

    # 3. Start navigation
    sys_runner.start()
    reached_goal = sys_runner.wait_for_completion(
        timeout=args.duration if getattr(args, "duration", 0.0) > 0 else None
    )

    # 4. Drop sequence at goal
    if reached_goal and not getattr(args, "skip_drop", False):
        gripper_ctrl.drop(chassis=sys_runner.robot.chassis if sys_runner.robot else None)
    elif not reached_goal:
        print("[main] Navigation did not reach the goal; drop skipped.")

    return reached_goal


# ---------------------------------------------------------------------------
# CLI Command Handlers
# ---------------------------------------------------------------------------

def cmd_simulate(args):
    print("=" * 65)
    print("🤖 STARTING MULTI-THREADING SIMULATION (STEP 3 PID NAVIGATION)")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=True,
    )
    sys_runner.connect_robot()

    def sig_handler(sig, frame):
        print("\nInterrupt received. Stopping...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    try:
        run_pick_and_wait_for_navigation(sys_runner, args)
    finally:
        sys_runner.shutdown()


def cmd_run(args):
    print("=" * 65)
    print("🤖 STARTING LIVE AUTONOMOUS NAVIGATION (PID GRID CONTROL)")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=False,
        conn_type=args.conn_type,
    )
    connected = sys_runner.connect_robot()
    if not connected and not args.allow_mock_fallback:
        print("[Error] Failed to connect to robot hardware. Exiting.")
        return 1

    def sig_handler(sig, frame):
        print("\nInterrupt received. Emergency stop initiated...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    try:
        run_pick_and_wait_for_navigation(sys_runner, args)
    finally:
        sys_runner.shutdown()
    return 0


def cmd_step_test(args):
    cells = args.cells
    print("=" * 65)
    print(f"🎯 TESTING {cells} GRID CELL(S) (60x60 cm) WITH PID CENTERING")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=args.mock,
        conn_type=args.conn_type,
    )
    sys_runner.connect_robot()
    sys_runner.setup_threads()

    if sys_runner.thread_2_controller:
        sys_runner.thread_2_controller.base_speed = args.speed
        sys_runner.thread_2_controller.wall_pid.nominal_side_dist_mm = args.nominal_side
        sys_runner.thread_2_controller.set_commands([f"Move Forward: {cells} cells"])

    def sig_handler(sig, frame):
        print("\nInterrupt received. Stopping...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    sys_runner.start()
    sys_runner.wait_for_completion(timeout=30.0)
    sys_runner.shutdown()


def cmd_turn_test(args):
    direction_str = str(args.direction).lower()
    if direction_str in ("left", "l"):
        deg, cmd_text = 90.0, "Turn Left (90 deg)"
    elif direction_str in ("right", "r"):
        deg, cmd_text = -90.0, "Turn Right (90 deg)"
    elif direction_str in ("around", "a", "180"):
        deg, cmd_text = 180.0, "Turn Around (180 deg)"
    else:
        try:
            deg = float(direction_str)
            cmd_text = f"Turn {deg:+.0f} deg"
        except ValueError:
            print(f"Unknown direction '{args.direction}'. Use 'left', 'right', 'around', or degrees like '90' or '-90'.")
            return 1

    print("=" * 65)
    print(f"🔄 TESTING TURN: {cmd_text} (z={deg:+.0f}°)")
    print("=" * 65)

    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=args.mock,
        conn_type=args.conn_type,
    )
    sys_runner.connect_robot()
    sys_runner.setup_threads()

    if sys_runner.thread_2_controller:
        sys_runner.thread_2_controller.set_commands([cmd_text])

    def sig_handler(sig, frame):
        print("\nInterrupt received. Stopping...")
        sys_runner.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    sys_runner.start()
    sys_runner.wait_for_completion(timeout=15.0)
    sys_runner.shutdown()
    return 0


def cmd_gripper_action(action_name: str, args):
    """Directly controls gripper pick or drop."""
    print("=" * 65)
    print(f"🦾 EXECUTING GRIPPER {action_name.upper()}")
    print("=" * 65)
    sys_runner = RobotSystem(mock_mode=args.mock, conn_type=args.conn_type)
    sys_runner.connect_robot()
    ctrl = SimpleGripperController(ep_robot=sys_runner.robot, dry_run=sys_runner.mock_mode)

    try:
        if action_name == "pick":
            ctrl.pick(extend_cm=args.extend_cm, lift_cm=args.lift_cm)
        elif action_name == "drop":
            ctrl.drop(chassis=sys_runner.robot.chassis if sys_runner.robot else None, back_cm=args.back_cm)
        elif action_name == "open":
            ctrl.open()
        elif action_name == "close":
            ctrl.close()
        elif action_name == "recenter":
            ctrl.recenter()
    finally:
        sys_runner.shutdown(save_telemetry=False)


def cmd_monitor(args):
    print("=" * 65)
    print("📡 LIVE SENSOR MONITOR (THREAD 1)")
    print("=" * 65)
    sys_runner = RobotSystem(
        calibration_file=args.calibration,
        sensor_rate_hz=args.rate,
        mock_mode=args.mock,
        conn_type=args.conn_type,
    )
    sys_runner.connect_robot()
    sys_runner.setup_threads()

    def sig_handler(sig, frame):
        sys_runner.shutdown(save_telemetry=False)
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    sys_runner.thread_1_sensor.start_collecting()

    try:
        print(f"{'Frame':<8} | {'Sharp L (mm)':<13} | {'Sharp R (mm)':<13} | {'ToF (mm)':<10} | {'Yaw (deg)':<10} | {'Walls (L/F/R)'}")
        print("-" * 75)
        while True:
            state = sys_runner.sensor_hub.wait_for_next_state(timeout=1.0)
            if state:
                sl = f"{state.sharp_left_mm:.1f}" if state.sharp_left_mm is not None else "N/A"
                sr = f"{state.sharp_right_mm:.1f}" if state.sharp_right_mm is not None else "N/A"
                tof = f"{state.tof_filtered_mm:.1f}" if state.tof_filtered_mm is not None else "N/A"
                walls = f"{'L' if state.wall_left_detected else '-'}/{'F' if state.wall_front_detected else '-'}/{'R' if state.wall_right_detected else '-'}"
                print(f"{state.frame_index:<8} | {sl:<13} | {sr:<13} | {tof:<10} | {state.yaw:<10.1f} | {walls}")
            time.sleep(0.1)
    finally:
        sys_runner.shutdown(save_telemetry=False)


def cmd_analyze(args):
    target = resolve_telemetry_target(args.file)
    print(f"📊 Analyzing telemetry: {target}")
    TelemetryAnalyzer.analyze_file(target, save_plot=not args.no_plot)


def cmd_calibrate(args):
    from src.calibrate import collect_live, fit_command, init_csv
    if args.cal_cmd in ("init-csv", "init"):
        init_csv(args.path)
    elif args.cal_cmd in ("collect-live", "collect"):
        collect_live(args.sensor, args.output, args.board_id, args.port, args.samples, args.conn_type)
    elif args.cal_cmd in ("fit", "f"):
        fit_command(args.input, args.output_dir)


def cmd_map(args):
    print("🗺️ Launching Pygame Grid Map & A* Path Planner...")
    from src.map_planner import main as launch_map_gui
    launch_map_gui()


# ---------------------------------------------------------------------------
# CLI Argument Parser Setup
# ---------------------------------------------------------------------------

def print_help_menu():
    """Prints a friendly dashboard of all quick commands."""
    print("""
🤖 RoboMaster EP Navigation System — Quick Command Menu

Usage:
    python main.py <command> [options]   or   ./run <command>

Commands:
    sim, simulate       Run dry-run simulation (no robot needed)
    run, r              Run autonomous navigation on live robot
    step, st [N]        Step forward N cells (default: 1 cell / 60 cm)
    turn, t [DIR]       Turn robot: right, left, around (default: right)
    map, m              Open interactive Grid Map & A* Path Planner GUI
    mon, live           Live stream sensor telemetry from Thread 1
    ana, a [RUN]        Analyze telemetry logs (auto-picks latest run if omitted)
    pick                Execute gripper pick sequence
    drop                Execute gripper drop sequence
    cal, calibrate      Sharp IR sensor calibration tools (Step 1)

Examples:
    python main.py sim
    python main.py map
    python main.py step 1
    python main.py turn right
    python main.py mon
    python main.py ana
    python main.py run -y
""")


def build_parser() -> argparse.ArgumentParser:
    cfg = load_settings()
    r_cfg = cfg.get("robot", {})
    s_cfg = cfg.get("sensor_pipeline", {})
    c_cfg = cfg.get("controller", {})
    g_cfg = cfg.get("gripper", {})
    sys_cfg = cfg.get("system", {})

    parser = argparse.ArgumentParser(
        description="RoboMaster EP Autonomous Grid Navigation System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command")

    # 1. Run live (aliases: run, r)
    run_p = subparsers.add_parser("run", aliases=["r"], help="Run live robot autonomous navigation")
    run_p.add_argument("--plan", default=sys_cfg.get("default_plan_file", "data/robot_map_plan.json"), help="Map plan JSON path")
    run_p.add_argument("--commands", default="", help="Custom comma-separated command list")
    run_p.add_argument("--calibration", default=s_cfg.get("calibration_file", "calibration_output/calibration.json"))
    run_p.add_argument("--conn-type", choices=("ap", "sta"), default=r_cfg.get("conn_type", "ap"), help="Connection mode (default: ap)")
    run_p.add_argument("--rate", type=float, default=s_cfg.get("sensor_rate_hz", 20.0), help="Sensor rate Hz")
    run_p.add_argument("--speed", type=float, default=c_cfg.get("cruising_speed", 0.25), help="Speed m/s")
    run_p.add_argument("--nominal-side", type=float, default=c_cfg.get("nominal_side_dist_mm", 140.0), help="Nominal wall distance mm")
    run_p.add_argument("--duration", type=float, default=0.0, help="Timeout in seconds")
    run_p.add_argument("--backward-cm", type=float, help="Initial backward move cm")
    run_p.add_argument("--extend-cm", type=float, default=g_cfg.get("extend_cm", 7.0), help="Arm extension cm")
    run_p.add_argument("--lift-cm", type=float, default=g_cfg.get("lift_cm", 10.0), help="Arm lift cm")
    run_p.add_argument("--skip-pick", action="store_true", help="Skip initial pick sequence")
    run_p.add_argument("--skip-drop", action="store_true", help="Skip final drop sequence")
    run_p.add_argument("-y", "--yes", action="store_true", help="Auto-confirm all prompts")
    run_p.add_argument("--allow-mock-fallback", action="store_true", help="Fallback to mock if connection fails")

    # 2. Simulate (aliases: simulate, sim, s)
    sim_p = subparsers.add_parser("simulate", aliases=["sim", "s"], help="Run simulation / dry-run")
    sim_p.add_argument("--plan", default=sys_cfg.get("default_plan_file", "data/robot_map_plan.json"), help="Map plan JSON path")
    sim_p.add_argument("--commands", default="", help="Custom comma-separated command list")
    sim_p.add_argument("--calibration", default=s_cfg.get("calibration_file", "calibration_output/calibration.json"))
    sim_p.add_argument("--rate", type=float, default=s_cfg.get("sensor_rate_hz", 20.0), help="Sensor rate Hz")
    sim_p.add_argument("--speed", type=float, default=c_cfg.get("cruising_speed", 0.25), help="Speed m/s")
    sim_p.add_argument("--nominal-side", type=float, default=c_cfg.get("nominal_side_dist_mm", 140.0), help="Nominal wall distance mm")
    sim_p.add_argument("--duration", type=float, default=0.0, help="Timeout in seconds")
    sim_p.add_argument("--backward-cm", type=float, help="Initial backward move cm")
    sim_p.add_argument("--extend-cm", type=float, default=g_cfg.get("extend_cm", 7.0), help="Arm extension cm")
    sim_p.add_argument("--lift-cm", type=float, default=g_cfg.get("lift_cm", 10.0), help="Arm lift cm")
    sim_p.add_argument("--skip-pick", action="store_true", help="Skip initial pick sequence")
    sim_p.add_argument("--skip-drop", action="store_true", help="Skip final drop sequence")
    sim_p.add_argument("-y", "--yes", action="store_true", help="Auto-confirm all prompts")

    # 3. Step-test (aliases: step-test, step, st)
    step_p = subparsers.add_parser("step-test", aliases=["step", "st"], help="Test N grid cell move with PID")
    step_p.add_argument("cells", nargs="?", type=int, default=1, help="Number of cells to move (default: 1)")
    step_p.add_argument("--conn-type", choices=("ap", "sta"), default=r_cfg.get("conn_type", "ap"))
    step_p.add_argument("--calibration", default=s_cfg.get("calibration_file", "calibration_output/calibration.json"))
    step_p.add_argument("--rate", type=float, default=s_cfg.get("sensor_rate_hz", 20.0))
    step_p.add_argument("--speed", type=float, default=c_cfg.get("cruising_speed", 0.25))
    step_p.add_argument("--nominal-side", type=float, default=c_cfg.get("nominal_side_dist_mm", 140.0))
    step_p.add_argument("--mock", action="store_true", help="Run in mock mode")

    # 4. Turn-test (aliases: turn-test, turn, t)
    turn_p = subparsers.add_parser("turn-test", aliases=["turn", "t"], help="Test in-place turn (right, left, around)")
    turn_p.add_argument("direction", nargs="?", default="right", help="Direction: right, left, around, or degrees (default: right)")
    turn_p.add_argument("--conn-type", choices=("ap", "sta"), default=r_cfg.get("conn_type", "ap"))
    turn_p.add_argument("--calibration", default=s_cfg.get("calibration_file", "calibration_output/calibration.json"))
    turn_p.add_argument("--rate", type=float, default=s_cfg.get("sensor_rate_hz", 20.0))
    turn_p.add_argument("--mock", action="store_true")

    # 5. Gripper direct controls (pick, drop, open, close, recenter)
    pick_p = subparsers.add_parser("pick", help="Run gripper pick sequence")
    pick_p.add_argument("--extend-cm", type=float, default=g_cfg.get("extend_cm", 7.0))
    pick_p.add_argument("--lift-cm", type=float, default=g_cfg.get("lift_cm", 10.0))
    pick_p.add_argument("--conn-type", default=r_cfg.get("conn_type", "ap"))
    pick_p.add_argument("--mock", action="store_true")

    drop_p = subparsers.add_parser("drop", help="Run gripper drop sequence")
    drop_p.add_argument("--back-cm", type=float, default=g_cfg.get("drop_backup_cm", 30.0))
    drop_p.add_argument("--conn-type", default=r_cfg.get("conn_type", "ap"))
    drop_p.add_argument("--mock", action="store_true")

    # 6. Monitor (aliases: monitor, mon, live)
    mon_p = subparsers.add_parser("monitor", aliases=["mon", "live"], help="Live stream sensor telemetry")
    mon_p.add_argument("--conn-type", choices=("ap", "sta"), default=r_cfg.get("conn_type", "ap"))
    mon_p.add_argument("--calibration", default=s_cfg.get("calibration_file", "calibration_output/calibration.json"))
    mon_p.add_argument("--rate", type=float, default=s_cfg.get("sensor_rate_hz", 20.0))
    mon_p.add_argument("--mock", action="store_true")

    # 7. Analyze (aliases: analyze, ana, a)
    ana_p = subparsers.add_parser("analyze", aliases=["ana", "a"], help="Analyze telemetry log and generate graphs")
    ana_p.add_argument("file", nargs="?", default=None, help="Path or run number (e.g. 1, run1, or telemetry_logs/run1). Auto-picks latest if omitted.")
    ana_p.add_argument("--no-plot", action="store_true", help="Skip plot generation")

    # 8. Calibrate (aliases: calibrate, cal)
    cal_p = subparsers.add_parser("calibrate", aliases=["cal"], help="Sharp IR sensor calibration tools (Step 1)")
    cal_sub = cal_p.add_subparsers(dest="cal_cmd", required=True)
    c_init = cal_sub.add_parser("init-csv", aliases=["init"], help="Create CSV template")
    c_init.add_argument("path", nargs="?", default="data/calibration_measurements.csv")
    c_live = cal_sub.add_parser("collect-live", aliases=["collect"], help="Collect live Sharp sensor values")
    c_live.add_argument("sensor", choices=("sharp_left", "sharp_right"))
    c_live.add_argument("--output", default="data/calibration_measurements.csv")
    c_live.add_argument("--board-id", type=int)
    c_live.add_argument("--port", type=int)
    c_live.add_argument("--samples", type=int, default=10)
    c_live.add_argument("--conn-type", choices=("ap", "sta"), default=r_cfg.get("conn_type", "ap"))
    c_fit = cal_sub.add_parser("fit", aliases=["f"], help="Fit polynomial curves")
    c_fit.add_argument("input", nargs="?", default="data/calibration_measurements.csv")
    c_fit.add_argument("--output-dir", default="calibration_output")

    # 9. Map GUI (aliases: map, m)
    subparsers.add_parser("map", aliases=["m"], help="Launch interactive Grid Map & A* Planner GUI")

    return parser


def main():
    if len(sys.argv) == 1:
        print_help_menu()
        return 0

    parser = build_parser()
    args = parser.parse_args()

    cmd = args.command
    if cmd in ("run", "r"):
        return cmd_run(args)
    elif cmd in ("simulate", "sim", "s"):
        return cmd_simulate(args)
    elif cmd in ("step-test", "step", "st"):
        return cmd_step_test(args)
    elif cmd in ("turn-test", "turn", "t"):
        return cmd_turn_test(args)
    elif cmd == "pick":
        return cmd_gripper_action("pick", args)
    elif cmd == "drop":
        return cmd_gripper_action("drop", args)
    elif cmd in ("monitor", "mon", "live"):
        return cmd_monitor(args)
    elif cmd in ("analyze", "ana", "a"):
        return cmd_analyze(args)
    elif cmd in ("calibrate", "cal"):
        return cmd_calibrate(args)
    elif cmd in ("map", "m"):
        return cmd_map(args)

    return 0


if __name__ == "__main__":
    sys.exit(main())
