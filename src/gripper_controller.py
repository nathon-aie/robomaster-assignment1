#!/usr/bin/env python3
"""RoboMaster EP arm and gripper sequences used by the main navigation flow."""

import time


class SimpleGripperController:
    def __init__(self, ep_robot=None, dry_run=False):
        self.dry_run = dry_run
        self.arm = getattr(ep_robot, "robotic_arm", None) if ep_robot else None
        self.gripper = getattr(ep_robot, "gripper", None) if ep_robot else None

    def _move_arm(self, x=0, y=0, action_name=""):
        if action_name:
            print(f"[arm] {action_name}")
        if self.dry_run:
            return
        if not self.arm:
            raise RuntimeError("Arm unavailable")
        action = self.arm.move(x=x, y=y)
        if hasattr(action, "wait_for_completed"):
            action.wait_for_completed()

    def open(self):
        if not self.gripper:
            if self.dry_run:
                print("[gripper] open (dry-run)")
                return
            raise RuntimeError("Gripper unavailable")
        print("[gripper] opening...")
        self.gripper.open()
        time.sleep(1.0)

    def close(self):
        if not self.gripper:
            if self.dry_run:
                print("[gripper] close (dry-run)")
                return
            raise RuntimeError("Gripper unavailable")
        print("[gripper] closing...")
        self.gripper.close()
        time.sleep(1.0)

    def recenter(self):
        print("[arm] recentering (closing gripper & retracting arm)...")
        self.close()
        if self.dry_run:
            print("[arm] recenter (dry-run)")
            return
        if not self.arm:
            raise RuntimeError("Arm unavailable")
        action = self.arm.recenter()
        if hasattr(action, "wait_for_completed"):
            action.wait_for_completed()

    def pick(self, extend_cm=7.0, lift_cm=10.0):
        """Open, move to the object, close the gripper, and lift it."""
        print("[action] --- Start Pick ---")
        self.open()
        time.sleep(0.7)
        self._move_arm(x=150, y=0, action_name="lower arm 10 cm")
        time.sleep(0.7)
        self._move_arm(x=0, y=-200, action_name="lower arm 10 cm")
        time.sleep(0.7)
        self._move_arm(x=100, y=0, action_name="lower arm 10 cm")
        time.sleep(0.7)
        self.close()
        self._move_arm(x=0, y=200, action_name="lift arm")
        time.sleep(0.7)
        self._move_arm(x=-100, y=0, action_name="retract arm")
        print("[action] Pick Finished\n")

    def drop(self, chassis=None, back_cm=50):
        """Move to the drop position, release the object, and recenter."""
        print("[action] --- Start Drop ---")
        if chassis and not self.dry_run:
            print(f"[chassis] backing up {back_cm} cm...")
            action = chassis.move(x=-(back_cm / 100.0), y=-(back_cm / 100.0) + 0.325, z=0, xy_speed=0.7)
            if hasattr(action, "wait_for_completed"):
                action.wait_for_completed()

        self._move_arm(x=150, y=0, action_name="lower arm 10 cm")
        self._move_arm(x=0, y=-200, action_name="lower arm 10 cm")
        self.open()
        time.sleep(1.0)
        self._move_arm(x=0, y=200, action_name="lift arm")
        time.sleep(1.0)
        self._move_arm(x=-100, y=0, action_name="retract arm")
        self.recenter()
        print("[action] Drop Finished\n")