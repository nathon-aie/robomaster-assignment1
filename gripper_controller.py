#!/usr/bin/env python3
import time

class SimpleGripperController:
    def __init__(self, ep_robot=None, dry_run=False):
        self.dry_run = dry_run
        self.arm = getattr(ep_robot, "robotic_arm", None) if ep_robot else None
        self.gripper = getattr(ep_robot, "gripper", None) if ep_robot else None

    def _move_arm(self, x=0, y=0, action_name=""):
        if action_name: print(f"[arm] {action_name}")
        if self.dry_run: return
        if not self.arm: raise RuntimeError("Arm unavailable")
        action = self.arm.move(x=x, y=y)
        if hasattr(action, "wait_for_completed"): action.wait_for_completed()

    def open(self):
        if not self.gripper:
            if self.dry_run: return print("[gripper] open (dry-run)")
            raise RuntimeError("Gripper unavailable")
        print("[gripper] opening...")
        self.gripper.open()
        time.sleep(1.0)

    def close(self):
        if not self.gripper:
            if self.dry_run: return print("[gripper] close (dry-run)")
            raise RuntimeError("Gripper unavailable")
        print("[gripper] closing...")
        self.gripper.close()
        time.sleep(1.0)

    def recenter(self):
        """หุบ gripper แล้วดึงแขนกลับจุดเริ่มต้น"""
        print("[arm] recentering (closing gripper & retracting arm)...")
        self.close()
        if self.dry_run: return print("[arm] recenter (dry-run)")
        if not self.arm: raise RuntimeError("Arm unavailable")
        action = self.arm.recenter()
        if hasattr(action, "wait_for_completed"): action.wait_for_completed()

    def pick(self, extend_cm=5, lift_cm=10):
        """ลำดับการหยิบ: เปิดปาก -> ยื่น -> หุบจับ -> ยกขึ้น"""
        print("[action] --- Start Pick ---")
        self.open()
        self._move_arm(x=extend_cm * 10, y=0, action_name=f"extend {extend_cm} cm")
        time.sleep(1)
        self.close()
        self._move_arm(x=0, y=lift_cm * 10, action_name=f"lift {lift_cm} cm")
        print("[action] Pick Finished\n")

    def drop(self, chassis=None, back_cm=15):
        """ลำดับการวาง: ถอยแขนลง -> ปล่อยของ -> ถอยหุ่น -> หุบปากเก็บ"""
        print("[action] --- Start Drop ---")
        self._move_arm(x=0, y=-100, action_name="lower arm 10 cm") # 1. ถอยแขนลงก่อน
        self.open()                                                # 2. ปล่อยของ
        if chassis:                                                # 3. ถอยหุ่น
            print(f"[chassis] backing up {back_cm} cm...")
            if not self.dry_run:
                chassis.move(x=-(back_cm / 100.0), y=0, z=0, xy_speed=0.5).wait_for_completed()
        self.recenter()                                            # 4. หุบปากเก็บ
        print("[action] Drop Finished\n")