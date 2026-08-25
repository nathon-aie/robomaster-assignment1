#!/usr/bin/env python3
from robomaster import robot
from gripper_controller import SimpleGripperController

def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    gripper_ctrl = SimpleGripperController(ep_robot)

    try:
        # 1. หยิบของ (เปิดปาก -> ยื่น 5 ซม. -> หนีบ -> ยกขึ้น 10 ซม.)
        gripper_ctrl.pick(extend_cm=7, lift_cm=10)

        # 2. วางของ (ลงแขน -> ปล่อย -> ถอยหุ่น 15 ซม. -> หุบปากเก็บ)
        gripper_ctrl.drop(chassis=ep_robot.chassis, back_cm=15)

    except Exception as exc:
        print(f"[main] error: {exc}")
    finally:
        ep_robot.close()

if __name__ == "__main__":
    main()