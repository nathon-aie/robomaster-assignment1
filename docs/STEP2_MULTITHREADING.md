# Step 2 — Multi-Threading Architecture for RoboMaster EP

เอกสารและคู่มือการทำงานของระบบ **Multi-threading (2 Threads)** สำหรับ RoboMaster EP ตามข้อกำหนดใน [REQ.md](../REQ.md)

---

## 🏗 โครงสร้างสถาปัตยกรรม (Architecture)

ระบบแบ่งการทำงานออกเป็น **2 Threads** หลักที่สื่อสารกันผ่าน **Thread-safe Shared State Hub (`SensorHub`)**:

```text
+-------------------------------------------------------------------------------+
|                       RoboMaster EP Hardware / SDK                            |
|  - Sharp Left (id1, port1 ADC)          - ToF Distance (Front mm)             |
|  - Sharp Right (id2, port2 ADC)         - IMU / Attitude (Yaw/Pitch/Roll)     |
|  - Chassis Odometry (X, Y, Z, Vel)      - ESC / Status / Gripper Status       |
+---------------------------------------+---------------------------------------+
                                        | (SDK Subscriptions / Callbacks)
                                        v
+-------------------------------------------------------------------------------+
|                  THREAD 1: SensorCollectorThread (Producer @ 20 Hz)           |
|                                                                               |
|  1. Outlier Rejection Filter   -> ป้องกันสัญญาณกระโดดหรือค่านอกช่วง           |
|  2. Median Filter (Window=5)   -> ขจัด Noise แบบ Spike จากแสงสะท้อน IR        |
|  3. Exponential Moving Average -> Low-pass Filter (EMA) เพิ่มความนิ่ง         |
|  4. Polynomial Calibration     -> แปลง ADC เป็นระยะ mm จาก calibration.json    |
|  5. Wall Feature Extraction    -> วิเคราะห์กำแพงซ้าย/ขวา/หน้า และผลต่าง |L-R|  |
|  6. Telemetry Logger           -> บันทึก Time-series ลง Buffer แบบเรียลไทม์   |
+---------------------------------------+---------------------------------------+
                                        |
                 Atomic Snapshot Update | (Thread-Safe Lock)
                                        v
+-------------------------------------------------------------------------------+
|                         SensorHub (Shared Memory Hub)                         |
|   - get_latest_state() -> RobotSensorSnapshot (ดึงได้ทันที ไม่ต้องเรียก SDK ซ้ำ) |
|   - wait_for_next_state(timeout)                                              |
|   - get_history_snapshot() -> สำหรับนำไปทำ Realtime Grid Mapping              |
+---------------------------------------+---------------------------------------+
                                        |
                          ดึง Snapshot  | (Zero Hardware Overhead)
                                        v
+-------------------------------------------------------------------------------+
|                  THREAD 2: RobotControllerThread (Consumer)                   |
|                                                                               |
|  - รับแผนการเดินจาก robot_map_plan.json หรือคำสั่งภารกิจ                      |
|  - ตรวจสอบ Sensor Snapshot จาก Thread 1 เพื่อนำไปควบคุมการเคลื่อนที่         |
|  - สั่งงาน Actuator (Chassis Move, Turn, Gripper Open/Close)                  |
|  - รองรับ PID Control ใน Step 3 และ Gripper Placement ใน Step 4                |
+-----------------------------------------------------------------------+-------+
                                        |
                                        v (เมื่อจบการทำงาน)
+-------------------------------------------------------------------------------+
|                      Telemetry Analyzer & Exporter                            |
|  - telemetry_logs/run1/run1_<ts>.json (ข้อมูลดิบ + สถิติสรุป)                  |
|  - telemetry_logs/run1/run1_<ts>.csv (ตาราง Time-series สำหรับนำเข้า Excel)    |
|  - telemetry_logs/run1/run1_<ts>_plot.png (กราฟวิเคราะห์ 4 มิติ)               |
+-------------------------------------------------------------------------------+
```

---

## 📁 ไฟล์สำคัญในระบบ

- [src/sensor_pipeline.py](../src/sensor_pipeline.py): ฟิลเตอร์กรองสัญญาณ (Median, EMA, Outlier), ตัวแปลงสมการ [calibration.json](../calibration_output/calibration.json), โครงสร้าง `SensorHub` และ `SensorCollectorThread` (Thread 1)
- [src/robot_controller.py](../src/robot_controller.py): `RobotControllerThread` (Thread 2), รองรับการอ่านและรันคำสั่งจาก [data/robot_map_plan.json](../data/robot_map_plan.json)
- [src/telemetry.py](../src/telemetry.py): ตัวบันทึกข้อมูล Time-series และตัววิเคราะห์สถิติหลังรันเสร็จพร้อมพลอตรายงาน
- [src/robot_system.py](../src/robot_system.py): ตัวจัดการหลัก (Master Orchestrator) จัดการ Lifecycle ทั้งสอง Thread และ RoboMaster SDK
- [main.py](../main.py): Master CLI Runner สำหรับสั่งรันหุ่นจริง, โหมดจำลอง, ตรวจดูค่า Sensor สด, หรือวิเคราะห์ Log

---

## 🚀 วิธีการใช้งาน (Usage Guide)

### 1. ทดสอบรัน Simulation ด้วยแผนที่ `data/robot_map_plan.json`
ใช้ทดสอบความถูกต้องของ 2 Threads, แผนการเดิน, และการบันทึก Log โดยไม่ต้องเชื่อมต่อหุ่นจริง:
```cmd
python main.py simulate --plan data/robot_map_plan.json -y
```

### 2. รันหุ่นยนต์จริงกับ RoboMaster EP
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi AP ของ RoboMaster EP แล้วรัน:
```cmd
python main.py run --conn-type ap --plan data/robot_map_plan.json
```

### 3. มอนิเตอร์ค่า Sensor สดจาก Thread 1 (Live Monitor)
แสดงข้อมูลระยะ Sharp L/R, ToF, Yaw, และการตรวจจับกำแพงแบบ Real-time:
```cmd
python main.py monitor --conn-type ap
```
*(หรือทดสอบดูค่าแบบ mock: `python main.py monitor --mock`)*

### 4. วิเคราะห์และพลอตกราฟข้อมูลหลังการรัน (Post-run Analysis)
```cmd
python main.py analyze telemetry_logs/run1
```
