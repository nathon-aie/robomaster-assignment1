# 🤖 RoboMaster EP Autonomous Grid Navigation System

ระบบควบคุมหุ่นยนต์ **DJI RoboMaster EP** สำหรับการเคลื่อนที่อัตโนมัติแบบ **Grid Navigation (60x60 cm)** ด้วยสถาปัตยกรรม **Multi-Threading (2 Threads)**, ระบบควบคุม **Closed-Loop PID Centering** ให้อยู่กึ่งกลางระหว่างกำแพง (8 Wall Cases), และระบบ **Gripper Pick & Drop** ตามข้อกำหนดใน [REQ.md](REQ.md)

---

## 📂 โครงสร้างโปรเจกต์ (Project Structure)

```text
├── README.md                     # เอกสารรวมและคู่มือเริ่มต้นใช้งาน (Quickstart)
├── REQ.md                        # ข้อกำหนดและสเปกของโปรเจกต์
├── requirements.txt              # รายการ Python dependencies (Python 3.8)
├── main.py                       # Master CLI Entry Point (รวมทุกคำสั่งไว้ในที่เดียว)
│
├── src/                          # ซอร์สโค้ดหลักของระบบ (Core Package)
│   ├── __init__.py
│   ├── calibrate.py              # โมดูล Calibrate เซนเซอร์ Sharp / ToF / Gripper (Step 1)
│   ├── sensor_pipeline.py        # Thread 1: กรองสัญญาณ (Median, EMA), SensorHub, Shared State
│   ├── pid_controller.py         # Step 3: PID Centering Controller จำแนก 8 Wall Cases
│   ├── robot_controller.py       # Thread 2: ควบคุมการเคลื่อนที่ทีละ Grid และ Actuators
│   ├── robot_system.py           # ตัวควบคุมหลัก (Master Orchestrator) จัดการ 2 Threads & SDK
│   ├── gripper_controller.py     # โมดูลควบคุมแขนกลและ Gripper ลำดับ Pick & Drop (Step 4)
│   ├── telemetry.py              # ตัวบันทึกข้อมูล Time-series แยกโฟลเดอร์รัน และวิเคราะห์สถิติ
│   └── map_planner.py            # GUI แผนที่จำลอง Grid + ระบบหาเส้นทาง A* (Pygame)
│
├── docs/                         # เอกสารอธิบายการทำงานแต่ละ Step โดยละเอียด
│   ├── STEP1_CALIBRATION.md      # คู่มือ Step 1: การ Calibrate เซนเซอร์และหาสมการ Polynomial
│   ├── STEP2_MULTITHREADING.md   # คู่มือ Step 2: สถาปัตยกรรม Multi-Threading (2 Threads)
│   └── STEP3_PID.md              # คู่มือ Step 3: Closed-Loop PID Grid-by-Grid Navigation (8 Cases)
│
├── data/                         # ข้อมูล Dataset และแผนที่
│   ├── calibration_measurements.csv # ข้อมูลผลการวัดเพื่อใช้ Fitting สมการ
│   └── robot_map_plan.json       # แผนที่และลำดับคำสั่งที่ Export มาจาก map_planner.py
│
├── calibration_output/           # ผลลัพธ์สมการ Calibration (.json) และกราฟฟิตติ้ง (.png)
│   ├── calibration.json
│   ├── sharp_left_calibration.png
│   └── sharp_right_calibration.png
│
├── telemetry_logs/               # บันทึกประวัติการรันจริงแยกโฟลเดอร์ตามรอบรันอัตโนมัติ (run1/, run2/, ...)
│   └── run1/                     # แต่ละรอบเก็บ run1_<timestamp>.json, run1_<timestamp>.csv, run1_<timestamp>_plot.png
│
└── tests/                        # ชุด Automated Unit & Integration Tests (21/21 Pass 100%)
    ├── __init__.py
    ├── test_calibration.py       # ทดสอบฟังก์ชัน Fitting สมการ Calibration
    ├── test_multithreading.py    # ทดสอบ Concurrency, Thread-safety, และการสร้างโฟลเดอร์ Telemetry
    └── test_step3_pid.py         # ทดสอบสมการ PID Centering และ 8 Wall Cases
```

---

## ⚙️ การติดตั้งและตั้งค่า Environment (Setup)

โปรเจกต์นี้รองรับ **Python 3.8**:

```bash
# 1. สร้าง Virtual Environment
python3.8 -m venv .venv

# 2. ติดตั้ง Dependencies
.venv/bin/python -m pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
```

---

## 🚀 คู่มือการใช้งานด่วนผ่าน `main.py` (Master CLI)

ระบบมี Master CLI ตัวเดียวที่เรียกใช้งานฟังก์ชันทั้งหมดได้อย่างสะดวก:

### 1. วาดแผนที่และสร้างเส้นทางเดิน (Map Planner GUI)
เปิดโปรแกรม Pygame เพื่อวาดกำแพง กำหนดจุด Start/Goal และหาเส้นทางด้วย A*:
```bash
.venv/bin/python main.py map
```
> **คีย์ลัดในหน้าต่าง Map**:
> - `คลิกซ้าย`: เพิ่ม/ลบกำแพง | `S + คลิก`: วางจุด Start | `G + คลิก`: วางจุด Goal
> - `Spacebar`: หาเส้นทางที่สั้นที่สุด (Shortest Path)
> - `T`: หาเส้นทางที่เลี้ยวน้อยที่สุด (Min Turns)
> - `K`: บันทึกแผนที่และชุดคำสั่งลง `data/robot_map_plan.json`
> - `L`: โหลดแผนที่จาก JSON | `C`: ล้างแผนที่

---

### 2. ทดสอบในโหมดจำลอง (Simulation / Dry-Run)
ทดสอบระบบ 2 Threads + Step 3 PID เดินตามแผนที่จำลองโดยไม่ต้องต่อหุ่นจริง:
```bash
# 2.1 รันจำลองแบบปกติ (มีลำดับถามยืนยัน Pick -> เดิน -> Drop)
.venv/bin/python main.py simulate --plan data/robot_map_plan.json

# 2.2 รันจำลองแบบ Auto-confirm ทั้งหมด
.venv/bin/python main.py simulate --plan data/robot_map_plan.json -y

# 2.3 รันจำลองแบบข้ามขั้นตอนคีบและวาง (ทดสอบการเดินตามแผนที่อย่างเดียว)
.venv/bin/python main.py simulate --plan data/robot_map_plan.json --skip-pick --skip-drop -y
```

---

### 3. ทดสอบเดินและหมุนในสนามจริงทีละ Grid (Step & Turn Test)
ใช้สำหรับนำหุ่นไปวางในสนามจริงเพื่อทดสอบระบบ PID ปรับบาลานซ์กึ่งกลางกำแพง และทดสอบการเลี้ยว:
```bash
# ทดสอบเดินหน้า 1 ช่อง Grid (60 cm)
.venv/bin/python main.py step-test --cells 1 --conn-type ap

# ทดสอบเดินหน้า 2 ช่อง Grid ต่อเนื่อง
.venv/bin/python main.py step-test --cells 2 --conn-type ap

# ทดสอบการหมุนตัว (เลี้ยวขวา 90 องศา, เลี้ยวซ้าย, กลับหลัง)
.venv/bin/python main.py turn-test --direction right --conn-type ap
.venv/bin/python main.py turn-test --direction left --conn-type ap
.venv/bin/python main.py turn-test --direction around --conn-type ap
```

---

### 4. รันหุ่นยนต์จริงเต็มรูปแบบ (Full Autonomous Run)
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi AP ของ RoboMaster EP แล้วสั่งรัน:
```bash
# 4.1 รันเต็มรูปแบบ: คีบของ -> เดินตามแผนที่ด้วย PID -> วางของ
.venv/bin/python main.py run --conn-type ap --plan data/robot_map_plan.json

# 4.2 รันแบบเดินตามแผนที่อย่างเดียว (ข้าม Pick & Drop)
.venv/bin/python main.py run --conn-type ap --plan data/robot_map_plan.json --skip-pick --skip-drop -y
```
*(มี Emergency Stop ปลอดภัย กด `Ctrl + C` ได้ตลอดเวลา)*

---

### 5. มอนิเตอร์ค่าเซนเซอร์สดจาก Thread 1 (Live Monitor)
ดูค่าระยะ Sharp Left/Right (mm), ToF (mm), Yaw (deg), และสถานะกำแพงแบบ Real-time:
```bash
.venv/bin/python main.py monitor --conn-type ap
```

---

### 6. วิเคราะห์สถิติและพลอตกราฟหลังรัน (Post-run Analysis)
ข้อมูลการรันจะถูกบันทึกลงในโฟลเดอร์ `telemetry_logs/run1/`, `telemetry_logs/run2/` อัตโนมัติ:
```bash
# วิเคราะห์โดยระบุชื่อโฟลเดอร์รันโดยตรง
.venv/bin/python main.py analyze telemetry_logs/run1

# หรือระบุไฟล์ JSON โดยตรง
.venv/bin/python main.py analyze telemetry_logs/run1/run1_20260826_133759.json
```

---

### 7. เครื่องมือ Calibrate เซนเซอร์ (Step 1)
```bash
# เก็บค่าจากหุ่นจริงลง CSV
.venv/bin/python main.py calibrate collect-live sharp_left --board-id 1 --port 1
.venv/bin/python main.py calibrate collect-live sharp_right --board-id 2 --port 2
.venv/bin/python main.py calibrate collect-live tof --tof-index 0

# คำนวณสมการ Polynomial และเซฟ calibration.json พร้อมกราฟ
.venv/bin/python main.py calibrate fit data/calibration_measurements.csv
```

---

## 🧪 การรัน Automated Test Suite (21/21 Passed)

รันชุดทดสอบความถูกต้องของระบบทั้งหมด:
```bash
.venv/bin/python -m unittest discover -s tests
```

---

## 📚 เอกสารประกอบฉบับเต็ม

- 📖 [docs/STEP1_CALIBRATION.md](docs/STEP1_CALIBRATION.md) — รายละเอียดการ Calibrate เซนเซอร์ Sharp/ToF/Gripper
- 📖 [docs/STEP2_MULTITHREADING.md](docs/STEP2_MULTITHREADING.md) — สถาปัตยกรรม Multi-threading (Producer-Consumer)
- 📖 [docs/STEP3_PID.md](docs/STEP3_PID.md) — รายละเอียดระบบ PID Centering และตาราง 8 Wall Decision Cases
