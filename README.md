# 🤖 RoboMaster EP Autonomous Grid Navigation System

ระบบควบคุมหุ่นยนต์ **DJI RoboMaster EP** สำหรับการเคลื่อนที่อัตโนมัติแบบ **Grid Navigation (60x60 cm)** ด้วยสถาปัตยกรรม **Multi-Threading (2 Threads)**, ระบบควบคุม **Closed-Loop PID Centering** ให้อยู่กึ่งกลางระหว่างกำแพง (8 Wall Cases), และระบบ **Gripper Pick & Drop** ตามข้อกำหนดใน [REQ.md](REQ.md)

---

# กลุ่ม ภัยพิบัติทั้ง 4 (PhaiPiBud_Thang_Si)
## 👥 สมาชิกในกลุ่ม
1. **นายคุณัชญ์ ทวีรัตน์** รหัสนักศึกษา 6810110038
2. **นายชัชนันท์ บุญส่ง** รหัสนักศึกษา 6810110055
3. **นายพลกฤต บัวลอย** รหัสนักศึกษา 6810110223
4. **นายศุภกิตต์ เชี่ยวหมอน** รหัสนักศึกษา 6810110354

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
│   ├── STEP3_PID.md              # คู่มือ Step 3: Closed-Loop PID Grid-by-Grid Navigation (8 Cases)
│   ├── STEP4_GRIPPER.md          # คู่มือ Step 4: ลำดับการควบคุมแขนกลและกริปเปอร์ Pick & Drop
│   └── STEP5_MAP_AND_PATHING.md  # คู่มือ Step 5: แผนที่ A* Pathfinding และ Pathing Overlay Analysis
│
├── analyze/                      # โฟลเดอร์วิเคราะห์และพล็อตเส้นทางเดิน
│   └── pathing.ipynb             # สมุดบันทึก Jupyter วิเคราะห์และพล็อตเส้นทางจริงทับแผนที่
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
└── telemetry_logs/               # บันทึกประวัติการรันจริงแยกโฟลเดอร์ตามรอบรันอัตโนมัติ (run1/, run2/, ...)
    └── run*/                     # แต่ละรอบเก็บ run*_<timestamp>.json, run*_<timestamp>.csv, run*_<timestamp>_plot.png
```

---

## ⚙️ การติดตั้งและตั้งค่า Environment (Setup)

โปรเจกต์นี้รองรับ **Python 3.8**:

```cmd
# 1. สร้าง Virtual Environment
python -m venv .venv

# 2. ติดตั้ง Dependencies
python -m pip install --upgrade pip
pip install -r requirements.txt
```

---

## 🚀 คู่มือการใช้งานด่วนผ่าน `main.py` (Master CLI)

ระบบมี Master CLI ตัวเดียวที่เรียกใช้งานฟังก์ชันทั้งหมดได้อย่างสะดวก:

### 1. วาดแผนที่และสร้างเส้นทางเดิน (Map Planner GUI)
เปิดโปรแกรม Pygame เพื่อวาดกำแพง กำหนดจุด Start/Goal และหาเส้นทางด้วย A*:
```cmd
python main.py map
```
> **คีย์ลัดในหน้าต่าง Map**:
> - `คลิกซ้าย`: เพิ่ม/ลบกำแพง | `S + คลิก`: วางจุด Start | `G + คลิก`: วางจุด Goal
> - `Spacebar`: หาเส้นทางที่สั้นที่สุด (Shortest Path)
> - `T`: หาเส้นทางที่เลี้ยวน้อยที่สุด (Min Turns)
> - `K`: บันทึกแผนที่และชุดคำสั่งลง `data\robot_map_plan.json`
> - `L`: โหลดแผนที่จาก JSON | `C`: ล้างแผนที่

---

### 2. ทดสอบในโหมดจำลอง (Simulation / Dry-Run)
ทดสอบระบบ 2 Threads + Step 3 PID เดินตามแผนที่จำลองโดยไม่ต้องต่อหุ่นจริง:
```cmd
# 2.1 รันจำลองแบบปกติ (มีลำดับถามยืนยัน Pick -> เดิน -> Drop)
python main.py simulate --plan data\robot_map_plan.json

# 2.2 รันจำลองแบบ Auto-confirm ทั้งหมด
python main.py simulate --plan data\robot_map_plan.json -y

# 2.3 รันจำลองแบบข้ามขั้นตอนคีบและวาง (ทดสอบการเดินตามแผนที่อย่างเดียว)
python main.py simulate --plan data\robot_map_plan.json --skip-pick --skip-drop -y
```

---

### 3. ทดสอบเดินและหมุนในสนามจริงทีละ Grid (Step & Turn Test)
ใช้สำหรับนำหุ่นไปวางในสนามจริงเพื่อทดสอบระบบ PID ปรับบาลานซ์กึ่งกลางกำแพง และทดสอบการเลี้ยว:
```cmd
# ทดสอบเดินหน้า 1 ช่อง Grid (60 cm)
python main.py step-test --cells 1

# ทดสอบเดินหน้า 2 ช่อง Grid ต่อเนื่อง
python main.py step-test --cells 2

# ทดสอบการหมุนตัว (เลี้ยวขวา 90 องศา, เลี้ยวซ้าย, กลับหลัง)
python main.py turn-test --direction right
python main.py turn-test --direction left
python main.py turn-test --direction around
```

---

### 4. รันหุ่นยนต์จริงเต็มรูปแบบ (Full Autonomous Run)
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi AP ของ RoboMaster EP แล้วสั่งรัน:
```cmd
# 4.1 รันเต็มรูปแบบ: คีบของ -> เดินตามแผนที่ด้วย PID -> วางของ
python main.py run --plan data\robot_map_plan.json

# 4.2 รันแบบเดินตามแผนที่อย่างเดียว (ข้าม Pick & Drop)
python main.py run --plan data\robot_map_plan.json --skip-pick --skip-drop -y
```
*(มี Emergency Stop ปลอดภัย กด `Ctrl + C` ได้ตลอดเวลา)*

---

### 5. มอนิเตอร์ค่าเซนเซอร์สดจาก Thread 1 (Live Monitor)
ดูค่าระยะ Sharp Left/Right (mm), ToF (mm), Yaw (deg), และสถานะกำแพงแบบ Real-time:
```cmd
python main.py monitor
```

---

### 6. วิเคราะห์สถิติและพลอตกราฟหลังรัน (Post-run Analysis)
ข้อมูลการรันจะถูกบันทึกลงในโฟลเดอร์ `telemetry_logs\run1\`, `telemetry_logs\run2/` อัตโนมัติ:
```cmd
# วิเคราะห์โดยระบุชื่อโฟลเดอร์รันโดยตรง
python main.py analyze telemetry_logs\run*

# หรือระบุไฟล์ JSON โดยตรง
python main.py analyze telemetry_logs\run*\run*_*_*.json
```

---

### 7. เครื่องมือ Calibrate เซนเซอร์ (Step 1)
```cmd
# เก็บค่าจากหุ่นจริงลง CSV
python main.py calibrate collect-live sharp_left --board-id 1 --port 1
python main.py calibrate collect-live sharp_right --board-id 2 --port 2
python main.py calibrate collect-live tof --tof-index 0

# คำนวณสมการ Polynomial และเซฟ calibration.json พร้อมกราฟ
python main.py calibrate fit data\calibration_measurements.csv
```

---

## 📚 เอกสารประกอบฉบับเต็ม

- 📖 [docs/STEP1_CALIBRATION.md](docs/STEP1_CALIBRATION.md) — รายละเอียดการ Calibrate เซนเซอร์ Sharp/ToF/Gripper
- 📖 [docs/STEP2_MULTITHREADING.md](docs/STEP2_MULTITHREADING.md) — สถาปัตยกรรม Multi-threading (Producer-Consumer)
- 📖 [docs/STEP3_PID.md](docs/STEP3_PID.md) — รายละเอียดระบบ PID Centering และตาราง 8 Wall Decision Cases
- 📖 [docs/STEP4_GRIPPER.md](docs/STEP4_GRIPPER.md) — ลำดับการควบคุมแขนกลและกริปเปอร์ Pick & Drop
- 📖 [docs/STEP5_MAP_AND_PATHING.md](docs/STEP5_MAP_AND_PATHING.md) — แผนที่สภาพแวดล้อม A* Pathfinding และการพล็อตวิเคราะห์เส้นทางใน Jupyter
