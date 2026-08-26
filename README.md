# 🤖 RoboMaster EP Autonomous Grid Navigation System

ระบบควบคุมหุ่นยนต์ **DJI RoboMaster EP** สำหรับการเคลื่อนที่อัตโนมัติแบบ **Grid Navigation (60x60 cm)** ด้วยสถาปัตยกรรม **Multi-Threading (2 Threads)** และระบบควบคุม **Closed-Loop PID Centering** ให้อยู่กึ่งกลางระหว่างกำแพงตามเงื่อนไขใน [REQ.md](REQ.md)

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
│   ├── telemetry.py              # ตัวบันทึกข้อมูล Time-series และวิเคราะห์สถิติพร้อมพลอตกราฟ
│   ├── map_planner.py            # GUI แผนที่จำลอง Grid + ระบบหาเส้นทาง A* (Pygame)
│   └── panel/                    # Step 4: RoboMaster Mission Control Center
│       ├── geometry.py           # แปลงพิกัดหุ่น (m/deg) <-> พิกัดแผนที่ (grid cell)
│       ├── occupancy.py          # Occupancy grid + กำแพงขอบช่อง + save/load JSON
│       ├── pathfinding.py        # A* + จัดลำดับ Checkpoint (TSP) + metrics
│       ├── robot_state.py        # RobotState มาตรฐาน, trail จริง, tracking timeout
│       ├── sensors.py            # Sensor Interface: ของจริง (SensorHub) / จำลอง (raycast)
│       ├── mapper.py             # อัลกอริทึม mapping ตัวเดียว ใช้ร่วมกันทั้ง 2 แหล่ง
│       ├── explorer.py           # Frontier-based exploration planner (Auto Map)
│       ├── robot_iface.py        # Real / Mock / Simulated Robot Interface
│       ├── simulation.py         # SimRobot + Ground-truth map + Simulation Engine
│       ├── mission.py            # Mission state machine + ระบบความปลอดภัยทั้งหมด
│       └── ui/                   # Dashboard (app.py, map_view.py, widgets.py, theme.py)
│
├── docs/                         # เอกสารอธิบายการทำงานแต่ละ Step โดยละเอียด
│   ├── STEP1_CALIBRATION.md      # คู่มือ Step 1: การ Calibrate เซนเซอร์และหาสมการ Polynomial
│   ├── STEP2_MULTITHREADING.md   # คู่มือ Step 2: สถาปัตยกรรม Multi-Threading (2 Threads)
│   ├── STEP3_PID.md              # คู่มือ Step 3: Closed-Loop PID Grid-by-Grid Navigation (8 Cases)
│   └── STEP4_MISSION_CONTROL.md  # คู่มือ Step 4: Mission Control Center (Panel)
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
├── telemetry_logs/               # บันทึกประวัติการรันจริง (.json, .csv, และกราฟวิเคราะห์ .png)
│
└── tests/                        # ชุด Automated Unit & Integration Tests (100% Pass)
    ├── __init__.py
    ├── test_calibration.py       # ทดสอบฟังก์ชัน Fitting สมการ Calibration
    ├── test_multithreading.py    # ทดสอบ Concurrency & Thread-safety ของ 2 Threads
    ├── test_step3_pid.py         # ทดสอบสมการ PID Centering และ 8 Wall Cases
    └── test_panel.py             # ทดสอบ Mission Control: แผนที่/A*/mapping/tracking/UI
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
.venv/bin/python main.py simulate --plan data/robot_map_plan.json
```

---

### 3. ทดสอบเดินในสนามจริงทีละ Grid (Step Test)
ใช้สำหรับนำหุ่นไปวางในสนามจริงเพื่อทดสอบระบบ PID ปรับบาลานซ์กึ่งกลางกำแพง:
```bash
# ทดสอบเดิน 1 ช่อง Grid (60 cm)
.venv/bin/python main.py step-test --cells 1 --conn-type ap

# ทดสอบเดิน 2 ช่อง Grid ต่อเนื่อง
.venv/bin/python main.py step-test --cells 2 --conn-type ap
```

---

### 4. รันหุ่นยนต์จริงเต็มรูปแบบ (Full Autonomous Run)
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi AP ของ RoboMaster EP แล้วสั่งรัน:
```bash
.venv/bin/python main.py run --conn-type ap --plan data/robot_map_plan.json
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
```bash
.venv/bin/python main.py analyze telemetry_logs/run_<timestamp>.json
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

### 8. RoboMaster Mission Control Center (Step 4)
แผงควบคุมภารกิจเต็มรูปแบบ: วาดแผนที่, A*, Auto-Mapping ด้วยเซนเซอร์บนตัวหุ่น,
simulation, และติดตามหุ่นจริงแบบ real-time
```bash
# โหมดจำลอง (ไม่แตะฮาร์ดแวร์)
.venv/bin/python main.py panel

# โหมดหุ่นจริง (ยังต้องกด CONNECT -> ARM -> RUN และยืนยันใน dialog ก่อนหุ่นจะขยับ)
.venv/bin/python main.py panel --mode real --conn-type ap

# โหลด/บันทึกแผนที่ของตัวเอง และตั้งขนาดช่อง Grid เอง
.venv/bin/python main.py panel --map data/my_field.json --cell-size 0.60
```
> **ในหน้าต่าง Panel**:
> - `MODE`: `SIM` / `REAL` / `MOCK` — สลับเป็น REAL ต้องยืนยันใน dialog เสมอ
> - `CONNECT → ARM → RUN` : หุ่นจริงไม่ขยับเองเด็ดขาด
> - `!! EMERGENCY STOP` (หรือ `F1`): อยู่มุมขวาบนตลอดเวลา
> - แถบล่างจัดกลุ่มเป็น: ขนาดแผนที่ / `TOOLS` / `EDIT` / `ROTATE MAP` / `ROTATE ROBOT` /
>   `FILE` / `PLAN` / `TURN ROBOT NOW` / `VIEW` / `SIM SPEED`
> - เครื่องมือแผนที่: Select / Wall / Eraser / Start / Goal / Checkpoint / Robot / Obstacle
>   (คลิกลากเพื่อวาดกำแพงยาว, คลิกขวาที่หุ่นเพื่อหมุนทิศเริ่มต้น)
> - `ROTATE MAP`: หมุนทั้งแผนที่ 90° (กำแพง/Start/Goal/Checkpoint/หุ่น ไปพร้อมกัน, 6×9 → 9×6)
> - `ROTATE ROBOT`: หมุนทิศเริ่มต้นบนแผนที่ | `TURN ROBOT NOW`: สั่งหุ่นจริงหมุนเดี๋ยวนี้ (ต้อง ARM)
> - เครื่องมือ `Place` + ปุ่ม `CARRY`: ให้หุ่นไปหนีบขวดที่ Goal แล้วเอาไปวางที่จุด `P`
>   โดยหันหน้าตามทิศที่กำหนด (คลิกซ้ำ/คลิกขวาที่จุด P เพื่อหมุนทิศ)
> - `AUTO MAP` เดินครบทุกช่องที่ไปถึงได้ แล้ววางเส้นทาง A* ให้อัตโนมัติ
> - `Space` = สร้างเส้นทาง A*, `AUTO MAP` = ให้หุ่นสำรวจแผนที่เอง
> - `R` / `Shift+R` = หมุนหุ่น, `[` / `]` = หมุนแผนที่
> - `Ctrl+S` / `Ctrl+L` = บันทึก / โหลดแผนที่, `1`–`8` = เลือกเครื่องมือ
>
>
> **รันแบบออฟไลน์**: ไม่ต้องใช้อินเทอร์เน็ต — ต่อ Wi-Fi หุ่น (`RMEP-xxxxxx`) แล้วเปิดได้เลย
> เปิดจาก terminal ของตัวเอง หรือดับเบิลคลิก `run_panel.bat`
> **ห้ามเปิด panel จากใน session ของเครื่องมืออื่น** — ถ้า session นั้นถูก kill ตอนหุ่นกำลังวิ่ง
> โปรเซสจะตายทั้งที่ความเร็วยังค้างในล้อ แล้วหุ่นจะวิ่งต่อจนชน
>
> รายละเอียดทั้งหมด: [docs/STEP4_MISSION_CONTROL.md](docs/STEP4_MISSION_CONTROL.md)

---

## 🧪 การรัน Automated Test Suite

รันชุดทดสอบความถูกต้องของระบบทั้งหมด:
```bash
.venv/bin/python -m unittest discover -s tests
```

---

## 📚 เอกสารประกอบฉบับเต็ม

- 📖 [docs/STEP1_CALIBRATION.md](docs/STEP1_CALIBRATION.md) — รายละเอียดการ Calibrate เซนเซอร์ Sharp/ToF/Gripper
- 📖 [docs/STEP2_MULTITHREADING.md](docs/STEP2_MULTITHREADING.md) — สถาปัตยกรรม Multi-threading (Producer-Consumer)
- 📖 [docs/STEP3_PID.md](docs/STEP3_PID.md) — รายละเอียดระบบ PID Centering และตาราง 8 Wall Decision Cases
- 📖 [docs/STEP4_MISSION_CONTROL.md](docs/STEP4_MISSION_CONTROL.md) — Mission Control Center: แผนที่, A*, Auto-Mapping, Simulation, Real-time Tracking, Safety
