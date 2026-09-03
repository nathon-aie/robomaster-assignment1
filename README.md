# 🤖 RoboMaster EP Autonomous Grid Navigation & Exploration System

ระบบควบคุมหุ่นยนต์ **DJI RoboMaster EP** สำหรับการเคลื่อนที่อัตโนมัติแบบ **Grid Navigation (60×60 cm)**, การสำรวจและสร้างแผนที่เขาวงกตอัตโนมัติ (**Autonomous Maze Exploration**), สถาปัตยกรรม **Multi-Threading (2 Threads)**, ระบบควบคุม **Closed-Loop PID Centering** ให้อยู่กึ่งกลางระหว่างกำแพง (8 Wall Decision Cases), และระบบ **Gripper Pick & Drop** ตามข้อกำหนดใน [REQ.md](REQ.md) และ [MAPPING.md](MAPPING.md)

---

## 👥 สมาชิกในกลุ่ม: ภัยพิบัติทั้ง 4 (PhaiPiBud_Thang_Si)
1. **นายคุณัชญ์ ทวีรัตน์** รหัสนักศึกษา 6810110038
2. **นายชัชนันท์ บุญส่ง** รหัสนักศึกษา 6810110055
3. **นายพลกฤต บัวลอย** รหัสนักศึกษา 6810110223
4. **นายศุภกิตต์ เชี่ยวหมอน** รหัสนักศึกษา 6810110354

---

## 🏗️ สถาปัตยกรรมระบบ (System Architecture)

ระบบทำงานด้วยโครงสร้าง **Multi-Threading (2 Threads)** เพื่อแยกกระบวนการประมวลผลเซนเซอร์และการควบคุมมอเตอร์/Actuators ออกจากกัน ป้องกันปัญหาความหน่วงและอาการค้างของสัญญาณ:

```mermaid
graph TD
    subgraph T1 [Thread 1: Sensor Pipeline / SensorHub]
        S_SharpL[Sharp IR Left (ID1, Port1)] -->|Raw ADC| Filter[Median Filter + EMA]
        S_SharpR[Sharp IR Right (ID2, Port2)] -->|Raw ADC| Filter
        S_ToF[ToF Distance Front] -->|Distance mm| Filter
        S_IMU[IMU Attitude / Yaw] -->|Yaw Angle| Filter
        S_Odo[Chassis Odometry] -->|Position x, y| Filter
        Filter --> SH[(SensorHub: Thread-safe Snapshot)]
    end

    subgraph T2 [Thread 2: Motion & Control]
        SH -->|Read State| PID[PID Centering Controller<br/>8 Wall Decision Cases]
        SH -->|Read State| EXP[Autonomous Grid Mapper / Explorer<br/>DFS + BFS Frontier Backtracking]
        PID --> Motion[Grid Movement 60x60 cm]
        EXP --> Motion
        Motion --> Actuator[Chassis, Robotic Arm & Gripper]
    end

    subgraph LOG [Telemetry & Analysis]
        SH -.-> TL[(Telemetry Logger)]
        Actuator -.-> TL
        TL --> Export[JSON / CSV Run Logs & Plot PNG]
    end
```

---

## 📂 โครงสร้างโปรเจกต์ (Project Structure)

```text
├── README.md                     # เอกสารรวมและคู่มือเริ่มต้นใช้งานฉบับสมบูรณ์
├── REQ.md                        # ข้อกำหนดและสเปกของโปรเจกต์
├── MAPPING.md                    # เอกสารรายละเอียดระบบ Autonomous Grid Mapping & Maze Exploration
├── WORKFLOW.md                   # แผนภาพและขั้นตอนการทำงานของระบบ
├── REPORT_3.md                   # รายงานสรุปผลการทดลองและการทำงาน
├── requirements.txt              # รายการ Python dependencies (Python 3.8)
├── main.py                       # Master CLI Entry Point (รวมทุกคำสั่งไว้ในที่เดียว)
│
├── src/                          # ซอร์สโค้ดหลักของระบบ (Core Package)
│   ├── __init__.py
│   ├── calibrate.py              # โมดูล Calibrate เซนเซอร์ Sharp / ToF / Gripper (Step 1)
│   ├── sensor_pipeline.py        # Thread 1: กรองสัญญาณ (Median, EMA), SensorHub, Thread-safe State (Step 2)
│   ├── pid_controller.py         # Step 3: PID Centering Controller จำแนก 8 Wall Cases
│   ├── robot_controller.py       # Thread 2: ควบคุมการเคลื่อนที่ทีละ Grid และ Actuators
│   ├── robot_system.py           # ตัวควบคุมหลัก (Master Orchestrator) จัดการ 2 Threads & SDK Lifecycle
│   ├── gripper_controller.py     # โมดูลควบคุมแขนกลและ Gripper ลำดับ Pick & Drop (Step 4)
│   ├── grid_mapper.py            # โมดูล Grid Mapping & Autonomous Maze Exploration (DFS + Backtracking)
│   ├── telemetry.py              # ตัวบันทึกข้อมูล Time-series แยกโฟลเดอร์รัน และวิเคราะห์สถิติ
│   └── map_planner.py            # GUI แผนที่จำลอง Grid + ระบบหาเส้นทาง A* (Pygame) (Step 5)
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
│   ├── robot_map_plan.json       # แผนที่และลำดับคำสั่งที่ Export มาจาก map_planner.py
│   └── discovered_map.json       # แผนที่ที่ได้จากการสำรวจเขาวงกตอัตโนมัติ
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

โปรเจกต์นี้รองรับและแนะนำให้ใช้ **Python 3.8**:

```bash
# 1. สร้าง Virtual Environment
python3 -m venv .venv

# 2. เปิดใช้งาน Virtual Environment
# Linux / macOS:
source .venv/bin/activate
# Windows:
# .venv\Scripts\activate

# 3. ติดตั้ง Dependencies
python -m pip install --upgrade pip
pip install -r requirements.txt
```

---

## 🚀 คู่มือการใช้งานผ่าน `main.py` (Master CLI)

ระบบรวมทุกคำสั่งไว้ใน `main.py` เพื่อความสะดวกในการใช้งาน:

### 1. วาดแผนที่และสร้างเส้นทางเดิน (Map Planner GUI)
เปิดโปรแกรม Pygame เพื่อวาดกำแพง กำหนดจุด Start/Goal และหาเส้นทางด้วย A*:
```bash
python main.py map
```
> **คีย์ลัดในหน้าต่าง Map Planner**:
> - `คลิกซ้าย`: เพิ่ม/ลบกำแพง | `S + คลิก`: วางจุด Start | `G + คลิก`: วางจุด Goal
> - `Spacebar`: หาเส้นทางที่สั้นที่สุด (Shortest Path)
> - `T`: หาเส้นทางที่เลี้ยวน้อยที่สุด (Min Turns)
> - `K`: บันทึกแผนที่และชุดคำสั่งลง `data/robot_map_plan.json`
> - `L`: โหลดแผนที่จาก JSON | `C`: ล้างแผนที่

---

### 2. ทดสอบในโหมดจำลอง (Simulation / Dry-Run)
ทดสอบระบบ 2 Threads + Step 3 PID เดินตามแผนที่จำลองโดยไม่ต้องต่อหุ่นจริง:
```bash
# 2.1 รันจำลองแบบปกติ (มีขั้นตอนถามยืนยัน Pick -> เดิน -> Drop)
python main.py simulate --plan data/robot_map_plan.json

# 2.2 รันจำลองแบบ Auto-confirm ทั้งหมด
python main.py simulate --plan data/robot_map_plan.json -y

# 2.3 รันจำลองแบบข้ามขั้นตอนคีบและวาง (ทดสอบการเดินตามแผนที่อย่างเดียว)
python main.py simulate --plan data/robot_map_plan.json --skip-pick --skip-drop -y
```

---

### 3. ทดสอบเดินและหมุนในสนามจริงทีละ Grid (Step & Turn Test)
ใช้สำหรับนำหุ่นไปวางในสนามจริงเพื่อทดสอบระบบ PID ปรับบาลานซ์กึ่งกลางกำแพง และทดสอบการเลี้ยว:
```bash
# ทดสอบเดินหน้า 1 ช่อง Grid (60 cm)
python main.py step-test --cells 1 --conn-type ap

# ทดสอบเดินหน้า 2 ช่อง Grid ต่อเนื่อง
python main.py step-test --cells 2 --conn-type ap

# ทดสอบการหมุนตัว (เลี้ยวขวา 90 องศา, เลี้ยวซ้าย, กลับหลัง 180 องศา)
python main.py turn-test --direction right --conn-type ap
python main.py turn-test --direction left --conn-type ap
python main.py turn-test --direction around --conn-type ap
```

---

### 4. รันหุ่นยนต์จริงเต็มรูปแบบ (Full Autonomous Run)
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi AP ของ RoboMaster EP แล้วสั่งรัน:
```bash
# 4.1 รันเต็มรูปแบบ: คีบของ -> เดินตามแผนที่ด้วย PID -> วางของ
python main.py run --conn-type ap --plan data/robot_map_plan.json

# 4.2 รันแบบเดินตามแผนที่อย่างเดียว (ข้าม Pick & Drop)
python main.py run --conn-type ap --plan data/robot_map_plan.json --skip-pick --skip-drop -y
```
*(มีระบบ Emergency Stop: กด `Ctrl + C` ได้ตลอดเวลา หุ่นยนต์จะหยุดการเคลื่อนที่ทันทีและบันทึก Log)*

---

### 5. สำรวจเขาวงกตและสร้างแผนที่อัตโนมัติ (Autonomous Maze Exploration)
หุ่นยนต์จะเคลื่อนที่สำรวจเขาวงกตที่ไม่เคยรู้แผนที่ล่วงหน้าด้วยตัวเอง (DFS + Frontier Backtracking) และ Export แผนที่ออกมาเป็นไฟล์ JSON:
```bash
# 5.1 รันจำลองการสำรวจแบบ Mock Simulation
python main.py explore --mock --sim-maze data/robot_map_plan.json --output data/discovered_map.json

# 5.2 รันสำรวจบนหุ่นยนต์จริง
python main.py explore --conn-type ap --output data/discovered_map.json --start-col 0 --start-row 3
```

---

### 6. วาดรูปกราฟิกแผนที่จากไฟล์ JSON (Plot Map Image)
สร้างรูปภาพ `.png` ของแผนที่จากไฟล์ JSON เพื่อดูโครงสร้างเขาวงกต:
```bash
python main.py plot-map data/discovered_map.json --output data/discovered_map.png
```

---

### 7. มอนิเตอร์ค่าเซนเซอร์สดจาก Thread 1 (Live Monitor)
ดูค่าระยะ Sharp Left/Right (mm), ToF (mm), Yaw (deg), และสถานะกำแพงแบบ Real-time:
```bash
python main.py monitor --conn-type ap
```

---

### 8. วิเคราะห์สถิติและพลอตกราฟหลังรัน (Post-run Analysis)
ข้อมูลการรันจะถูกบันทึกลงในโฟลเดอร์ `telemetry_logs/run1/`, `telemetry_logs/run2/` อัตโนมัติ:
```bash
# วิเคราะห์โดยระบุชื่อโฟลเดอร์รันโดยตรง
python main.py analyze telemetry_logs/run1

# หรือระบุไฟล์ JSON โดยตรง
python main.py analyze telemetry_logs/run1/run1_20260831_120000.json
```

---

### 9. เครื่องมือ Calibrate เซนเซอร์ (Step 1)
```bash
# 9.1 เก็บค่าจากหุ่นจริงลง CSV
python main.py calibrate collect-live sharp_left --board-id 1 --port 1 --conn-type ap
python main.py calibrate collect-live sharp_right --board-id 2 --port 2 --conn-type ap
python main.py calibrate collect-live tof --tof-index 0 --conn-type ap

# 9.2 คำนวณสมการ Polynomial และบันทึก calibration.json พร้อมกราฟ
python main.py calibrate fit data/calibration_measurements.csv
```

---

## 🧠 ตารางการตัดสินใจระบบควบคุม PID (8 Wall Decision Cases)

| เคส | กำแพงหน้า (ToF) | กำแพงข้าง (Sharp) | เกณฑ์ตรวจจับ | กลยุทธ์การควบคุม PID Centering |
| :---: | :---: | :---: | :---: | :--- |
| **Case 1** | มี ($< 500\text{ mm}$) | มีทั้งสองข้าง | $\|L - R\| < 20\text{ mm}$ | ปรับความเร็วแกนขวาง $V_y$ ตามผลต่าง $(L - R)$ ให้อยู่กึ่งกลาง + ชะลอหยุดตามระยะ ToF |
| **Case 2** | มี ($< 500\text{ mm}$) | มีเฉพาะซ้าย | $\|L - 140\| < 20\text{ mm}$ | ปรับ $V_y$ รักษาระยะซ้าย $140\text{ mm}$ + ชะลอหยุดตามระยะ ToF |
| **Case 3** | มี ($< 500\text{ mm}$) | มีเฉพาะขวา | $\|R - 140\| < 20\text{ mm}$ | ปรับ $V_y$ รักษาระยะขวา $140\text{ mm}$ + ชะลอหยุดตามระยะ ToF |
| **Case 4** | มี ($< 500\text{ mm}$) | ไม่มีกำแพงข้าง | ไม่มีข้อมูลข้าง | ล็อกทิศทาง Yaw ตรงหน้า ($W_z$) และเคลื่อนที่ $60\text{ cm}$ พร้อมตรวจวัดระยะ ToF |
| **Case 5** | ไม่มี ($\ge 500\text{ mm}$) | มีทั้งสองข้าง | $\|L - R\| < 20\text{ mm}$ | ปรับความเร็วแกนขวาง $V_y$ ตามผลต่าง $(L - R)$ ให้อยู่กึ่งกลางตลอดการวิ่ง $60\text{ cm}$ |
| **Case 6** | ไม่มี ($\ge 500\text{ mm}$) | มีเฉพาะซ้าย | $\|L - 140\| < 20\text{ mm}$ | ปรับ $V_y$ รักษาระยะซ้าย $140\text{ mm}$ ตลอดการวิ่ง $60\text{ cm}$ |
| **Case 7** | ไม่มี ($\ge 500\text{ mm}$) | มีเฉพาะขวา | $\|R - 140\| < 20\text{ mm}$ | ปรับ $V_y$ รักษาระยะขวา $140\text{ mm}$ ตลอดการวิ่ง $60\text{ cm}$ |
| **Case 8** | ไม่มี ($\ge 500\text{ mm}$) | ไม่มีกำแพงข้าง | ไม่มีข้อมูลข้าง | เดินหน้าตรง $60\text{ cm}$ ด้วย Odometry + ล็อก Heading ด้วย IMU Yaw |

---

## 📚 เอกสารประกอบฉบับเต็ม

- 📖 [REQ.md](REQ.md) — ข้อกำหนดและสเปกโจทย์ระบบ RoboMaster EP
- 📖 [CLASSWORK.md](CLASSWORK.md) — รายงานและทฤษฎีเปรียบเทียบ Occupancy Grid Mapping (OGM)
- 📖 [MAPPING.md](MAPPING.md) — คู่มือโมดูล Autonomous Grid Mapping และการสำรวจเขาวงกตอัตโนมัติ
- 📖 [WORKFLOW.md](WORKFLOW.md) — รายละเอียดสถาปัตยกรรมและ Flow การทำงานของระบบ
- 📖 [REPORT_3.md](REPORT_3.md) — รายงานสรุปผลการทดลองและสถิติ Telemetry
- 📖 [docs/STEP1_CALIBRATION.md](docs/STEP1_CALIBRATION.md) — รายละเอียดการ Calibrate เซนเซอร์ Sharp/ToF/Gripper
- 📖 [docs/STEP2_MULTITHREADING.md](docs/STEP2_MULTITHREADING.md) — สถาปัตยกรรม Multi-threading (Producer-Consumer)
- 📖 [docs/STEP3_PID.md](docs/STEP3_PID.md) — รายละเอียดระบบ PID Centering และตาราง 8 Wall Decision Cases
- 📖 [docs/STEP4_GRIPPER.md](docs/STEP4_GRIPPER.md) — ลำดับการควบคุมแขนกลและกริปเปอร์ Pick & Drop
- 📖 [docs/STEP5_MAP_AND_PATHING.md](docs/STEP5_MAP_AND_PATHING.md) — แผนที่สภาพแวดล้อม A* Pathfinding และการพล็อตวิเคราะห์เส้นทางใน Jupyter

