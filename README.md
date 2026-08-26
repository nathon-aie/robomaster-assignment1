# 🤖 RoboMaster EP Autonomous Grid Navigation System

ระบบควบคุมหุ่นยนต์ **DJI RoboMaster EP** สำหรับการเคลื่อนที่อัตโนมัติแบบ **Grid Navigation (60x60 cm)** ด้วยสถาปัตยกรรม **Multi-Threading (2 Threads)**, ระบบควบคุม **Closed-Loop PID Centering** ให้อยู่กึ่งกลางระหว่างกำแพง (8 Wall Cases), และระบบ **Gripper Pick & Drop** ตามข้อกำหนดใน [REQ.md](REQ.md)

---

## 📂 โครงสร้างโปรเจกต์ (Project Structure)

```text
├── run                           # 🚀 Executable CLI Quick Launcher
├── main.py                       # 🎯 Master CLI Entry Point (รวมทุกคำสั่งไว้ในที่เดียว)
├── README.md                     # 📖 คู่มือการใช้งานฉบับสมบูรณ์
├── REQ.md                        # 📋 เอกสารสเปกและข้อกำหนด
├── config/                       # ⚙️ การตั้งค่าระบบทั้งหมด (Settings)
│   └── settings.yaml             # ไฟล์รวมค่าเริ่มต้น PID, Sensor, Gripper, Grid ทั้งหมด (แก้ที่นี่ที่เดียว)
│
├── src/                          # 🧠 ซอร์สโค้ดหลักของระบบ (Core Package)
│   ├── __init__.py
│   ├── config_loader.py          # ตัวโหลดการตั้งค่าจาก settings.yaml พร้อม fallback defaults
│   ├── robot_system.py           # ตัวควบคุมหลัก (Master Orchestrator) จัดการ 2 Threads & SDK
│   ├── robot_controller.py       # Thread 2: ควบคุมการเคลื่อนที่ทีละ Grid และ Actuators
│   ├── sensor_pipeline.py        # Thread 1: กรองสัญญาณ (Median, EMA), SensorHub, Shared State
│   ├── pid_controller.py         # Step 3: PID Centering Controller จำแนก 8 Wall Cases
│   ├── gripper_controller.py     # โมดูลควบคุมแขนกลและ Gripper ลำดับ Pick & Drop
│   ├── telemetry.py              # ตัวบันทึกข้อมูล Time-series แยกโฟลเดอร์รัน และวิเคราะห์สถิติ
│   ├── map_planner.py            # GUI แผนที่จำลอง Grid + ระบบหาเส้นทาง A* (Pygame)
│   └── calibrate.py              # โมดูล Calibrate เซนเซอร์ Sharp IR ซ้าย-ขวา (Step 1)
│
├── docs/                         # 📚 เอกสารอธิบายการทำงานแต่ละ Step โดยละเอียด
│   ├── STEP1_CALIBRATION.md      # คู่มือ Step 1: การ Calibrate เซนเซอร์และหาสมการ Polynomial
│   ├── STEP2_MULTITHREADING.md   # คู่มือ Step 2: สถาปัตยกรรม Multi-Threading (2 Threads)
│   ├── STEP3_PID.md              # คู่มือ Step 3: Closed-Loop PID Grid-by-Grid Navigation (8 Cases)
│   ├── STEP4_GRIPPER.md          # คู่มือ Step 4: ระบบแขนกลและ Gripper ลำดับ Pick & Drop
│   └── STEP5_MAP_PLANNER.md      # คู่มือ Step 5: การวางแผนที่และคำนวณเส้นทาง A* (robot_map_plan.json)
│
├── data/                         # 🗂️ ข้อมูล Dataset และแผนที่
│   ├── calibration_measurements.csv # ข้อมูลผลการวัดเพื่อใช้ Fitting สมการ
│   └── robot_map_plan.json       # แผนที่และลำดับคำสั่งที่ Export มาจาก map_planner.py
│
├── calibration_output/           # 📈 ผลลัพธ์สมการ Calibration (.json) และกราฟฟิตติ้ง (.png)
│   ├── calibration.json
│   ├── sharp_left_calibration.png
│   └── sharp_right_calibration.png
│
└── telemetry_logs/               # 📊 บันทึกประวัติการรันจริงแยกโฟลเดอร์ตามรอบรันอัตโนมัติ (run1/, run2/, ...)
    └── run1/                     # แต่ละรอบเก็บ run1_<timestamp>.json, run1_<timestamp>.csv, run1_<timestamp>_plot.png
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

## 🚀 คู่มือการใช้งานด่วน (Quick CLI Reference)

ระบบมี Short Aliases และตัวช่วยรัน `./run` (หรือเรียกผ่าน `python main.py`) ที่สั้น กระชับ และจำง่าย:

| คำสั่งย่อ (`./run`) | คำสั่งเต็ม (`python main.py`) | หน้าที่การทำงาน |
| :--- | :--- | :--- |
| **`./run sim`** | `python main.py sim` | ทดสอบจำลองเดินตามแผนที่ (Simulation / Dry-run) |
| **`./run map`** | `python main.py map` | เปิด Map Planner GUI (Pygame + A*) เพื่อสร้างเส้นทาง |
| **`./run step 1`** | `python main.py step 1` | ทดสอบหุ่นจริงเดินหน้า 1 ช่อง (60 cm) พร้อม PID Centering |
| **`./run step 2`** | `python main.py step 2` | ทดสอบเดินหน้า 2 ช่องต่อเนื่อง |
| **`./run turn right`** | `python main.py turn right` | ทดสอบหมุนตัวเลี้ยวขวา 90° (`left`=ซ้าย, `around`=180°) |
| **`./run mon`** | `python main.py mon` | มอนิเตอร์ค่าเซนเซอร์สดจาก Thread 1 (ToF, Sharp L/R, Yaw) |
| **`./run ana`** | `python main.py ana` | วิเคราะห์ Log รอบล่าสุดอัตโนมัติ (หรือระบุรอบเช่น `./run ana 1`) |
| **`./run run`** | `python main.py run` | รันหุ่นจริงเต็มรูปแบบ (Pick $\rightarrow$ เดินตามแผนที่ $\rightarrow$ Drop) |
| **`./run pick`** | `python main.py pick` | สั่งแขนกลและ Gripper คีบของโดยตรง |
| **`./run drop`** | `python main.py drop` | สั่งแขนกลและ Gripper วางของโดยตรง |
| **`./run`** | `python main.py` | แสดงเมนูช่วยเหลือและตัวอย่างคำสั่ง |

---

### 1. วาดแผนที่และสร้างเส้นทางเดิน (Map Planner GUI)
```bash
./run map
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
# รันจำลองแบบปกติ (มีลำดับถามยืนยัน Pick -> เดิน -> Drop)
./run sim

# รันจำลองแบบ Auto-confirm ทั้งหมด (ข้ามการกด Enter)
./run sim -y

# รันจำลองแบบเดินตามแผนที่อย่างเดียว (ข้าม Pick & Drop)
./run sim --skip-pick --skip-drop -y
```

---

### 3. ทดสอบเดินและหมุนในสนามจริงทีละ Grid (Step & Turn Test)
ใช้สำหรับนำหุ่นไปวางในสนามจริงเพื่อทดสอบระบบ PID ปรับบาลานซ์กึ่งกลางกำแพง และทดสอบการเลี้ยว:
```bash
# ทดสอบเดินหน้า 1 ช่อง Grid (60 cm)
./run step 1

# ทดสอบเดินหน้า 2 ช่อง Grid ต่อเนื่อง
./run step 2

# ทดสอบการหมุนตัว
./run turn right     # เลี้ยวขวา 90°
./run turn left      # เลี้ยวซ้าย 90°
./run turn around    # กลับหลังหัน 180°
```

---

### 4. รันหุ่นยนต์จริงเต็มรูปแบบ (Full Autonomous Run)
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi AP ของ RoboMaster EP แล้วสั่งรัน:
```bash
# 4.1 รันเต็มรูปแบบ: คีบของ -> เดินตามแผนที่ด้วย PID -> วางของ
./run run

# 4.2 รันแบบเดินตามแผนที่อย่างเดียว (ข้าม Pick & Drop)
./run run --skip-pick --skip-drop -y
```
*(มี Emergency Stop ปลอดภัย กด `Ctrl + C` ได้ตลอดเวลา)*

---

### 5. มอนิเตอร์ค่าเซนเซอร์สดจาก Thread 1 (Live Monitor)
ดูค่าระยะ Sharp Left/Right (mm), ToF (mm), Yaw (deg), และสถานะกำแพงแบบ Real-time:
```bash
./run mon
```

---

### 6. วิเคราะห์สถิติและพลอตกราฟหลังรัน (Post-run Analysis)
```bash
# วิเคราะห์รอบล่าสุดอัตโนมัติ (ไม่ต้องระบุชื่อไฟล์ยาวๆ)
./run ana

# หรือวิเคราะห์รอบที่ต้องการ เช่น run1, run2
./run ana 1
./run ana 2
```

---

### 7. เครื่องมือ Calibrate เซนเซอร์ (Step 1)
```bash
# เก็บค่าจากหุ่นจริงลง CSV
./run cal collect sharp_left --board-id 1 --port 1
./run cal collect sharp_right --board-id 2 --port 2
./run cal collect tof --tof-index 0

# คำนวณสมการ Polynomial และเซฟ calibration.json พร้อมกราฟ
./run cal fit
```

---

## 📚 เอกสารประกอบฉบับเต็ม

- 📖 [docs/STEP1_CALIBRATION.md](docs/STEP1_CALIBRATION.md) — รายละเอียดการ Calibrate เซนเซอร์ Sharp/ToF/Gripper
- 📖 [docs/STEP2_MULTITHREADING.md](docs/STEP2_MULTITHREADING.md) — สถาปัตยกรรม Multi-threading (Producer-Consumer & Telemetry)
- 📖 [docs/STEP3_PID.md](docs/STEP3_PID.md) — รายละเอียดระบบ PID Centering และตาราง 8 Wall Decision Cases
- 📖 [docs/STEP4_GRIPPER.md](docs/STEP4_GRIPPER.md) — รายละเอียดระบบแขนกลและ Gripper Pick & Drop
- 📖 [docs/STEP5_MAP_PLANNER.md](docs/STEP5_MAP_PLANNER.md) — การใช้งาน Map Planner GUI และการคำนวณเส้นทาง A* Search
