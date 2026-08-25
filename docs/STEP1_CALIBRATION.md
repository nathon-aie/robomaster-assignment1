# Step 1 — Calibration

เอกสารและคู่มือการ Calibrate เซนเซอร์สำหรับ RoboMaster EP (Sharp IR, ToF, Gripper) ตามข้อกำหนดใน [REQ.md](../REQ.md)

---

## 🛠 การติดตั้ง Dependencies

```bash
python3.8 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
```

---

## 📖 วิธีใช้งาน

### 1. เชื่อมต่อ RoboMaster EP
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi access point ของ RoboMaster EP ก่อนรันคำสั่ง `collect-live` โดยโปรแกรมใช้ `connection type = ap` เป็นค่าเริ่มต้น

### 2. สร้างไฟล์บันทึกค่า (CSV Template)
```bash
.venv/bin/python main.py calibrate init-csv data/calibration_measurements.csv
```

### 3. เก็บค่า Sharp และ ToF จากหุ่นจริง
วางกำแพงหรือเป้าหมายที่ระยะจริง แล้วรันคำสั่งต่อไปนี้ โปรแกรมจะถามระยะจริงเป็น mm และอ่านค่าจาก RoboMaster SDK โดยอัตโนมัติ:

```bash
# Sharp ด้านซ้าย (Sensor Adapter ID 1, Port 1)
.venv/bin/python main.py calibrate collect-live sharp_left --board-id 1 --port 1

# Sharp ด้านขวา (Sensor Adapter ID 2, Port 2)
.venv/bin/python main.py calibrate collect-live sharp_right --board-id 2 --port 2

# ToF ด้านหน้า (Distance Sensor Index 0)
.venv/bin/python main.py calibrate collect-live tof --tof-index 0
```

> **หมายเหตุ**:
> - ใช้ `--conn-type sta` เฉพาะกรณีเชื่อมต่อผ่าน Wi-Fi Router ในโหมด Station
> - ควรวัดอย่างน้อย 5–8 ระยะต่อเซนเซอร์ และวัดซ้ำระยะละ 3–5 ครั้ง (ช่วงระยะ Sharp ประมาณ 40–300 mm ตาม Datasheet GP2Y0A41SK0F)

### 4. เพิ่มค่า Gripper
SDK ของ RoboMaster EP รายงานสถานะ Gripper เป็น `opened` / `closed` / `normal` ไม่ใช่ระยะเปิดจริง ดังนั้นให้วัดระยะจริงด้วยมือ แล้วกรอกค่าใน [data/calibration_measurements.csv](../data/calibration_measurements.csv):

```csv
sensor,raw_value,reference_mm,sample_id
sharp_left,812,40,1
sharp_left,540,80,2
tof,42,42,1
gripper,101,100,1
```

### 5. คำนวณสมการและสร้างกราฟ Polynomial Fit
```bash
.venv/bin/python main.py calibrate fit data/calibration_measurements.csv
```

ผลลัพธ์จะถูกบันทึกไว้ที่:
- [calibration_output/calibration.json](../calibration_output/calibration.json) — สัมประสิทธิ์สมการ Polynomial แต่ละเซนเซอร์
- `calibration_output/sharp_left_calibration.png` — กราฟเปรียบเทียบ Curve Fitting ของ Sharp ฝั่งซ้าย
- `calibration_output/sharp_right_calibration.png` — กราฟเปรียบเทียบ Curve Fitting ของ Sharp ฝั่งขวา

---

## 🧪 การทดสอบโมดูล Calibration

```bash
.venv/bin/python -m unittest tests/test_calibration.py
```
