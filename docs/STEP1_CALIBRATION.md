# Step 1 — Sharp IR Sensor Calibration

เอกสารและคู่มือการ Calibrate เซนเซอร์ **Sharp IR (GP2Y0A41SK0F)** ด้านซ้ายและขวาสำหรับ RoboMaster EP ตามข้อกำหนดใน [REQ.md](../REQ.md)

---

## 🎯 วัตถุประสงค์ (Overview)

เซนเซอร์ Sharp GP2Y0A41SK0F ส่งสัญญาณออกมาเป็นค่าแรงดันไฟฟ้า **ADC (Analog 100–800)** ซึ่งมีความไม่เป็นเชิงเส้น (Non-linear) จึงต้องทำการแปลงค่า ADC เป็นระยะทางมิลลิเมตร (mm) ในโลกจริงด้วย **Polynomial Curve Fitting ($ax^2 + bx + c$)**

*(หมายเหตุ: เซนเซอร์ ToF ด้านหน้ารายงานระยะทางเป็นมิลลิเมตร (mm) ตรงกับระยะจริงอยู่แล้ว และ Gripper ทำงานตามพิกัดคงที่ จึงไม่ต้องทำ Curve Fitting)*

---

## 📖 วิธีใช้งาน

### 1. เชื่อมต่อ RoboMaster EP
เชื่อมต่อคอมพิวเตอร์เข้ากับ Wi-Fi Access Point ของ RoboMaster EP (`conn-type: ap`)

### 2. สร้างไฟล์บันทึกค่า (CSV Template)
```bash
./run cal init data/calibration_measurements.csv
```

### 3. เก็บค่า ADC จากเซนเซอร์ Sharp จริง
วางกำแพงหรือแผ่นกั้นที่ระยะจริง (เช่น 50, 100, 150, 200, 250, 300 mm) แล้วรันคำสั่ง:

```bash
# Sharp ด้านซ้าย (Sensor Adapter ID 1, Port 1)
./run cal collect sharp_left --board-id 1 --port 1

# Sharp ด้านขวา (Sensor Adapter ID 2, Port 2)
./run cal collect sharp_right --board-id 2 --port 2
```

> **คำแนะนำการวัด**:
> - ควรวัดอย่างน้อย 5–8 ระยะต่อเซนเซอร์ (ช่วงระยะ Sharp ที่แนะนำคือ 40–300 mm)

### 4. คำนวณสมการ Polynomial Fit และสร้างกราฟ
```bash
./run cal fit
```

ผลลัพธ์จะถูกบันทึกไว้ที่:
- [calibration_output/calibration.json](../calibration_output/calibration.json) — สัมประสิทธิ์สมการ $ax^2 + bx + c$
- `calibration_output/sharp_left_calibration.png` — กราฟเปรียบเทียบ Curve Fitting ของ Sharp ฝั่งซ้าย
- `calibration_output/sharp_right_calibration.png` — กราฟเปรียบเทียบ Curve Fitting ของ Sharp ฝั่งขวา

