# Step 3 — Grid-by-Grid Navigation & PID Centering Control

คู่มือและเอกสารอธิบายระบบ **การเดินทีละ Grid + PID Control ปรับกึ่งกลางระหว่างกำแพง (แกน Y)** ตามข้อกำหนดใน [REQ.md](../REQ.md)

---

## 🎯 วัตถุประสงค์ (Step 3 Requirements)

1. **เดินทีละ Grid**: แบ่งการเคลื่อนที่ระยะไกลออกเป็นทีละช่อง ($60 \times 60$ cm, ความหนากำแพง 7.5 cm)
2. **PID Centering Control (แกน Y)**: ดึงข้อมูล Snapshot จาก **Thread 1** มาคำนวณและปรับความเร็วเบี่ยงข้าง ($v_y$) เพื่อรักษาให้ตัวหุ่นยนต์อยู่ตรงกลางระหว่างกำแพงตลอดเวลา
3. **Heading Stabilization Lock (แกน Z)**: รักษาองศาของหุ่นยนต์ (Yaw $0^\circ, 90^\circ, 180^\circ, 270^\circ$) ด้วย IMU Heading PID ($v_z$)
4. **ครอบคลุม 8 รูปแบบสภาพแวดล้อมกำแพง**:

---

## 📊 ตารางการตัดสินใจ 8 รูปแบบกำแพง (Wall Decision Table)

| Case | สภาพกำแพง | เงื่อนไขเซนเซอร์ | การคำนวณ Error แกน Y ($e_y$) | เกณฑ์ Deadband / เป้าหมาย |
| :---: | :--- | :--- | :--- | :--- |
| **1.1** | **มีกำแพงหน้า + 2 ข้าง** | ToF $< 350$mm, Sharp L & R $< 260$mm | $e_y = \text{Sharp}_L - \text{Sharp}_R$ | $\|\text{Sharp}_L - \text{Sharp}_R\| < 20$ mm ($2$ cm) |
| **1.2** | **มีกำแพงหน้า + ซ้ายอย่างเดียว** | ToF $< 350$mm, Sharp L $< 260$mm | $e_y = \text{Sharp}_L - \text{Nominal}$ | $\text{Sharp}_L \pm 20$ mm จากระยะกึ่งกลางปกติ |
| **1.3** | **มีกำแพงหน้า + ขวาอย่างเดียว** | ToF $< 350$mm, Sharp R $< 260$mm | $e_y = \text{Nominal} - \text{Sharp}_R$ | $\text{Sharp}_R \pm 20$ mm จากระยะกึ่งกลางปกติ |
| **1.4** | **มีกำแพงหน้า + ไม่มีกำแพงข้าง** | ToF $< 350$mm, No Side Walls | $e_y = 0$ | เดินตรง + เบรกเมื่อ ToF ถึงระยะกึ่งกลางช่อง |
| **2.1** | **ไม่มีกำแพงหน้า + 2 ข้าง** | ToF $\ge 350$mm, Sharp L & R $< 260$mm | $e_y = \text{Sharp}_L - \text{Sharp}_R$ | $\|\text{Sharp}_L - \text{Sharp}_R\| < 20$ mm ($2$ cm) |
| **2.2** | **ไม่มีกำแพงหน้า + ซ้ายอย่างเดียว** | ToF $\ge 350$mm, Sharp L $< 260$mm | $e_y = \text{Sharp}_L - \text{Nominal}$ | $\text{Sharp}_L \pm 20$ mm จากระยะกึ่งกลางปกติ |
| **2.3** | **ไม่มีกำแพงหน้า + ขวาอย่างเดียว** | ToF $\ge 350$mm, Sharp R $< 260$mm | $e_y = \text{Nominal} - \text{Sharp}_R$ | $\text{Sharp}_R \pm 20$ mm จากระยะกึ่งกลางปกติ |
| **2.4** | **ไม่มีกำแพงหน้า + ไม่มีกำแพงข้าง** | ToF $\ge 350$mm, No Side Walls | $e_y = 0$ | เดินตรง 60 cm ด้วย Odometry + ล็อกมุม Yaw |

> **หมายเหตุค่าระยะปกติ (Nominal)**: สำหรับช่องขนาด 60 cm กำแพงหนา 7.5 cm (ความกว้างช่องว่างด้านใน $\approx 525$ mm) และหุ่นกว้าง 250 mm จะมีระยะห่างจากขอบหุ่นถึงกำแพงแต่ละฝั่ง $\approx 137.5 - 140.0$ mm (กำหนดเป็นค่าเริ่มต้น `nominal_side_dist_mm = 140.0`)

---

## 🛠 การทำงานของ Controller Loop ในแต่ละ Grid

```
             +------------------------------------+
             | เริ่มต้น Grid Step (เป้าหมาย 60cm) |
             +-----------------+------------------+
                               |
                               v
                +------------------------------+
         +----->| ดึง Snapshot สดจาก Thread 1  | (ไม่มี Delay ฮาร์ดแวร์)
         |      +--------------+---------------+
         |                     |
         |                     v
         |      +------------------------------+
         |      | จำแนกสถานะกำแพง (8 Cases)    |
         |      +--------------+---------------+
         |                     |
         |                     v
         |      +------------------------------+
         |      | คำนวณ PID:                   |
         |      |  - vy: ปรับระยะเบี่ยงข้าง    | (Deadband 20mm)
         |      |  - vz: ล็อกมุมหัวหุ่น (Yaw)  |
         |      |  - vx: ความเร็วเดินหน้า      |
         |      +--------------+---------------+
         |                     |
         |                     v
         |      +------------------------------+
         |      | chassis.drive_speed(vx,vy,vz)|
         |      +--------------+---------------+
         |                     |
         +--- [ระยะทาง < 60cm และ ToF ยังไม่ชน]---+
                               |
                               v [เดินครบ 60cm หรือ ToF ถึงเป้า]
                +------------------------------+
                | align_at_cell_center()       | (จูนละเอียดหยุดนิ่งตรงกลาง)
                +------------------------------+
```

---

## 🚀 คำสั่งสำหรับทดสอบและใช้งาน

### 1. ทดสอบเดินเฉพาะ 1 ช่อง Grid (Step Test)
ใช้สำหรับวางหุ่นในสนามจริงเพื่อดูการปรับกึ่งกลางของหุ่นยนต์:
```bash
# ทดสอบเดิน 1 ช่องกับหุ่นจริง (AP mode)
.venv/bin/python main.py step-test --cells 1 --conn-type ap

# หรือทดสอบเดิน 2 ช่อง
.venv/bin/python main.py step-test --cells 2 --conn-type ap
```

### 2. รันตามแผนที่ [data/robot_map_plan.json](../data/robot_map_plan.json) พร้อมระบบ PID
```bash
# รันหุ่นยนต์จริง
.venv/bin/python main.py run --conn-type ap --plan data/robot_map_plan.json

# หรือรันในโหมดจำลอง (Simulation)
.venv/bin/python main.py simulate --plan data/robot_map_plan.json
```

### 3. ปรับจูน Parameter เพิ่มเติม
- `--speed`: ความเร็วเดินหน้าปกติ (default: `0.25` m/s)
- `--nominal-side`: ระยะห่างเป้าหมายจากหุ่นถึงกำแพงเดี่ยว (default: `140.0` mm)
```bash
.venv/bin/python main.py run --conn-type ap --speed 0.20 --nominal-side 140.0
```

### 4. รันชุดทดสอบความถูกต้องของ PID (11 Tests)
```bash
.venv/bin/python -m unittest tests/test_step3_pid.py
```
