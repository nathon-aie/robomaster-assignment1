# Step 5 — Environment Mapping, A* Path Planning & Trajectory Analysis

คู่มือและเอกสารอธิบายระบบ **การสร้างแผนที่สภาพแวดล้อม (Map Planning GUI)**, การหาเส้นทางอัตโนมัติด้วย **A* Algorithm**, การส่งออกคำสั่งเป็น **JSON Format** และการวิเคราะห์เส้นทางการเดินจริงผ่าน **Jupyter Notebook (`analyze\pathing.ipynb`)** ตามข้อกำหนดใน [REQ.md](../REQ.md)

---

## 🎯 วัตถุประสงค์ (Step 5 Requirements)

1. **GUI วาดแผนที่และวางกำแพง (Map Planner GUI)**:
   - กำหนดขนาดสนาม Grid ($6 \times 5$ ช่อง, ขนาดช่องละ $60 \times 60\text{ cm}$)
   - ตีกรอบกำแพงรอบนอกอัตโนมัติ และสามารถคลิกเพิ่ม/ลบกำแพงภายในสนามได้อย่างอิสระ
   - กำหนดจุดเริ่มต้น **Start (S)** และจุดเป้าหมาย **Goal (G)**
2. **อัลกอริทึมหาเส้นทางเดินอัตโนมัติ (A* Pathfinding)**:
   - ค้นหาเส้นทางที่ดีที่สุดได้ 2 โหมด: **เส้นทางสั้นที่สุด (Shortest Path)** หรือ **เลี้ยวน้อยที่สุด (Minimum Turns)**
   - คำนวณมุมหันหน้าของหุ่นยนต์ (Heading) และกำแพงสัมพัทธ์รอบตัวหุ่นยนต์ (Front, Back, Left, Right) ในแต่ละสเต็ป
3. **ส่งออกแผนและชุดคำสั่ง (Export to JSON)**:
   - บันทึกข้อมูลทั้งหมดลงไฟล์ [data\robot_map_plan.json](../data\robot_map_plan.json) เพื่อให้หุ่นยนต์โหลดไปใช้งาน
   - ออกแบบให้หุ่นยนต์เริ่มต้นหันหน้าไปตามแนวทางเดินเสมอ (ทิศ North/ขึ้น) ไม่จำเป็นต้องหมุนหุ่นตอนเริ่ม
4. **พล็อตเปรียบเทียบเส้นทางจริงทับแผนที่ (Trajectory Overlay in Jupyter)**:
   - ดึงข้อมูลแผนที่และ Log จาก [telemetry_logs/](../telemetry_logs/) มาแปลงพิกัดเข้าสู่ Grid Map
   - แสดงผลในสมุดบันทึก [analyze\pathing.ipynb](../analyze\pathing.ipynb) พร้อมวิเคราะห์ความแม่นยำและค่าความคลาดเคลื่อน

---

## 🏗 โครงสร้างสถาปัตยกรรมและไฟล์ที่เกี่ยวข้อง

```
+-------------------------------------------------------------------------------+
|                       1. Map Planner GUI (Pygame)                             |
|                           [src/map_planner.py]                                |
|                                                                               |
|   - วาดกำแพง Grid 5x6 ช่อง (60x60 cm)                                         |
|   - หาเส้นทาง A* (Shortest Path / Min Turns)                                  |
|   - คำนวณ Relative View Walls (Front/Back/Left/Right)                         |
|   - Generate Motion Commands (Move Forward N cells, Turn 90 deg)              |
+---------------------------------------+---------------------------------------+
                                        | (Export แผนที่และคำสั่ง)
                                        v
+-------------------------------------------------------------------------------+
|                       2. แผนการเดินหุ่นยนต์ JSON                              |
|                       [data\robot_map_plan.json]                              |
|   - grid_info: rows=6, cols=5, grid_size=100px                                |
|   - start: [0, 5], goal: [3, 1]                                               |
|   - walls: รายการตำแหน่งกำแพงของทุก Cell                                      |
|   - path: ลำดับพิกัด Waypoints ที่ต้องเดินผ่าน                                |
|   - commands: รายการคำสั่งของ Thread 2 (Move Forward / Turn)                  |
|   - step_wall_history: ประวัติกำแพงสัมพัทธ์รอบตัวหุ่นยนต์                     |
+---------------------------------------+---------------------------------------+
                                        |
                 +----------------------+----------------------+
                 | (โหลดเข้า Controller)                       | (โหลดเข้าสมุดบันทึก)
                 v                                             v
+---------------------------------+           +---------------------------------+
|      3. Robot Execution         |           |   4. Pathing & Trajectory       |
|   [src/robot_controller.py]     |           |          Analysis               |
|                                 |           |     [analyze\pathing.ipynb]     |
| - สั่งหุ่นยนต์เดินตามคำสั่ง JSON  |           |                                 |
| - PID Centering ตามกำแพง 8 แบบ  |           | - แปลง Odometry -> Grid Map      |
| - บันทึก Telemetry Log ลงไฟล์    |           | - พล็อต Planned vs Actual Path  |
|   (telemetry_logs/runX/)        |---------->| - คำนวณ MAE/RMSE Deviation      |
+---------------------------------+           | - พล็อต Sensor Profile vs Time  |
                                              +---------------------------------+
```

---

## 🎮 วิธีใช้งาน Map Planner GUI (`map.py` / `src/map_planner.py`)

เปิดโปรแกรมวาดแผนที่ผ่าน Master CLI:
```cmd
python main.py map
```
*(หรือรันผ่าน shim: `python map.py`)*

### ⌨️ ตารางคีย์ลัดในการควบคุม (Shortcuts):
| ปุ่ม / การคลิก | การทำงาน |
| :--- | :--- |
| **คลิกซ้าย** | เพิ่มหรือลบกำแพงตรงเส้นขอบระหว่างช่อง Grid |
| **กดปุ่ม `S` ค้างไว้ + คลิกช่อง** | ย้าย/วางตำแหน่งจุดเริ่มต้น **Start (สีเขียว)** |
| **กดปุ่ม `G` ค้างไว้ + คลิกช่อง** | ย้าย/วางตำแหน่งจุดเป้าหมาย **Goal (สีแดง)** |
| **`Spacebar`** | คำนวณเส้นทางที่สั้นที่สุด (**Shortest Path**) ด้วย A* |
| **`T`** | คำนวณเส้นทางที่เลี้ยวน้อยที่สุด (**Min Turns**) ด้วย A* |
| **`K`** | **บันทึกแผนที่ (Save)** และคำสั่งลง [data\robot_map_plan.json](../data\robot_map_plan.json) |
| **`L`** | **โหลดแผนที่ (Load)** ล่าสุดจากไฟล์ JSON |
| **`C`** | ล้างแผนที่และเส้นทางทั้งหมด (**Clear Map**) |

---

## 📈 การวิเคราะห์เส้นทางใน Jupyter Notebook (`analyze\pathing.ipynb`)

เปิดสมุดบันทึกการวิเคราะห์:
```cmd
jupyter notebook analyze\pathing.ipynb
```

### ฟังก์ชันหลักใน Notebook:
1. **`load_map_plan()`**: โหลดข้อมูล Grid, Start, Goal, กำแพง และเส้นทางวางแผน
2. **`load_latest_telemetry()`**: ตรวจจับและดึง Log การเดินจริงรอบล่าสุดจาก `telemetry_logs/` อัตโนมัติ
3. **`extract_robot_trajectory()`**: แปลงพิกัดหุ่นยนต์เข้าสู่ Grid Map Coordinates:
   $$X_{grid}(t) = col_{start} + 0.5 + \frac{pos\_y(t)}{0.60}$$
   $$Y_{grid}(t) = row_{start} + 0.5 - \frac{pos\_x(t)}{0.60}$$
4. **`plot_map_with_trajectory()`**: พล็อตแผนที่ Grid Map พร้อมทับเส้นทาง Planned Path (A*) และ Actual Trajectory พร้อมลูกศรแสดงทิศทางหุ่นยนต์ (IMU Yaw)
5. **`calculate_path_metrics()`**: คำนวณตัวชี้วัดความแม่นยำ:
   - Planned Distance vs Actual Traveled Distance
   - Mean Absolute Error (MAE) และ Max Path Deviation (mm)
   - เวลาที่ใช้ และความเร็วเฉลี่ย ($m/s$)
6. **Sensor Profiles Plot**: กราฟ Sharp Left/Right, ค่าผลต่าง $|L-R|$, ToF Distance และ IMU Yaw เทียบกับเวลา

---

## 📁 ซอร์สโค้ดและไฟล์ที่เกี่ยวข้อง

- [src/map_planner.py](../src/map_planner.py): โปรแกรม GUI แผนที่และอัลกอริทึม A*
- [map.py](../map.py): Shim Entry Point สำหรับรัน Map Planner จาก root
- [data\robot_map_plan.json](../data\robot_map_plan.json): ไฟล์แผนที่และลำดับคำสั่งที่ Export ออกมา
- [analyze\pathing.ipynb](../analyze\pathing.ipynb): สมุดบันทึก Jupyter วิเคราะห์และพล็อตเส้นทางเดิน\n