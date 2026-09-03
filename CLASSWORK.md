# 📑 รายงานและเอกสารอ้างอิง: Classwork Occupancy Grid Mapping (OGM)

เอกสารสรุปการเปรียบเทียบและการตรวจสอบความสอดคล้องระหว่าง **ทฤษฎีบท / ขั้นตอนการทดลองในเอกสาร Classwork Occupancy Grid Mapping** (`/home/nathon/Downloads/CLasswork Occupancy Grid Mapping.docx`) กับ **การทำงานจริงของโปรเจกต์ RoboMaster EP Autonomous Grid Navigation System**

---

## 🎯 1. วัตถุประสงค์ตาม Classwork

1. **เข้าใจหลักการของ Occupancy Grid Mapping (OGM)**
2. **ทดลองใช้ IR Sensor ในการตรวจจับสิ่งกีดขวาง**
3. **เขียนโปรแกรมควบคุม RoboMaster ให้สร้างแผนที่ 4×4 Grid (ขนาด 60×60 cm ต่อช่อง)**
4. **แสดงผลการสร้างแผนที่แบบเรียลไทม์ (Real-Time Update)**

---

## 🔬 2. พื้นฐานทฤษฎีและหลักการทำงานของ Occupancy Grid Mapping

Occupancy Grid Mapping เป็นวิธีการสร้างแผนที่สภาพแวดล้อมที่นิยมใช้ในวิทยาการหุ่นยนต์ (Robotics) โดยแบ่งสภาพแวดล้อมออกเป็นตารางสี่เหลี่ยมย่อย (**Grid Cells**) และเก็บค่าความน่าจะเป็นหรือสถานะของการมีสิ่งกีดขวาง:

- **0 (Free)**: พื้นที่ว่าง ไม่มีสิ่งกีดขวาง
- **1 (Occupied)**: มีสิ่งกีดขวาง/กำแพง
- **0.5 (Unknown)**: พื้นที่ที่ยังไม่เคยถูกสำรวจ

### หลักการ 4 ขั้นตอนหลัก:
1. **การแบ่งพื้นที่ (Discretization)**: สภาพแวดล้อมจริงถูกแบ่งเป็นตาราง $4 \times 4$ ขนาด $60 \times 60\text{ cm}$ ต่อเซลล์
2. **การอ่านค่าจากเซนเซอร์ (Sensor Reading)**: ใช้ IR Sensor และ ToF Sensor อ่านระยะทาง 3 ทิศทาง (ซ้าย, หน้า, ขวา)
3. **การแปลงสถานะและอัปเดต (Thresholding & Map Update)**:
   - ระยะ $<$ Threshold $\rightarrow$ มีสิ่งกีดขวาง (Occupied)
   - ระยะ $\ge$ Threshold $\rightarrow$ พื้นที่ว่าง (Free)
4. **การแสดงผลแบบเรียลไทม์ (Visualization)**: แสดงผลการเปลี่ยนแปลงสถานะของแต่ละ Grid Cell ทันทีระหว่างการสำรวจ

---

## 📊 3. ตารางเปรียบเทียบความสอดคล้อง (Classwork vs. Implementation)

| หัวข้อตาม Classwork | ทฤษฎีและขั้นตอนในเอกสาร | การทำงานจริงในโปรเจกต์ | ผลการประเมิน |
| :--- | :--- | :--- | :---: |
| **1. การแบ่งพื้นที่ (Discretization)** | - แบ่งสภาพแวดล้อมเป็นตารางกริด $4 \times 4$<br>- ขนาดช่อง $60 \times 60\text{ cm}$ ต่อ 1 Grid Cell | ใน `DiscoveredGridMap` (`src/grid_mapper.py`):<br>- กำหนด `cols=4, rows=4`<br>- กำหนด `grid_size_m=0.60` ($60\text{ cm}$) | ✅ **ตรงตามทฤษฎี 100%** |
| **2. การอ่านค่าเซนเซอร์ 3 ด้าน (Sensor Reading)** | - อ่านค่าระยะทางจากเซนเซอร์ 3 ตัว:<br>  - ซ้าย (Left IR)<br>  - หน้า (Front IR / ToF)<br>  - ขวา (Right IR) | ใน `sense_current_cell()` (`src/grid_mapper.py`):<br>- อ่าน **Sharp IR Left** (ID 1, Port 1)<br>- อ่าน **ToF Front Sensor**<br>- อ่าน **Sharp IR Right** (ID 2, Port 2) ผ่าน Thread 1 | ✅ **ตรงตามทฤษฎี 100%** |
| **3. การแปลงค่าและ Thresholding** | - ถ้า $<$ threshold $\rightarrow$ มีสิ่งกีดขวาง (Occupied / 1 / Wall)<br>- ถ้า $\ge$ threshold $\rightarrow$ ช่องว่าง (Free / 0)<br>- ยังไม่สำรวจ $\rightarrow$ Unknown (0.5 / ?) | ใน `src/grid_mapper.py`:<br>- กำแพงข้าง (Sharp L/R) $< 220\text{ mm} \rightarrow$ Occupied (มีกำแพง)<br>- กำแพงหน้า (ToF) $< 350\text{ mm} \rightarrow$ Occupied (มีกำแพง)<br>- ช่องที่ยังไม่เดินถึง $\rightarrow$ `visited = False` (`?`) | ✅ **ตรงตามทฤษฎี 100%** |
| **4. การเคลื่อนที่แบบ Step-by-Step** | - เคลื่อนที่ทีละ Grid จากจุดศูนย์กลางช่องหนึ่งไปยังอีกช่องหนึ่ง ($60\text{ cm}$ ต่อก้าว) | ใน `execute_forward_one_cell()` (`src/grid_mapper.py`):<br>- สั่งเคลื่อนที่ทีละช่อง $60\text{ cm}$ พอดี<br>- มีการหยุดเพื่อ Flush Filter และอ่านค่าเซนเซอร์รอบใหม่ | ✅ **ตรงตามทฤษฎี 100%** |
| **5. การอัปเดตและแสดงผล Real-time** | - เมื่อหุ่นเข้าสู่ grid ใหม่ $\rightarrow$ อัปเดต Map ทันที<br>- แสดงผล Real-time ผ่านสัญลักษณ์ (unknown, occupied, free) | ใน `explore()` และ `render_ascii()` (`src/grid_mapper.py`):<br>- อัปเดตสถานะและกำแพงลงตารางทันทีในแต่ละ Step<br>- วาด ASCII Grid แสดงตำแหน่งหุ่นยนต์ ทิศทาง และกำแพงแบบ Real-time | ✅ **ตรงตามทฤษฎี 100%** |
| **6. การบันทึกและส่งออกแผนที่ (Map Export & Data)** | - บันทึกผลลัพธ์แผนที่ลงไฟล์<br>- บันทึก Step, ตำแหน่ง $(x, y)$, ค่าเซนเซอร์ และสถานะ | ใน `export_json()` (`src/grid_mapper.py`) และ `src/telemetry.py`:<br>- ส่งออกไฟล์ `discovered_map.json` และเรนเดอร์ภาพ `discovered_map.png`<br>- เก็บบันทึก Time-series Log เป็น CSV/JSON ครบทุก Step | ✅ **ตรงตามทฤษฎี 100%** |

---

## 🌟 4. ฟังก์ชันขั้นสูงที่พัฒนาเพิ่มเติมจาก Classwork พื้นฐาน

1. **Relative-to-Global Coordinate Transformation**:
   - หุ่นยนต์แปลงค่ากำแพงสัมพัทธ์ (ด้านหน้า/ซ้าย/ขวา) เป็นกำแพงโลก (`top`, `bottom`, `left`, `right`) ตามทิศทางหันหน้า (North, East, South, West) และอัปเดตกำแพงของช่องข้างเคียงแบบสองฝั่ง (Reciprocal Consistency)
2. **Autonomous Maze Exploration (DFS + BFS Frontier Backtracking)**:
   - หุ่นยนต์สำรวจเขาวงกตอัตโนมัติเต็มรูปแบบโดยใช้ DFS ในการเดินหน้าเข้าสู่พื้นที่ใหม่ และใช้ BFS หาทางย้อนกลับเมื่อเจอทางตัน (Dead End) จนสำรวจครบ 100%
3. **Multi-Threading Architecture (2 Threads) & Signal Filtering**:
   - **Thread 1 (Sensor Pipeline)**: รวบรวมและกรองสัญญาณรบกวนของเซนเซอร์ด้วย Median Filter และ Exponential Moving Average (EMA)
   - **Thread 2 (Robot Controller)**: ควบคุมการเคลื่อนที่และคำนวณอัลกอริทึม
4. **Closed-Loop PID Centering (8 Wall Decision Cases)**:
   - รักษาระยะให้อยู่กึ่งกลางระหว่างกำแพง ($|L - R| < 20\text{ mm}$ หรือ $L/R \approx 140\text{ mm}$) ขณะเคลื่อนที่ $60\text{ cm}$ และล็อกมุม Heading ด้วย IMU Yaw ป้องกันการเอียงตัว

---

## 🚀 5. คำสั่งที่เกี่ยวข้องสำหรับการรันและทดสอบ

```bash
# 1. ทดสอบสำรวจและสร้างแผนที่ OGM ในโหมดจำลอง (Mock Simulation)
python main.py explore --mock --sim-maze data/robot_map_plan.json --output data/discovered_map.json

# 2. รันสำรวจและสร้างแผนที่ OGM บนหุ่นยนต์จริง
python main.py explore --conn-type ap --output data/discovered_map.json --start-col 0 --start-row 3

# 3. เรนเดอร์รูปภาพแผนที่ที่สร้างได้ออกมาเป็นภาพ PNG
python main.py plot-map data/discovered_map.json --output data/discovered_map.png
```
