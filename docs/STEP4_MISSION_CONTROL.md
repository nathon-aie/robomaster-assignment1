# Step 4: RoboMaster Mission Control Center (Control Panel)

แผงควบคุมภารกิจแบบครบวงจร ต่อยอดจาก Step 1–3 เดิมทั้งหมด
(Thread 1 Sensor Pipeline, Thread 2 PID Grid Navigation, Map Planner A*)
โดย **ไม่ได้เขียนทับหรือลบของเดิม** — ทุกคำสั่งเดิมใน `main.py` ยังใช้ได้ตามปกติ

```bash
python main.py panel                      # เปิด Mission Control (โหมด Simulation)
python main.py panel --mode real --conn-type ap
python main.py panel --map data/my_map.json --cell-size 0.60
```

---

## 1. ภาพรวมสถาปัตยกรรม (Architecture)

```text
                        ui/app.py  (Pygame dashboard)
                                 |
                        mission.py  MissionController
                                 |
        +------------------+-----+------+--------------------+
        |                  |            |                    |
  robot_iface.py      sensors.py    mapper.py          pathfinding.py
   Real / Mock / Sim   Real / Sim   OccupancyMapper     A* + checkpoints
        |                  |            |                    |
   RobotSystem        SensorHub    occupancy.py         explorer.py
   (src/, Step 2-3)   (Thread 1)   OccupancyGrid        frontier search
        |                                                     |
  RoboMaster SDK  /  simulation.py SimRobot + ground truth ----+

                 geometry.py  CoordinateTransform   robot_state.py  RobotStateTracker
```

ไฟล์ทั้งหมดอยู่ใน `src/panel/` แยกโมดูลตามหน้าที่ ไม่มีไฟล์ยักษ์ไฟล์เดียว

| ไฟล์ | หน้าที่ |
| --- | --- |
| `geometry.py` | แปลงพิกัดหุ่น (m, deg) ↔ พิกัดแผนที่ (cell) — origin / cell size / heading / handedness ปรับได้หมด |
| `occupancy.py` | Occupancy grid: cell state (UNKNOWN/FREE/WALL/OBSTACLE) + edge wall 3 สถานะ + rotate + save/load JSON |
| `pathfinding.py` | A*, turn penalty, ลำดับ checkpoint (brute force ≤7, ไม่งั้น NN + 2-opt), metrics |
| `robot_state.py` | `RobotState` มาตรฐาน, trail จริง, tracking timeout |
| `sensors.py` | `SensorInterface`: `RealSensorInterface` (SensorHub) และ `SimulatedSensorInterface` (raycast + noise) |
| `mapper.py` | `OccupancyMapper` — อัลกอริทึม mapping ตัวเดียว ใช้ร่วมกันทั้งของจริงและ simulation |
| `explorer.py` | Frontier-based exploration planner |
| `robot_iface.py` | `RealRobotInterface` / `MockRobotInterface` / `SimRobotInterface` |
| `simulation.py` | `SimRobot` + ground-truth map + `SimulationEngine` |
| `mission.py` | State machine ของภารกิจ + safety ทั้งหมด |
| `ui/` | `app.py` dashboard, `map_view.py` แผนที่โต้ตอบ, `widgets.py`, `theme.py` |

---

## 2. เซนเซอร์ที่ใช้ (ของจริงเท่านั้น)

ใช้เฉพาะเซนเซอร์ที่โปรเจกต์นี้ต่อไว้จริงและมีโค้ดอยู่แล้วใน `src/sensor_pipeline.py`:

| ชื่อ | ที่มาใน SDK | ระยะ |
| --- | --- | --- |
| Front (ToF) | `robot.sensor.sub_distance` | 0.10 – 4.00 m |
| Left (Sharp IR) | `sensor_adaptor` ADC id=1 port=1 → calibration curve | 0.04 – 0.40 m |
| Right (Sharp IR) | `sensor_adaptor` ADC id=2 port=2 → calibration curve | 0.04 – 0.40 m |
| Pose | `chassis.sub_position` + `sub_attitude` (zeroed ตอนเริ่ม) | — |
| Velocity / IMU / ESC / Status / Gripper | `chassis.sub_velocity`, `sub_imu`, `sub_esc`, `sub_status`, `gripper.sub_status` | — |

**ไม่มี** LiDAR, ultrasonic ภายนอก, Arduino, Raspberry Pi, กล้องเสริม
และ **ไม่มีเซนเซอร์หลัง** — `back_mm` เป็น `None` เสมอ และ UI ไม่แสดง

Pose มาจาก odometry ของ chassis รวมกับ IMU yaw ที่ Thread 1 zero ให้แล้ว
ไม่มีการปลอมค่าตำแหน่งหรือค่าเซนเซอร์ในโหมด Real Robot

---

## 3. ระบบพิกัด (Coordinate Transform)

`(0,0)` ของหุ่น **ไม่ใช่** `(0,0)` ของแผนที่ ทุกพารามิเตอร์ตั้งค่าได้:

```text
robot (x_m, y_m, yaw)  --CoordinateTransform-->  map (col, row, heading)
```

* `x` = ทิศหน้าเริ่มต้น, `y` = ด้านขวาของหุ่น, `yaw` เพิ่ม = หมุนตามเข็ม
  (ตรงกับที่ `robot_controller.navigate_single_grid_step` ใช้อยู่เดิม)
* `origin_col/origin_row` = ช่องที่หุ่นยืนตอน zero odometry
* `cell_size_m` ปรับได้ (default 0.60 m ตาม REQ)
* `start_dir` = ทิศบนแผนที่ที่แกน +x ของหุ่นชี้ไป
* `handedness` สำหรับกลับด้านถ้าสนามจริงกลับทิศ

---

## 4. โหมดการทำงาน (Mode Separation)

| โหมด | ส่งคำสั่งไปฮาร์ดแวร์ | เซนเซอร์ | ใช้เมื่อ |
| --- | --- | --- | --- |
| `SIMULATION` | ไม่ | จำลอง (raycast + noise) | ออกแบบแผนที่, ทดสอบ A*, ทดสอบ auto-mapping |
| `REAL ROBOT` | **ใช่** | ของจริงจาก SensorHub | เดินในสนามจริง |
| `MOCK ROBOT` | ไม่ | `MockRobotActuators` เดิมของโปรเจกต์ | พัฒนาโดยไม่มีหุ่น |

* สลับเป็น `REAL ROBOT` ต้องกดยืนยันใน dialog ก่อนเสมอ
* `RealRobotInterface` **ไม่ยอม** fall back ไป mock เงียบ ๆ
  (ต่างจาก `RobotSystem.connect_robot()` เดิม) — ถ้าต่อไม่ได้จะรายงาน error ตรง ๆ
* สลับโหมดระหว่างภารกิจกำลังรันไม่ได้

---

## 5. ขั้นตอนความปลอดภัย (Safety Chain)

### รันแบบออฟไลน์ และห้ามรันผ่านเครื่องมืออื่น

โปรเจกต์นี้ **ไม่ต้องใช้อินเทอร์เน็ตเลย** ตอนรัน — ต่อ Wi-Fi ของหุ่น (`RMEP-xxxxxx`)
แล้วเปิดได้ทันที (ตรวจแล้ว: ไม่มี urllib/requests/http ในโค้ดโปรเจกต์,
SDK คุยกับ `192.168.2.1` เท่านั้น)

เปิดจาก terminal ของตัวเอง หรือดับเบิลคลิก `run_panel.bat`

> **ห้าม** เปิด panel จากใน session ของเครื่องมืออื่น (เช่น AI coding assistant)
> ถ้า session นั้นโดน kill ตอนหุ่นกำลังวิ่ง โปรเซสจะตายทั้งที่ยังมีความเร็วค้างอยู่
> ในล้อ → **หุ่นวิ่งต่อจนชนกำแพง**

### กันหุ่นวิ่งต่อเมื่อโปรแกรมตาย (Runaway Protection)

`chassis.drive_speed()` เป็นคำสั่ง *ต่อเนื่อง* — ล้อจะหมุนด้วยความเร็วเดิมไปเรื่อย ๆ
จนกว่าจะมีคำสั่งใหม่ มี 2 ชั้นป้องกัน:

| ชั้น | ครอบคลุม | ไม่ครอบคลุม |
| --- | --- | --- |
| **SDK watchdog** — ส่ง `timeout=` ทุกครั้งที่ `drive_speed` (`drive_watchdog_sec`, default 0.4 s) | control loop ค้าง / UI freeze ขณะโปรเซสยังอยู่ | โปรเซสตาย (timer อยู่ใน process เดียวกัน) |
| **`CHASSIS_SAFETY_NET`** — `atexit` + SIGINT/SIGTERM/SIGBREAK + `finally` ใน run loop | ปิดหน้าต่าง, Ctrl+C, exception, taskkill ธรรมดา | `SIGKILL` / "End task" แบบบังคับ |

**ไม่มีอะไรใน process กัน SIGKILL ได้** — สวิตช์ปิดหุ่นต้องอยู่ใกล้มือเสมอ


```text
CONNECT  ->  ARM  ->  RUN  ->  (confirm dialog ถ้าเป็นหุ่นจริง)
```

* หุ่นจริงจะไม่ขยับเองเด็ดขาด ต้อง ARM + RUN แยกกันสองจังหวะ
* **EMERGENCY STOP** อยู่มุมขวาบนตลอดเวลา (คีย์ลัด `F1`) —
  ตัด `_running` ของ Thread 2 ทันที, latch ไว้จนกด `CLEAR E-STOP`, และ disarm อัตโนมัติ
* Tracking timeout (default 1.5 s ไม่มี pose ใหม่) → `TRACKING LOST` + หยุดหุ่น
* เจอสิ่งกีดขวางที่ไม่คาด → หยุด, บันทึกลงแผนที่, วางแผนใหม่
* ไม่เดินผ่านช่อง `UNKNOWN` เว้นแต่เปิด `allow_unknown_cells`
* คำสั่งจะถือว่าสำเร็จก็ต่อเมื่อ interface ยืนยัน หรือ pose จริงยืนยันว่าถึงช่องแล้ว

---

## 6. Auto Mapping (Frontier Exploration)

```text
scan sensors -> update occupancy grid -> หา frontier -> เลือก frontier ที่ดีที่สุด
     ^                                                              |
     +---------------- A* ไปยัง frontier -> เดินทีละช่อง <----------+
```

Frontier = ช่องที่รู้ว่าว่าง และยังมีอะไรให้เรียนรู้ข้าง ๆ —
ทั้ง *ช่องที่ยังไม่รู้* และ *ขอบที่ยังไม่เคยส่องดู*
(ToF ที่ยิงไกลจะ mark ช่องว่างล่วงหน้าโดยยังไม่รู้ว่ามีกำแพงกั้นหรือไม่)

การให้คะแนน frontier: ระยะทาง − information gain + จำนวนการเลี้ยว + โทษทางตัน

หยุดเมื่อ: ไม่มี frontier ที่ไปถึงได้ / ผู้ใช้กด STOP / Emergency Stop /
tracking หาย / เกิน step limit

**คุณภาพการ map** (วัดจาก `tests/test_panel.py` และ stress run):
6×6, 7×7 maze, 9×9 ว่าง, 6×2, 2×2 → สร้างกำแพงตรงกับ ground truth 100% ทั้งเปิดและปิด noise

ตัวช่วยความแม่นยำ:
* วัดจาก pose จริง ไม่ใช่จุดกึ่งกลางช่อง (ชดเชยหุ่นไม่อยู่กลางช่อง)
* ข้าม frame ที่หุ่นกำลังหมุน/อยู่กลางทาง (`heading_tolerance_deg`, `center_tolerance_cells`)
* ต้องมี 2 frame ที่เห็นตรงกันถึงจะลงกำแพง (`wall_confirm_votes`) — กัน noise สร้างกำแพงผี
* ไม่ integrate frame เดิมซ้ำ (กัน stale data)

---

### Full coverage (`full_coverage`, default เปิด)

Auto-map จะไม่จบจนกว่าจะ **ขับผ่านทุกช่องที่ไปถึงได้จริง** ไม่ใช่แค่ส่องเห็นจากช่องข้าง ๆ
ลำดับการเลือกเป้าหมาย:

1. **Frontier** — ช่องที่ยังมีอะไรให้เรียนรู้ (ช่อง/ขอบที่ยังไม่รู้)
2. หมด frontier แล้ว → **Coverage** — ช่องว่างที่ไปถึงได้แต่ยังไม่เคยขับผ่าน

จบแล้ว ถ้าตั้ง Start/Goal ไว้ จะ **วางเส้นทาง A* ให้อัตโนมัติ** (`plan_after_mapping`)
ดูความคืบหน้าได้จาก `coverage_summary()` → `(ขับผ่านแล้ว, ไปถึงได้ทั้งหมด)`

---

## 7b. Gripper: หยิบขวดไปวางตามทิศที่กำหนด

พอร์ตมาจาก branch `gripper` (`src/gripper_controller.py` — ใช้ `robot.robotic_arm`
และ `robot.gripper` ของ SDK จริง) แล้วต่อเข้ากับ panel

**เครื่องมือ `Place`** (แถว TOOLS): คลิกวางจุดปล่อยของ, คลิกซ้ำหรือคลิกขวาเพื่อหมุนทิศ
บนแผนที่แสดงเป็นช่องสีม่วงตัว `P` พร้อมลูกศรบอกทิศที่หุ่นจะหันตอนวาง

**ปุ่ม `CARRY`** (กลุ่ม GRIPPER) — ลำดับภารกิจ:

```text
ขับไปที่ Goal (ตำแหน่งขวด) -> หนีบขวด -> ขับไปที่ Place point
        -> หมุนหันทิศที่กำหนด -> ปล่อยขวด
```

| Backend | Gripper |
| --- | --- |
| `REAL ROBOT` | `SimpleGripperController` → `robotic_arm.move()` / `gripper.open()/close()` |
| `SIMULATION` | จำลอง (มีสถานะ carrying จริง แต่ไม่มีฮาร์ดแวร์) |
| `MOCK ROBOT` | ไม่มี gripper — ปุ่ม CARRY จะกดไม่ได้ |

จุด Place เก็บลง JSON, หมุนตามแผนที่, และถูกล้างถ้า resize จนหลุดขอบ
บนหุ่นจริงต้องยืนยันใน dialog ก่อนเริ่ม และสถานะ gripper แสดงในแผง ROBOT STATE

---

## 7. Simulation

* `SimRobot` เดินหน้า/หมุนอยู่กับที่ ชนกำแพงจริงของ ground truth
* จำลอง closed-loop centering ของหุ่นจริง: หลังจบทุกคำสั่งจะ settle เข้ากลางช่อง
  โดยเหลือ error ~2 cm / ~1° (ปรับได้ที่ `centering_error_m`, `centering_error_deg`)
* เซนเซอร์จำลอง **ไม่สมบูรณ์แบบ**: มี range window, blind spot, Gaussian noise,
  dropout, และ saturate ที่ระยะสูงสุดเหมือนของจริง
* ground-truth map **ซ่อนไว้** ระหว่าง auto-mapping — เปิดดูได้ด้วยปุ่ม `TRUTH` (debug)
* ความเร็ว 0.5x / 1x / 2x / 5x / 10x

Simulation ใช้ pipeline เดียวกับของจริงทุกขั้น:
`sensors → RobotStateTracker → OccupancyMapper → UI`

---

## 7c. Gripper: ตรวจจับวัตถุ, หยิบ, และวางแบบเล็งจุด

### ตรวจจับขวด/กระป๋อง (`objects.py`)

**ไม่มีการรู้จำวัตถุจริง** และโค้ดไม่แกล้งทำเป็นมี — vision module ของ DJI
มีแค่ GESTURE/LINE/MARKER/PERSON/ROBOT (ไม่มีคลาสขวด/กระป๋อง) และโปรเจกต์นี้
stub camera codec ทิ้งไปแล้ว (`load_robot_sdk`) จึงไม่มีภาพจากกล้อง

สิ่งที่มีจริงคือ **ToF ด้านหน้า** ซึ่งพอสำหรับคำถามที่ต้องตอบจริง ๆ คือ
*ข้างหน้ามีของให้หยิบไหม* เพราะแผนที่บอกอยู่แล้วว่ากำแพงควรอยู่ไกลแค่ไหน:

```text
ระยะที่วัดได้  <<  ระยะที่แผนที่คาดไว้   ->  มีของอยู่
```

ต้องเห็นตรงกันหลาย frame ติดกัน (`confirm_frames`) กัน noise สร้างวัตถุผี

### ถืออยู่ = ไม่สนของข้างหน้า

ขณะที่ gripper หนีบของอยู่ ตัวของจะบังลำแสง ToF ตลอดเวลา (~12 ซม.)
ถ้าไม่จัดการ หุ่นจะ **ขยับไม่ได้เลยทุกทิศ** เพราะมองว่ามีสิ่งกีดขวางตลอด
เมื่อ `carrying` เป็นจริง ระบบจะ:

| จุด | พฤติกรรม |
| --- | --- |
| `ObjectDetector.detect()` | คืนค่าไม่พบเสมอ — ของในมือไม่ใช่ของชิ้นใหม่ |
| `OccupancyMapper.ignore_front` | ข้ามเซนเซอร์หน้า ไม่ลงกำแพงผี |
| `mapper.obstacle_ahead()` | ไม่นับเป็นสิ่งกีดขวาง |
| `_front_clearance_mm()` | คืน `None` — ไม่บล็อกการเคลื่อนที่ |

### ปุ่ม `FETCH` / `DELIVER` (ปุ่มเดียว เปลี่ยนตามของในมือ)

```text
มือว่าง   -> ไปที่ Goal -> จอด "ข้าง ๆ" ช่องนั้น (ของกินพื้นที่ช่อง) -> หันเข้าหา
             -> ตรวจจับ -> หนีบ
ถืออยู่   -> ไปที่ Place point -> หันตามทิศที่กำหนด -> เล็งจุดย่อย -> วาง
```

ถ้าไม่ได้ตั้ง Place point จะหยิบแล้วถือรอ (`HOLDING`) กด CARRY อีกครั้งเพื่อไปวาง

### หน้าต่างเล็งจุดวาง (`PLACE TARGET`)

ช่องกริดกว้าง 60 ซม. แต่ขวดกว้างไม่กี่ ซม. — "วางในช่องนั้น" จึงยังไม่ละเอียดพอ
ปุ่ม `PLACE TARGET` เปิดหน้าต่างซูมช่อง Place ช่องเดียว แบ่งเป็นตาราง 5×5
คลิกเลือกช่องย่อยเพื่อกำหนดจุดปล่อยของจริง ๆ

* เก็บเป็น `place_offset` (เศษส่วนของช่อง, แกนจอ +x ตะวันออก / +y ใต้)
  → หมุนตามแผนที่ และบันทึกลง JSON
* ตอนวางจริง แปลงเป็นเฟรมหุ่น (เดินหน้า, ขวา) แล้วขยับ chassis ก่อนปล่อย
* ปุ่ม `CENTRE` รีเซ็ตกลางช่อง, `TURN` หมุนทิศที่จะหันตอนวาง

### `BACK TO START`

ขับกลับช่อง Start จากตำแหน่งไหนก็ได้ ใช้ replanning ตัวเดียวกับภารกิจอื่น
(หุ่นจริงต้อง ARM และยืนยันใน dialog ก่อน)

---

## 8. Dashboard

```text
+--------------------------------------------------------------+
| ROBOMASTER MISSION CONTROL      MODE / LINK / STATE  [E-STOP] |
+-------------+------------------------------+-----------------+
| MODE        |                              | SENSOR DEBUG    |
| CONNECTION  |          LIVE MAP            | AUTO MAPPING    |
| RUN CONTROL |                              | MISSION         |
| ROBOT STATE |                              | EVENT LOG       |
+-------------+------------------------------+-----------------+
| MAP SIZE | TOOLS | MAP ACTIONS | VIEW TOGGLES | SIM SPEED     |
+--------------------------------------------------------------+
```

**สถานะหุ่น**: `DISCONNECTED / CONNECTING / CONNECTED / READY / RUNNING / MOVING /
PAUSED / STOPPED / MAPPING / NAVIGATING / ERROR / EMERGENCY STOP / TRACKING LOST`

**แถบเครื่องมือด้านล่าง จัดกลุ่มตามหน้าที่**:

| กลุ่ม | ปุ่ม |
| --- | --- |
| (ขนาดแผนที่) | `WIDTH` `HEIGHT` `RESIZE` |
| `TOOLS` | Select, Wall, Eraser, Start, Goal, Checkpoint, Robot, Obstacle |
| `EDIT` | `CLEAR WALLS` `RESET MAP` `RANDOM` |
| `ROTATE MAP` | `<< CCW` `CW >>` |
| `ROTATE ROBOT` | `<< LEFT` `RIGHT >>` `FACING: <ทิศปัจจุบัน>` |
| `FILE` | `SAVE` `LOAD` `USE DESIGN MAP` |
| `PLAN` | `A* PATH` `AUTO MAP` |
| `TURN ROBOT NOW` | `TURN LEFT` `TURN RIGHT` `ABOUT FACE` |
| `VIEW` | `TRAIL` `CLEAR TRAIL` `SENSORS` `PATH` `TRUTH` |
| `SIM SPEED` | `0.5x` `1x` `2x` `5x` `10x` |

คลิกลากบนแผนที่ = วาดกำแพงต่อเนื่อง, คลิกขวาที่หุ่น = หมุนทิศเริ่มต้น

### หมุนแผนที่ (Rotate Map)
`ROTATE MAP` หมุนทั้งแผนที่ทีละ 90 องศา — ทั้ง cell, กำแพงขอบช่อง, สถานะ "เคยส่องดูแล้ว",
Start / Goal / Checkpoint, ตำแหน่งและทิศเริ่มต้นของหุ่น ย้ายไปพร้อมกันหมด
และ width/height สลับกันเมื่อหมุนเป็นจำนวนคี่ (เช่น 6×9 → 9×6)

แผนที่ทุกชั้น (working map, design map, ground truth) หมุนพร้อมกันเพื่อให้อยู่เฟรมเดียวกัน
ถ้าเชื่อมต่อหุ่นอยู่จะ restart ให้อัตโนมัติ (re-zero odometry ที่ช่องเริ่มต้นใหม่ —
บนหุ่นจริงคือรีเซ็ตเฟรมเท่านั้น **ไม่ขับหุ่น**) และ pose เก่าถูกทิ้งทันที
เพื่อไม่ให้ marker ค้างอยู่ช่องเดิมของเฟรมเก่า

หมุนระหว่างภารกิจกำลังรันไม่ได้

### หมุนหุ่น (Rotate Robot)
มีสองแบบ แยกกันชัดเจน:

* `ROTATE ROBOT` (กลุ่ม EDIT) — หมุน **ทิศเริ่มต้น** ที่วางไว้บนแผนที่ (N/E/S/W)
  ปุ่ม `FACING:` แสดงทิศปัจจุบันและกดเพื่อหมุนตามเข็มได้
* `TURN ROBOT NOW` — สั่งให้หุ่น **จริง** หมุนอยู่กับที่เดี๋ยวนี้ (±90°, 180°)
  ผ่าน preflight เดียวกับภารกิจ: ต้อง CONNECT + ARM (ถ้าเป็นหุ่นจริง),
  ติด Emergency Stop แล้วสั่งไม่ได้, และหุ่นจริงต้องยืนยันใน dialog ก่อน

**บนแผนที่**:
* เส้นทางที่วางแผนไว้ (ประ) vs เส้นทางที่เดินไปแล้ว (ทึบเขียว)
* trail ของหุ่น**จริง** (เหลือง) — คนละเส้นกับเส้นทางที่วางแผน
* sensor ray ตามระยะและมุมจริงของแต่ละเซนเซอร์ หมุนตาม heading
* หุ่นหมุนตาม heading จริง, เปลี่ยนเป็นสีส้มเมื่อ tracking หลุด

**คีย์ลัด**: `Space` = A* path, `F1` = Emergency Stop, `1`–`8` = เลือกเครื่องมือ,
`R` / `Shift+R` = หมุนทิศเริ่มต้นของหุ่น CW / CCW, `[` / `]` = หมุนแผนที่ CCW / CW,
`Ctrl+S` / `Ctrl+L` = save / load แผนที่

---

## 9. รูปแบบไฟล์แผนที่

superset ของ `data/robot_map_plan.json` เดิม — ไฟล์เก่าโหลดได้เลย

```json
{
  "version": 2,
  "grid_info": {"rows": 9, "cols": 9, "cell_size_m": 0.6},
  "width": 9, "height": 9,
  "start": [0, 0], "goal": [8, 8],
  "checkpoints": [[3, 4], [6, 2]],
  "robot": {"cell": [0, 0], "dir": 0},
  "cells": [[1, 1, 1, "..."]],
  "walls": [{"pos": [1, 1], "walls": {"top": true, "right": false,
                                      "bottom": false, "left": false}}],
  "known_edges": [["h", 1, 1]]
}
```

---

## 10. การทดสอบ

```bash
python -m unittest tests.test_panel          # 77 tests, headless (ไม่ต้องมีหุ่น/หน้าต่าง)
```

ครอบคลุม: ขนาดแผนที่ 2×2 → 30×30, การวาด/ลบกำแพง, save/load, ไฟล์แผนที่เก่า,
A* (ทางง่าย/เขาวงกต/ไม่มีทาง/checkpoint หลายจุด/turn penalty/unknown cells),
coordinate transform, raycast, mapping, frontier, tracking + timeout + communication loss,
sim robot (เดิน/หมุน/ชน/E-stop), mission end-to-end (goal, checkpoints, E-stop,
dynamic obstacle + replan, corridor ตัน, tracking timeout, หมุนแผนที่/หมุนหุ่น, manual turn)
และ dashboard (render, วาดกำแพงด้วยเมาส์, วาง marker, dialog ยืนยันโหมด real, resize,
ปุ่มหมุน, และตรวจว่าทุกปุ่มอยู่ในหน้าต่างจริง)
