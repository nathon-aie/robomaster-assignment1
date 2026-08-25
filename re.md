คำสั่งแก้ไขที่ 1: ซิงค์ Threshold และใช้ Flag จาก RobotSensorSnapshot โดยตรง
- ปัญหา: คลาส WallCenteringPID ซ้ำซ้อน Logic โดยสร้าง classify_wall_state ขึ้นมาคำนวณใหม่ และใช้ WALL_DETECT_THRESHOLD_MM = 260.0 ซึ่งไม่ตรงกับ sensor_pipeline.py ที่ใช้ $280\text{ mm}$ สำหรับกำแพงข้าง และ $350\text{ mm}$ สำหรับกำแพงหน้า  
- การแก้ไข: ตัดการคำนวณระยะกำแพงซ้ำซ้อนใน WallCenteringPID ออกทั้งหมด แล้วเปลี่ยนไปเรียกใช้ Flag สรุปผลจาก state.wall_left_detected, state.wall_right_detected, และ state.wall_front_detected ที่ Thread 1 ประมวลผลไว้แล้วโดยตรง

คำสั่งแก้ไขที่ 2: ปรับปรุง Deadband และแก้ State Update ใน PIDController
- ปัญหา: เมื่อ abs(error) < deadband คลาส PIDController สั่ง return 0.0 ทันที ทำให้ไม่อัปเดต self.last_error และ self.last_time ส่งผลให้เมื่อ Error กลับมาเกิน Deadband ค่า derivative ($D\text{-term}$) จะเกิดกระชาก (Spike) เนื่องจาก $\Delta t$ ข้ามช่วงเวลาไป
- การแก้ไข: ย้าย Logic Deadband ไปไว้ที่ขั้นตอนสุดท้ายก่อนส่งค่า Output ออกมา โดยต้องอัปเดต self.last_error และ self.last_time ในทุกๆ เฟรมที่มีการเรียกใช้ compute() เสมอ

คำสั่งแก้ไขที่ 3: เพิ่ม PID ควบคุมแกน X ($v_x$) และรองรับการถอยหลังเมื่อเลยระยะ
- ปัญหา: การคำนวณ $v_x$ ใน compute_control_speeds ใช้การชะลอความเร็วเชิงเส้น เมื่อหุ่นยนต์ถลำเลยระยะหยุด ($tof < front\_target\_mm$) ค่า dist_to_stop จะติดลบ เข้าเงื่อนไข dist_to_stop <= 20.0 ทำให้ $v_x = 0$ ส่งผลให้หุ่นยนต์ค้าง ไม่สามารถถอยหลังกลับมาที่ระยะ $150\text{ mm}$ ได้
- การแก้ไข: สร้าง pid_longitudinal (PID สำหรับแกน X) เพื่อควบคุมระยะ $tof\_filtered\_mm$ เข้าหา front_target_mm ($150\text{ mm}$) โดยตรง ซึ่งจะทำให้หุ่นยนต์สามารถขับถอยหลัง ($v_x < 0$) ได้เมื่อเลยจุดหยุด

คำสั่งแก้ไขที่ 4: แก้ปัญหา Blocking Call ใน _poll_adcs_if_needed
- ปัญหา: ใน SensorCollectorThread การสั่ง get_adc() แบบ Synchronous Call เมื่อค่านำเข้าขาดหาย ทำให้เกิด Delay ใน Loop การทำงาน 20Hz ของ Thread 1 ส่งผลให้ดรอปเฟรม  
- การแก้ไข: ตัดเมธอด _poll_adcs_if_needed ออก หรือใส่ Non-blocking Timeout และแยกการจัดการ Thread ไม่ให้ดึง Latency รวมของ Sensor Collector

คำสั่งแก้ไขที่ 5: ปรับปรุงการคำนวณ valid flag ไม่ให้ State สลับไปมา (Oscillation)
- ปัญหา: ค่า sharp_left_valid ตกเป็น False ทันทีเมื่อเกิด Outlier เพียงเฟรมเดียว ทำให้ PID สลับ Case ไปมาระหว่าง Case 1.1 (มี 2 ข้าง) และ Case 1.2 (มีข้างเดียว) แบบฉับพลัน  
- การแก้ไข: ปรับใน SensorFilterPipeline ให้มี Counter หรือ Latch Logic เพื่อกรองค่า Valid ให้อยู่นิ่งอย่างน้อย 2-3 เฟรมก่อนเปลี่ยนสถานะ True/False