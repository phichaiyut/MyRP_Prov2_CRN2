/*
================================================================================
  📘 คู่มือคำสั่ง MyRP_ProV2_Chea2_IMU  (อัปเดตให้ตรงกับไลบรารีปัจจุบัน)
================================================================================
  ⚠️ ชื่อคำสั่งเป็นตัวพิมพ์เล็ก/ใหญ่ตามที่เขียนในคู่มือนี้เท่านั้น (C++ แยกตัวพิมพ์)
     เช่น ff() ไม่ใช่ FF(), spinlg() ไม่ใช่ SpinLG(), gostart() ไม่ใช่ GoStart()

  คำศัพท์
    speed / spd  = ความเร็วมอเตอร์ (0-100)
    time         = เวลา (ms)
    cm           = ระยะทาง (เซนติเมตร โดยประมาณ คำนวณจากความเร็ว x เวลา)
    'select'     = คำสั่งเมื่อเจอแยก (ดูตารางข้อ 5 / ข้อ 12)
    direction    = ทิศสัมบูรณ์ของไจโร (อ้างอิงจากตอนเปิดเครื่อง/resetAngles) เช่น 0, 90, 180, 270


================================================================================
  1) เริ่มต้น / ปุ่ม / เสียง / ไฟ
================================================================================
  RobotSetup();        // เรียกใน setup() ครั้งแรก (เปิด I2C, ไจโร, ADC, โหลดค่า calibrate จาก EEPROM)
  Setting();           // ฟังก์ชันตั้งค่าใน Setting.ino
  sw();                // หยุดมอเตอร์ แล้วรอกดปุ่ม
                       //   ปุ่ม 19 = calibrate เซนเซอร์หน้า (A)   ปุ่ม 12 = calibrate เซนเซอร์หลัง (B)
                       //   ปุ่ม 33 กดสั้น = เริ่มวิ่ง | กดค้าง 3 วิ = calibrate เซนเซอร์กลาง (C)

  Beep(ms);            // ปี๊บ ms มิลลิวินาที
  Beep2(freq, ms);     // ปี๊บกำหนดความถี่
  beep(freq, dur);
  BeepScanner();       // ปี๊บสั้นแหลมครั้งเดียว คล้ายเครื่องยิงบาร์โค้ด (= beep(ToNe, 80))
  BZon();  BZoff();    // เปิด / ปิดเสียงค้าง
  RGB();               // ไฟ RGB


================================================================================
  2) มอเตอร์พื้นฐาน (ไม่ใช้เซนเซอร์)
================================================================================
  Motor(L, R);              // สั่งมอเตอร์ซ้าย/ขวา (-100..100)
  Move(L, R, time);         // สั่งมอเตอร์ค้างไว้ time ms
  MotorStop();              // หยุด (ปล่อยไหล)
  MotorStop(ms);            // หยุดแล้วปี๊บ ms
  MotorShot();              // เบรกแบบลัดขั้ว (หยุดทันที กันไถล)

  fd(speed, time);  bk(speed, time);    // เดินหน้า / ถอยหลัง (ใช้ค่า balance ใน Setting.ino)
  sl(speed, time);  sr(speed, time);    // หมุนซ้าย / ขวา อยู่กับที่
  tl(speed, time);  tr(speed, time);    // เลี้ยวล้อเดียว ซ้าย / ขวา

  set_Freq("Coreless_Motors");          // ตั้งความถี่ PWM ตามชนิดมอเตอร์ (ใส่ใน Setting.ino)


================================================================================
  3) ออกตัว / เข้าเส้นชัย / จัดหุ่นให้ตรง
================================================================================
  gostart(speed);   gostart(L, R);    // วิ่งจนเซนเซอร์หน้าพ้นจุด Start
  goend(speed);     goend(L, R);      // วิ่งจนเซนเซอร์กลางเจอเส้น Finish แล้วหยุด

  // จัดหุ่นให้ตรงกับเส้น (Counter = จำนวนรอบ) — ทำเสร็จแล้วจะ SetRobotAngle() ให้อัตโนมัติ
  balancef(n);  setf(n);      // ด้านหน้า (เซนเซอร์หน้า F[0] / F[7])
  balanceb(n);  setb(n);      // ด้านหลัง (เซนเซอร์หลัง)
  balancefc(n); setfc(n);     // ใช้เซนเซอร์กลาง โดยเดินหน้า
  balancebc(n); setbc(n);     // ใช้เซนเซอร์กลาง โดยถอยหลัง

  set_f(n);   set_b(n);       // จัดตรงแบบเร็ว ด้านหน้า / ด้านหลัง
  set_fc(n);  set_bc(n);      // จัดตรงด้วยเซนเซอร์กลาง เดินหน้า / ถอยหลัง

  lf(time);   lb(time);       // ส่ายจัดหุ่นให้อยู่กลางเส้นอยู่กับที่ (เซนเซอร์หน้า / หลัง) แล้วหยุด


================================================================================
  4) วิ่งตามเส้น PID
================================================================================
  ▶ เดินหน้า (เซนเซอร์หน้า)                         ▶ ถอยหลัง (เซนเซอร์หลัง)
  ff(speed, 'select');                              bb(speed, 'select');
      // หยุดเมื่อ F[0] หรือ F[7] เจอเส้น หรือ F[2]+F[5] เจอเส้น

  ffc(speed, 'select');                             bbc(speed, 'select');      // แยก + (ซ้ายและขวาเจอพร้อมกัน)
  ffc2(speed, 'select');                            bbc2(speed, 'select');     // แยก + แบบหลวม (คู่ 0/7, 1/6 หรือ 2-5 เจอ)
  ffl(speed, 'select');   ffl0(speed, 'select');    bbl(...);  bbl0(...);      // แยกซ้าย (F[0])
  ffl2(speed, 'select');                            bbl2(speed, 'select');     // แยกซ้ายแบบเต็ม (F[0..4])
  ffr(speed, 'select');   ffr7(speed, 'select');    bbr(...);  bbr7(...);      // แยกขวา (F[7])
  ffr2(speed, 'select');                            bbr2(speed, 'select');     // แยกขวาแบบเต็ม (F[3..7])

  ffnum(speed, 'select', n);  ffn(...);             bbnum(...);  bbn(...);     // หยุดเมื่อเซนเซอร์ตัวที่ n (0-7) เจอเส้น
  ffwhite(speed, 'select');   ffw(...);             bbwhite(...);  bbw(...);   // หยุดเมื่อไม่เจอเส้นเลย (พื้นขาวทั้งแผง)
  ffblack(speed, 'select');   ffb(...);             bbblack(...);  bbb(...);   // วิ่งตรง (ไม่ PID) จนเจอเส้นดำ
  ffblack(L, R, 'select');    ffb(L, R, 'select');  bbblack(L, R, ...);  bbb(L, R, ...);

  ▶ หยุดด้วยเซนเซอร์วัดระยะ (analog ตั้งขาด้วย SetAnalogDistance)
  ff_distance(speed, 'select', ค่า);  ffd(...);     // เดินหน้าจนค่าเซนเซอร์ >= ค่า
  bb_distance(speed, 'select', ค่า);  bbd(...);     // ถอยจนค่าเซนเซอร์ <= ค่า
  ff_distances(speed, 'select', ค่า); ffds(...);    // เดินหน้า แล้วค่อย ๆ ผ่อนความเร็วจนหยุดที่ค่า (PID หยุด)

  ▶ ตามเวลา / ตามระยะ
  fftimer(speed, time);             bbtimer(speed, time);
  fftimer(speed, time, 'select');   bbtimer(speed, time, 'select');
  fft(speed, time, 'select');       bbt(speed, time, 'select');      // ชื่อย่อ (ต้องมี 'select')
  ffcm(speed, cm);                  bbcm(speed, cm);
  ffcm(speed, cm, 'select');        bbcm(speed, cm, 'select');


================================================================================
  5) ตารางคำสั่งเมื่อเจอแยก ('select') ของคำสั่ง PID (ff/bb/fft/ffcm/ffcl ฯลฯ)
================================================================================
   's' : เบรกหยุดทันที
   'S' : เดินช้าจนเซนเซอร์หน้า F[0]/F[7] เจอเส้นแล้วเบรก
   'p' : วิ่งต่อจนเซนเซอร์หน้าพ้นแยก
   'l' 'L' : เดินจนเซนเซอร์กลาง (ใต้ล้อ) เจอเส้น แล้วหมุนซ้าย spinl()
   'r' 'R' : เดินจนเซนเซอร์กลางเจอเส้น แล้วหมุนขวา spinr()
   'q' : เลี้ยวซ้ายทันที (TurnLeft)        'e' : เลี้ยวขวาทันที (TurnRight)
   'c' : เดินจนเซนเซอร์กลางเจอเส้นแล้วหยุด
   'a' : เดินจนเซนเซอร์กลางเจอเส้น แล้วหมุนซ้ายโดยใช้เซนเซอร์หลัง (spinl_B)
   'd' : เดินจนเซนเซอร์กลางเจอเส้น แล้วหมุนขวาโดยใช้เซนเซอร์หลัง (spinr_B)
   'b' : เดินจนเซนเซอร์หลังเจอเส้นแล้วหยุด
   'g' : หยุดแล้วล็อกมุมด้วยไจโร (speed = เวลาที่ล็อก ms)
   อื่น ๆ : หยุดแล้วปี๊บ

   ตัวพิมพ์ใหญ่ P Q E C A D B G ทำเหมือนตัวพิมพ์เล็ก แต่จะวิ่งเข้าหาเส้นด้วย PID ก่อน
   (ToFront() สำหรับเดินหน้า / ToBack() สำหรับถอยหลัง)
   หมายเหตุ: ตอนถอยหลัง 'q'/'e' ใช้ TurnRight_B()/TurnLeft_B() (ทิศกลับด้านจากเดินหน้า)


================================================================================
  6) เลี้ยว / หมุน ด้วยเซนเซอร์เส้น
================================================================================
  spinl();   spinr();          // หมุนซ้าย / ขวา จนเจอเส้นถัดไป (ความเร็วจาก SetTurnSpeed)
  spinl(speed);  spinr(speed);
  spinl2();  spinr2();         // หมุนข้าม 2 เส้น   (มีรุ่น spinl2(speed) / spinr2(speed))

  spinl_B();  spinr_B();       // หมุนโดยใช้เซนเซอร์หลัง   (มีรุ่นใส่ speed)
  spinl2_B(); spinr2_B();      // หมุนข้าม 2 เส้น ใช้เซนเซอร์หลัง

  TurnLeft();    TurnRight();     // เลี้ยวล้อเดียว เช็คเส้นด้วยเซนเซอร์หน้า (ตั้งด้วย TurnSpeedLeft/Right)
  TurnLeft_B();  TurnRight_B();   // เลี้ยวโดยใช้เซนเซอร์หลัง (ตั้งด้วย TurnBackSpeedLeft/Right)
  TurnLeftBackF();  TurnRightBackF();  // เลี้ยวล้อเดียวถอยหลัง เช็คเส้นด้วยเซนเซอร์หน้า F[]
  TurnLeftBackB();  TurnRightBackB();  // เลี้ยวล้อเดียวถอยหลัง เช็คเส้นด้วยเซนเซอร์หลัง B[]
      // ตั้งความเร็วใน Setting.ino: TurnSpeedLeftBackF/RightBackF/LeftBackB/RightBackB(l, r, delay)


================================================================================
  7) เข้ากลางหุ่น / วิ่งเข้าหาเส้น
================================================================================
  ToCenter();     // เดินหน้าจนเซนเซอร์กลางซ้ายหรือขวาเจอเส้น แล้วหยุด
  ToCenterL();    // จนเซนเซอร์กลางซ้ายเจอเส้น
  ToCenterR();    // จนเซนเซอร์กลางขวาเจอเส้น
  BackCenter();   // ถอยหลังจนเซนเซอร์กลางเจอเส้น
  ToFront();      // เดินช้าด้วย PID จนเซนเซอร์หน้า F[0]/F[7] เจอเส้น
  ToBack();       // ถอยช้าด้วย PID จนเซนเซอร์หลัง B[0]/B[7] เจอเส้น
  // ความเร็วตั้งด้วย SetToCenterSpeed(); โหมดเดินด้วย set_line_center(0 = เดินตรง | 1 = ตามเส้น)


================================================================================
  8) วิ่งโค้งตามเส้น (Circle: เลื่อนจุดกลางเส้นไปซ้าย/ขวาชั่วคราว)
================================================================================
  // CL = โค้งซ้าย (ใช้ set_position_line_l) | CR = โค้งขวา (ใช้ set_position_line_r)
  ffcl(speed, 'select');            ffcr(speed, 'select');          // จนเซนเซอร์ F[0] / F[7] เจอเส้น
  fftimercl(speed, time);           fftimercr(speed, time);
  fftimercl(speed, time, 'select'); fftimercr(speed, time, 'select');
  ffcmcl(speed, cm);                ffcmcr(speed, cm);              // มีเร่ง/ผ่อนความเร็วต้น-ท้าย
  ffcmcl(speed, cm, 'select');      ffcmcr(speed, cm, 'select');

  bbcl(speed, 'select');            bbcr(speed, 'select');
  bbtimercl(speed, time);           bbtimercr(speed, time);
  bbtimercl(speed, time, 'select'); bbtimercr(speed, time, 'select');
  bbcmcl(speed, cm);                bbcmcr(speed, cm);
  bbcmcl(speed, cm, 'select');      bbcmcr(speed, cm, 'select');


================================================================================
  9) แขนกล / Servo
================================================================================
  // ขา: 36 = ยกขึ้นลง | 34 = มือซ้าย | 35 = มือขวา (+ 37, 38, 39 ใช้ทั่วไป)
  Servo(ขา, องศา);                    // เช่น Servo(36, 90);
  Servo(ยก, ซ้าย, ขวา);                // สั่ง 3 ตัวพร้อมกัน
  Servo(ยก, ซ้าย, ขวา, ความเร็ว);      // ค่อย ๆ ขยับ (ความเร็ว = ms ต่อ 1 องศา ยิ่งมากยิ่งช้า)
  S34_trim(x);  S35_trim(x);  S36_trim(x);   // ชดเชยองศา servo

  armupdown(องศา);                     armupdown(องศา, ความเร็ว);          // ยกขึ้นลง
  arm_left_right(ซ้าย, ขวา);           arm_left_right(ซ้าย, ขวา, ความเร็ว); // มือซ้าย ขวา

  // ท่าสำเร็จรูป (อยู่ใน Servo.ino ปรับองศาได้ที่ตัวแปรด้านบนไฟล์) — ทุกท่ามีรุ่นใส่ความเร็ว เช่น arm_up(2)
  arm_ready();       // แขนลง กางมือเตรียมคีบ
  arm_open_down();   arm_down_open();    // กางมือ + เอาแขนลง (ลำดับต่างกัน)
  arm_open_up();     arm_up_open();      // กางมือ + ยกแขน
  arm_down_close();  arm_close_down();   // เอาแขนลง + หุบมือ
  arm_up_close();    arm_close_up();     // ยกแขน + หุบมือ
  arm_big_box();     arm_big_box_up();   // คีบกล่องใหญ่ / คีบแล้วยก
  arm_up();  arm_up45();  arm_down();    // ยกแขน / ยก 45 / ลดแขน
  arm_open();  arm_close();  arm_big();  // กางมือ / หุบมือ / หุบลูกใหญ่
  arm_open_l();  arm_open_r();           // กางมือซ้าย / ขวา
  arm_behihd();                          // มือไปด้านหลัง (สะกดตามชื่อฟังก์ชันใน Servo.ino)

  SerialServoControl();   // ทดสอบ servo ผ่าน Serial พิมพ์ "ขา องศา" เช่น 36 90, พิมพ์ exit เพื่อออก
  Servo_34.detach();      // ปลด servo (Servo_34 ... Servo_39)


================================================================================
  10) ไจโร: ตั้งค่า / อ่านมุม / ล็อกมุม
================================================================================
  resetAngles();       // รีเซ็ตมุมเป็น 0 (ทำตอนหุ่นนิ่ง — RobotSetup เรียกให้แล้ว)
  SetRobotAngle();     // จำทิศปัจจุบันเป็นทิศที่จะวิ่งตรง
  gyroZ();             // อ่านมุมปัจจุบัน (องศา)

  SetG(time);   setg(time);     // หมุนอยู่กับที่ให้กลับมาทิศที่จำไว้ (current_degree) นาน time ms
  SetGB(time);  setgb(time);    // แบบเดียวกัน สำหรับหลังถอยหลัง
  SetFG(time);  setfg(time);    // จำทิศตอนนี้ แล้วล็อกไว้นาน time ms


================================================================================
  11) ไจโร: หมุน / เลี้ยวตามองศา
================================================================================
  // ค่าองศา: (+) = ขวา , (-) = ซ้าย  | ไม่ใส่ speed = ใช้ maxSpd จาก SetGyroSpin / SetGyroTurn
  spindegree(deg);    spindegree(speed, deg);     // หมุนอยู่กับที่
  turndegree(deg);    turndegree(speed, deg);     // เลี้ยวล้อเดียวเดินหน้า
  turndegreeb(deg);   turndegreeb(speed, deg);    // เลี้ยวล้อเดียวถอยหลัง (+ = ท้ายไปซ้าย/หน้าหันขวา)
  turndegree_none(deg);   turndegree_none(speed, deg);    // เลี้ยวแบบไม่หยุด (ต่อคำสั่งถัดไปได้เลย, default speed 50)
  turndegreeb_none(deg);  turndegreeb_none(speed, deg);
  rotatedegree(speedL, speedR, deg);              // วิ่งโค้ง กำหนดความเร็วล้อซ้าย/ขวาเอง จนได้องศา
  rotatedegree(speedL, speedR, deg, kp, kd);
  // ตัวอย่าง: turndegree(90);  spindegree(-90);  turndegreeb(-90);

  ▶ ชื่อสั้น (ใส่องศาเป็นบวกเสมอ ฟังก์ชันกำหนดทิศเอง)  — ทุกตัวมีรุ่น (speed, deg)
  spinlg(deg);  spinrg(deg);     หรือ  slg(deg);  srg(deg);      // หมุนซ้าย / ขวา
  turnlg(deg);  turnrg(deg);     หรือ  tlg(deg);  trg(deg);      // เลี้ยวซ้าย / ขวา เดินหน้า
  turnlbg(deg); turnrbg(deg);    หรือ  tlbg(deg); trbg(deg);     // เลี้ยวซ้าย / ขวา ถอยหลัง

  ▶ เลี้ยวต่อเนื่อง 2 จังหวะ (ไม่หยุดกลางทาง) — เช่นหลบสิ่งกีดขวาง / ย้ายเลน
  tlrg(deg);   tlrg(speed, deg);   tlrg(speed, deg1, deg2);     // เลี้ยวซ้ายแล้วขวา
  trlg(deg);   trlg(speed, deg);   trlg(speed, deg1, deg2);     // เลี้ยวขวาแล้วซ้าย
  tlrbg(deg);  tlrbg(speed, deg);  tlrbg(speed, deg1, deg2);    // ถอยหลัง ซ้ายแล้วขวา
  trlbg(deg);  trlbg(speed, deg);  trlbg(speed, deg1, deg2);    // ถอยหลัง ขวาแล้วซ้าย

  ▶ หมุน / เลี้ยวไปยังทิศสัมบูรณ์ (0, 90, 180, 270 ...) ไม่ว่าตอนนี้หันทิศไหน
  spindirection(dir);         spindirection(speed, dir);
  turndirection(dir);         turndirection(speed, dir);
  turndirectionb(dir);        turndirectionb(speed, dir);
  turndirection_none(dir);    turndirection_none(speed, dir);
  turndirectionb_none(dir);   turndirectionb_none(speed, dir);
  relativeToDirection(dir);   // คืนค่ามุมที่ต้องหมุน (-180..180) ไปยังทิศนั้น
  // ตัวอย่าง: spindirection(90);


================================================================================
  12) ไจโร: วิ่งตรง
================================================================================
  ▶ ตามเวลา
  fftimerg(speed, time);              bbtimerg(speed, time);
  fftimerg(speed, time, 'select');    bbtimerg(speed, time, 'select');
  fftg(speed, time, 'select');        bbtg(speed, time, 'select');       // ชื่อย่อ (ต้องมี 'select')

  ▶ ตามระยะ (cm)
  ffcmg(speed, cm);                   bbcmg(speed, cm);                  // ระยะ >= 30 cm มีเร่ง/ผ่อนความเร็ว
  ffcmg(speed, cm, 'select');         bbcmg(speed, cm, 'select');
  ffcmgs(speed, cm);                  bbcmgs(speed, cm);                 // ความเร็วคงที่ ไม่มีเร่ง/ผ่อน
  ffcmgs(speed, cm, 'select');        bbcmgs(speed, cm, 'select');

  ▶ จนเจอเส้น / ตามเซนเซอร์ระยะ
  ffbg(speed, 'select');              bbbg(speed, 'select');             // จนเซนเซอร์หน้า/หลังเจอเส้นดำ
  ffdg(speed, 'select', ค่า);         bbdg(speed, 'select', ค่า);        // จนเซนเซอร์ระยะถึงค่า
  ffdgs(speed, 'select', ค่า);        bbdgs(speed, 'select', ค่า);

  ▶ ล็อกทิศสัมบูรณ์ระหว่างวิ่ง: ใส่ direction ต่อท้ายสุด
    dir = ทิศที่หน้าหุ่นหัน ทั้งเดินหน้าและถอยหลัง
    เช่น bbcmg(speed, cm, 'p', 180) = หน้าหุ่นหันทิศ 180 แล้วถอยหลัง
  fftimerg(speed, time, dir);              bbtimerg(speed, time, dir);
  fftimerg(speed, time, 'select', dir);    bbtimerg(speed, time, 'select', dir);
  fftg(speed, time, 'select', dir);        bbtg(speed, time, 'select', dir);
  ffcmg(speed, cm, dir);                   bbcmg(speed, cm, dir);
  ffcmg(speed, cm, 'select', dir);         bbcmg(speed, cm, 'select', dir);
  ffcmgs(speed, cm, dir);                  bbcmgs(speed, cm, dir);
  ffcmgs(speed, cm, 'select', dir);        bbcmgs(speed, cm, 'select', dir);
  ffbg(speed, 'select', dir);              bbbg(speed, 'select', dir);
  SetDirectionG(dir);   // ตั้งทิศเป้าหมายเองก่อนวิ่ง

  ▶ เข้ากลางหุ่นด้วยไจโร (วิ่งตรงด้วยไจโรแทน PID เส้น)
  ToCenterLRG();   // จนเซนเซอร์กลางซ้ายหรือขวาเจอเส้น
  ToCenterLG();    ToCenterRG();    // เฉพาะซ้าย / ขวา
  BackCenterG();   // ถอยหลังจนเซนเซอร์กลางเจอเส้น
  ToFrontG();      ToBackG();       // วิ่งจนเซนเซอร์หน้า / หลังเจอเส้น (ไม่หยุดมอเตอร์)

  ▶ ตาราง 'select' ของคำสั่งไจโร (ต่างจากข้อ 5)
   'L' / 'R' : หมุนซ้าย / ขวา 90° ทันที
   'l' / 'r' : เข้ากลางหุ่น (ToCenterLRG / BackCenterG) แล้วหมุนซ้าย / ขวา 90°
   'q' 'Q' / 'e' 'E' : เลี้ยวล้อเดียวซ้าย / ขวา 90° (ถอยหลังใช้ turndegreeb)
   'p' 'P' : วิ่งต่อจนพ้นแยก        'c' 'C' : จนเซนเซอร์กลางเจอเส้นแล้วหยุด
   'b' 'B' : จนเซนเซอร์หลังเจอเส้นแล้วหยุด
   's' : เบรกหยุดทันที              'S' : วิ่งจนเจอเส้นแล้วเบรก
   'G' : วิ่งจนเจอเส้นแล้วล็อกมุม    อื่น ๆ : ล็อกมุมด้วย SetG(speed)


================================================================================
  13) ตั้งค่า (ใส่ใน Setting.ino)
================================================================================
  ▶ เซนเซอร์เส้น
  RefLineValue(x);            // threshold เซนเซอร์หน้า-หลัง
  RefCenterLineValue(x);      // threshold เซนเซอร์กลาง
  TrackLineColor(0|1);        // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว
  Dottedline(0|1);            // 0 = ไม่มีเส้นประ | 1 = มีเส้นประ
  clampSensorValueF(min, max);  clampSensorValueB(min, max);  clampSensorValueC(min, max);  // กรองค่า calibrate
  SetAnalogDistance(A0);      // ขาเซนเซอร์วัดระยะ (A0-A3)

  ▶ PID เส้น
  Set_KP_KD(SPD_xx, kp, kd);  Set_KP_KD_Back(SPD_xx, kp, kd);   // KP/KD ตามช่วงความเร็ว SPD_10 ... SPD_100
  setBalanceSpeed(SPD_xx, L, R);  setBalanceBackSpeed(SPD_xx, L, R);   // ชดเชยล้อที่แรงกว่า
  SetDelayBreak(SPD_xx, f, b);    // เวลาเบรกตอนหยุดที่เส้น (ms) เดินหน้า / ถอยหลัง
  ModeSpdPID(mode, max, min);     // โหมดจำกัดกำลังล้อขณะ PID
  SetPIDDeadBand(x);              // error ที่เล็กกว่านี้ถือเป็น 0 (กันส่าย, default 20)
  set_position_line(2500);        // จุดกลางเส้น (1000 = ซ้าย 2500 = กลาง 4000 = ขวา)
  set_position_line_l(500);  set_position_line_r(4500);   // จุดกลางตอนวิ่งโค้ง CL / CR
  set_line_center(0|1);           // เข้ากลางหุ่น 0 = เดินตรง | 1 = ตามเส้น
  SetToCenterSpeed(x);            // ความเร็วเข้ากลางหุ่น / ToFront / ToBack
  set_brake_fc(ff, fc);  set_brake_bc(bf, bc);   // เวลาเบรกหลังเข้ากลางหุ่น

  ▶ การเลี้ยว
  SetTurnSpeed(x);                         // ความเร็ว spinl / spinr ('l' 'r')
  TurnSpeedLeft(l, r, delay);  TurnSpeedRight(l, r, delay);            // TurnLeft / TurnRight ('q' 'e')
  TurnBackSpeedLeft(l, r, delay);  TurnBackSpeedRight(l, r, delay);    // TurnLeft_B / TurnRight_B
  TurnSpeedLeftBackF(...);  TurnSpeedRightBackF(...);                  // TurnLeftBackF / TurnRightBackF
  TurnSpeedLeftBackB(...);  TurnSpeedRightBackB(...);                  // TurnLeftBackB / TurnRightBackB
  SetSensorTurnLeftRight(l, r);     // เซนเซอร์หน้าที่หยุด TurnLeft (F[0..l]) / TurnRight (F[7..r])
  SetSpinDebounceCount(n);          // จำนวนครั้งที่อ่านซ้ำก่อนยอมรับว่าพ้น/เจอเส้น (default 5)

  ▶ ไจโร
  SetGyroTurn(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // turndegree / turndegreeb
  SetGyroSpin(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // spindegree
  SetGyroRun(kp, kd);   SetGyroRunB(kp, kd);                  // วิ่งตรงเดินหน้า / ถอยหลัง
  ModeSpdGyro(mode, max, min);   ModeSpdGyro(modeF, modeB, max, min);

  ▶ โหมดจำกัดกำลังล้อ (ModeSpdPID / ModeSpdGyro)
    0 = 0..max (ล้อติดลบ → min) | 1 = min..max | 2 = -Speed..Speed | 3 = ..max (ติดลบ → -Speed) | 4 = 0..Speed


================================================================================
  14) ดูค่าผ่าน Serial Monitor (วนลูปไม่จบ ใช้ทดสอบเท่านั้น)
================================================================================
  Serial_FrontSensor();   Serial_BackSensor();   Serial_CenterSensor();   Serial_AllSensor();
  SerialCalibrate_FrontSensor();  SerialCalibrate_BackSensor();
  SerialCalibrate_CenterSensor(); SerialCalibrate_AllSensor();
  SerialPositionF();  SerialPositionB();  SerialPositionFB();   // ตำแหน่งเส้น
  SerialDistance();                                             // เซนเซอร์วัดระยะ

================================================================================
*/
