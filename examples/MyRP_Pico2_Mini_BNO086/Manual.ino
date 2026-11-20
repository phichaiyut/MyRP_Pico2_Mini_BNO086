/* ==========================================================================
   📘  คู่มือคำสั่ง MyRP_Pico2_mini_BNO086
       (ไฟล์นี้เป็น comment ล้วน ไม่มีผลกับโค้ด)
       กด Ctrl+F แล้วพิมพ์ [1] [2] ... เพื่อกระโดดไปหมวดที่ต้องการ
   --------------------------------------------------------------------------
   [1]  โครงโปรแกรม / ความหมายพารามิเตอร์     [10] Gyro หมุน / เลี้ยว
   [2]  ตั้งค่า (Setting.ino)                    [11] Gyro วิ่งตรง
   [3]  ตารางคำสั่งทางแยก 'select'              [12] Gyro ตามทิศ (Heading) + ตัวอย่าง
   [4]  เดินตามเส้น เดินหน้า  ff...              [13] เดินตามเส้น / โค้งจนถึงทิศ ffdir / ffdircl
   [5]  เดินตามเส้น ถอยหลัง  bb...              [14] มอเตอร์ / แบตเตอรี่
   [6]  วิ่งโค้ง cl / cr                         [15] แขน / เซอร์โว
   [7]  เข้ากึ่งกลาง / ออกตัว / เข้าเส้นชัย       [16] เซนเซอร์ / Calibrate
   [8]  หมุน / เลี้ยว ด้วยเซนเซอร์เส้น            [17] Debug (Serial Monitor)
   [9]  Gyro พื้นฐาน / ล็อกมุม                   [18] เสียง / ไฟ
   ========================================================================== */


/* ==========================================================================
   [1] โครงโปรแกรม / ความหมายพารามิเตอร์
   ========================================================================== */
//
// setup() {  RobotSetup();  Setting();  sw();  SetRobotAngle();  Mission();  }
//
//   speed   ความเร็ว 0-100
//   L, R    ความเร็วล้อซ้าย / ขวา  (-100 ถึง 100, ติดลบ = ถอย)
//   ms      เวลา (มิลลิวินาที)
//   cm      ระยะทาง (เซนติเมตร, ประมาณจากเวลา x ความเร็ว)
//   dist    ค่าเซนเซอร์วัดระยะ analogRead (ยิ่งใกล้ ค่ายิ่งมาก)
//   'p'     ตัวอักษรคำสั่งทางแยก ดูตาราง [3]
//   มุม     ลบ = ซ้าย, บวก = ขวา


/* ==========================================================================
   [2] ตั้งค่า (Setting.ino)
   ========================================================================== */
// --- มอเตอร์ ---
// set_Freq("Coreless_Motors");             // 20 kHz  (หรือ "coreless")
// set_Freq("DC_Motors");                   // 1 kHz
//
// --- กรองค่า calibrate ---
// clampSensorValueF(min, max);   clampSensorValueB(min, max);   clampSensorValueC(min, max);
//
// --- เซนเซอร์เส้น ---
// RefLineValue(500);                       // threshold เซนเซอร์หน้า-หลัง (0-1000)
// RefCenterLineValue(500);                 // threshold เซนเซอร์คู่กลาง (0-1000)
// TrackLineColor(0);                       // 0 = พื้นขาวเส้นดำ | 1 = พื้นดำเส้นขาว
// Dottedline(0);                           // 0 = ไม่มีเส้นประ | 1 = มีเส้นประ
//
// --- เข้ากึ่งกลาง ---
// set_line_center(1);                      // 0 = วิ่งตรงเข้ากลาง | 1 = เดินตามเส้นเข้ากลาง
// SetToCenterSpeed(20);                    // ความเร็วตอนเข้ากลาง
// set_brake_fc(ff, fc);                    // เวลาเบรก (ms) ตอนเดินหน้า
// set_center_push_ms(ms);                  // เวลาดันผ่านเส้นก่อนเช็กเซนเซอร์กลาง ToCenter / ToCenterG / 'c' 'b' (ค่าเริ่มต้น 20, -1 = นับ 20 รอบแบบเดิม)
// set_line_timeout(ms [, halt]);           // safety: วิ่งหาเส้นนานเกิน ms → หยุดมอเตอร์ (0 = ปิด ค่าเริ่มต้น)
//                                          //   halt = true ค้างไว้บี๊บจนกด reset | false = หยุดแล้วทำคำสั่งถัดไป
// set_brake_bc(bf, bc);                    // เวลาเบรก (ms) ตอนถอยหลัง
// set_pid_lf(kp, kd);  set_pid_lb(kp, kd); // PID ของ lf / lb (ค่าเริ่มต้น 0.015, 0.10)
//
// --- ความเร็วเลี้ยว (ใช้กับ select l r q e) ---
// SetTurnSpeed(50);                        // ความเร็วหมุน 'l' 'r'
// TurnSpeedLeft(L, R, ms);                 TurnSpeedRight(L, R, ms);         // 'q' 'e'  TurnLeft / TurnRight
// SetSensorTurnLeftRight(l, r);            // เซนเซอร์หน้าที่ใช้หยุด TurnLeft (F[0..l]) / TurnRight (F[7..r])
// SetSensorTurnLeftRightB(l, r);           // เซนเซอร์หลังที่ใช้หยุด TurnLeftB (B[7..l]) / TurnRightB (B[0..r])
// TurnBackSpeedLeft(L, R, ms);             TurnBackSpeedRight(L, R, ms);     // TurnLeftB / TurnRightB
// TurnSpeedLeftBackF(L, R, ms);            TurnSpeedRightBackF(L, R, ms);    // TurnLeftBackF / TurnRightBackF
// TurnSpeedLeftBackB(L, R, ms);            TurnSpeedRightBackB(L, R, ms);    // TurnLeftBackB / TurnRightBackB
//
// --- โหมดจำกัดความเร็วล้อ ---
// ModeSpdPID(mode, max, min);                                                // PID เส้น
// set_pid_mode2_limit(up, down);           // PID เส้น โหมด 2: -(Speed+down)..(Speed+up) (ค่าเริ่มต้น 20, 20)
// ModeSpdGyro(mode, max, min);             ModeSpdGyro(modeF, modeB, max, min);  // Gyro (แยกหน้า/หลังได้)
//     mode 0 = 0..max   1 = min..max   2 = -Speed..Speed   3 = ..max   4 = 0..Speed (ไม่ถอยล้อ)
// set_gyro_mode2_limit(up, down);          // Gyro โหมด 2: -(Speed+down)..(Speed+up) (ค่าเริ่มต้น 20, 20)
// set_gyro_mode4_limit(up);                // Gyro โหมด 4: 0..(Speed+up) (ค่าเริ่มต้น 20)
//
// --- PID ไจโร ---
// SetGyroTurn(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // เลี้ยวล้อเดียวเดินหน้า (turndegree)
// SetGyroTurnB(kp, kd, maxSpd, minSpd, smallAngle, stopThr);  // เลี้ยวล้อเดียวถอยหลัง (turndegreeb)
// SetGyroSpin(kp, kd, maxSpd, minSpd, smallAngle, stopThr);   // หมุนอยู่กับที่ (spindegree)
// SetGyroRun(kp, kd);                      SetGyroRunB(kp, kd);              // วิ่งตรง RunG / RunGB
// set_gyro_run_dterm_fresh(true);          // Kd ของ RunG / RunGB คิดเฉพาะตอนได้มุมใหม่ (มีผลจริง ต้องจูน kd ใหม่) | false = แบบเดิม (ค่าเริ่มต้น)
// SetGyroRotate(kp, kd, minSpd, smallAngle, stopThr);         // วิ่งโค้งกำหนดล้อเอง (rotatedegree / rotatedir)
//     ค่าเริ่มต้น (1.0, 2.0, 15, 25.0, 1.0) | ช่วงชะลอ: ความเร็วทั้ง 2 ล้อ × scale (สัดส่วนล้อเท่าเดิม)
//     scale = kp × (มุมที่เหลือ + kd × องศาที่เข้าใกล้ต่อ 10ms) / smallAngle
//     หมุนเลยมุม → เพิ่ม kd หรือ smallAngle | ชะลอนาน / ไปไม่ถึง → ลด kd หรือเพิ่ม minSpd
//
// --- เบรกหลังหมุน / เลี้ยวด้วยไจโร (ใช้กับทั้งคำสั่งตามมุม และตามทิศ [12]) ---
// set_brake_spin_gyro(spd, ms);            set_brake_spin_gyro_setg(ms);     // spindegree / spindir
// set_brake_turn_gyro(spd, ms);            set_brake_turn_gyro_setg(ms);     // turndegree / turndir
// set_brake_turn_b_gyro(spd, ms);          set_brake_turn_b_gyro_setg(ms);   // turndegreeb / turndirb
// set_brake_rotate_gyro(spd, ms);          set_brake_rotate_gyro_setg(ms);   // rotatedegree / rotatedir
//     (spd, ms)  = เบรกสวนทิศสั้น ๆ ทันทีที่ถึงมุม (ค่าเริ่มต้น 12, 5 | rotate = 15, 10)
//                  rotate: ล้อที่เร็วกว่าเบรกแรง spd ล้อที่ช้ากว่าเบรกตามสัดส่วน สวนทิศที่แต่ละล้อหมุนอยู่
//     _setg(ms)  = เวลาล็อกมุม SetG ต่อหลังเบรก (ค่าเริ่มต้น 5 | rotate = 20)
//
// --- กันหุ่นหมุนค้าง (safety timeout ของคำสั่งหมุน / เลี้ยว / ffdir ด้วยไจโร) ---
// set_gyro_timeout(stale_ms, total_ms);    // ค่าเริ่มต้น (200, 6000) | 0 = ปิด
//     stale_ms = IMU ไม่ส่งค่ามุมใหม่นานเกินนี้ → หยุดมอเตอร์ + บี๊บ ออกจากคำสั่ง (ใช้กับ ffdir / ffdeg ด้วย)
//     total_ms = หมุน / เลี้ยวนานเกินนี้ (ติดขอบ / ล้อฟรี) → หยุดมอเตอร์ + บี๊บ (ไม่ใช้กับ ffdir / ffdeg)
//
// --- ตัวคูณแปลงความเร็ว → ระยะทาง (Gyro วิ่งตามระยะ [11] [12]) ---
// set_speed_scale_ff_g(normal, short);     // ffcmg: normal = ระยะ >= 30 cm (มีเร่ง/ผ่อน), short = < 30 cm (ค่าเริ่มต้น 0.99, 1.5)
// set_speed_scale_bb_g(normal, short);     // bbcmg  (ค่าเริ่มต้น 0.99, 1.5)
// set_speed_scale_ff_bb_gs(ff, bb);        // ffcmgs / bbcmgs  (ค่าเริ่มต้น 1.65, 1.65)
//     วิ่งเกินระยะ = เพิ่มค่า, วิ่งไม่ถึง = ลดค่า
//
// --- ทางแยก Gyro ---
// set_track_snap90(true);                  // ทางแยก *g / *gb ล็อกแกน 90° อัตโนมัติ (ค่าเริ่มต้น) | false = ปิด  ดู [12]
//
// --- ตำแหน่งเส้น (0-5000, กลาง = 2500) ---
// set_position_line(2500);                 // ตำแหน่งปกติ
// set_position_line_l(500);                set_position_line_r(4500);        // วิ่งโค้ง cl / cr
//
// --- เซนเซอร์วัดระยะ ---
// SetAnalogDistance(28);                   // ขาอนาล็อก A0-A3 (26-29)
// set_ff_distances_pid(kp, kd, minSpd);    // ff_distances / ffds: ชะลอตามระยะ (ค่าเริ่มต้น 0.35, 0.8, 10)
//
// --- ตารางตามความเร็ว (ch = SPD_10, SPD_20 ... SPD_100 หรือใส่เลข 10, 20 ... 100 ตรง ๆ ก็ได้) ---
// setBalanceSpeed(ch, L, R);               // balance เดินหน้า (ล้อไหนแรงกว่า เพิ่มค่าข้างนั้น)
// setBalanceBackSpeed(ch, L, R);           // balance ถอยหลัง
// Set_KP_KD(ch, kp, kd);                   // PID เดินหน้า
// Set_KP_KD_Back(ch, kp, kd);              // PID ถอยหลัง
// SetDelayBreak(ch, f, b);                 // เวลาเบรก (ms) ตอนหยุดที่เส้น (ไถลเลย = เพิ่ม, ถอยเลย = ลด)


/* ==========================================================================
   [3] ตารางคำสั่งทางแยก 'select'  (ตัวสุดท้ายของ ff / bb / *g)
   --------------------------------------------------------------------------
     ตัวอักษร  | ff / bb (เดินตามเส้น)               | *g / *gb (Gyro)
     ---------+-------------------------------------+----------------------------
     's'      | หยุด                                 | หยุด
     'S'      | วิ่งถึงขอบเส้นแล้วหยุด                 | หยุด
     'p' 'P'  | วิ่งผ่าน                               | วิ่งผ่าน
     'c' 'C'  | เข้ากลางแล้วหยุด                       | เข้ากลางแล้วหยุด
     'l' 'L'  | เข้ากลาง + หมุนซ้าย                    | 'l' หมุนซ้ายทันที  'L' เข้ากลาง+หมุนซ้าย
     'r' 'R'  | เข้ากลาง + หมุนขวา                    | 'r' หมุนขวาทันที  'R' เข้ากลาง+หมุนขวา
     'q' 'Q'  | เลี้ยวโค้งซ้าย  (bb = ขวา)             | เลี้ยวโค้งซ้าย  (gb = ขวา)
     'e' 'E'  | เลี้ยวโค้งขวา  (bb = ซ้าย)             | เลี้ยวโค้งขวา  (gb = ซ้าย)
     'a' 'A'  | หมุนซ้ายด้วยเซนเซอร์หลัง               | -
     'd' 'D'  | หมุนขวาด้วยเซนเซอร์หลัง               | -
     'b' 'B'  | เข้ากลางแล้ววิ่งต่อถึงเส้นอีกฝั่ง          | เหมือนกัน
     'g' 'G'  | ปรับตรงด้วย gyro                      | ตัวอื่น ๆ = SetG(100)
   --------------------------------------------------------------------------
     ตัวพิมพ์ใหญ่ (P C Q E G) = เรียก ToFront() / ToBack() จัดตรงเส้นก่อน
     Gyro: 'L' 'R' = ToCenterG() / BackCenterG() ก่อน | P C Q E S B G = ToFrontG() / ToBackG() ก่อน
     Gyro: l L r R q Q e E ล็อกแกน 90° อัตโนมัติ (snap90) ปิดได้ด้วย set_track_snap90(false) ดู [12]
   ========================================================================== */


/* ==========================================================================
   [4] เดินตามเส้น เดินหน้า  ff...
   ========================================================================== */
// ff(speed, 'p');                          // วิ่งจนเจอแยกใดก็ได้ (F0 หรือ F7)
// ffc(speed, 'p');      ffc2(speed, 'p');  // เจอแยกตัด (F0 + F7) | ffc2 ยอมรับแยกกว้าง/แคบ
// ffl(speed, 'p');                         // เจอแยกซ้าย (F0)
// ffl2(speed, 'p');                        // เจอแยกซ้ายเต็ม (F0-F4)
// ffr(speed, 'p');                         // เจอแยกขวา (F7)
// ffr2(speed, 'p');                        // เจอแยกขวาเต็ม (F3-F7)
// ffnum(speed, 'p', n); ffn(speed, 'p', n);    // เจอเส้นที่เซนเซอร์ F[n] (0-7)
// ffsum(speed, 'p', sum);                      // ผลรวม F[0]..F[7] >= sum (0-8000)
// ffwhite(speed, 'p');  ffw(speed, 'p');       // วิ่งจนไม่เจอเส้นเลย
// ffblack(speed, 'p');  ffb(speed, 'p');       // วิ่งตรง (ไม่ PID) จนเจอเส้น
// ffblack(L, R, 'p');   ffb(L, R, 'p');
// ff_distance(speed, 'p', dist);   ffd(speed, 'p', dist);     // จนเซนเซอร์วัดระยะถึง dist
// ff_distances(speed, 'p', dist);  ffds(speed, 'p', dist);    // แบบค่อย ๆ ชะลอ
// fftimer(speed, ms);   fftimer(speed, ms, 'p');              // วิ่งตามเวลา
// fft(speed, ms);       fft(speed, ms, 'p');                  // ชื่อย่อของ fftimer
// ffcm(speed, cm);      ffcm(speed, cm, 'p');                 // วิ่งตามระยะทาง
// lf(ms);                                  // PID อยู่กับที่ จัดตัวตรงเส้น
//
// --- กำหนด Kp / Kd เอง (ใส่ต่อจาก speed) ---
// ใช้ได้กับ ff ffc ffc2 ffl ffl2 ffr ffr2 ffnum/ffn ffsum ffwhite/ffw ff_distance/ffd
// และฝั่ง bb... ทุกตัวที่คู่กัน  (ffblack/ffb ไม่ใช้ PID จึงไม่มี)
// ff(speed, kp, 'p');                      // กำหนด Kp อย่างเดียว (Kd ใช้ค่าจากตารางความเร็ว)
// ff(speed, kp, kd, 'p');                  // กำหนดทั้ง Kp และ Kd
// ffn(speed, kp, kd, 'p', n);              // ตัวที่มีพารามิเตอร์ต่อท้าย ใส่ Kp/Kd หลัง speed เหมือนกัน
// ffd(speed, kp, 'p', dist);
// fftimer(speed, kp, ms);       fftimer(speed, kp, kd, ms);       // ตามเวลา (ใส่ 'p' ท้ายสุดได้)
// fftimer(speed, kp, ms, 'p');  fftimer(speed, kp, kd, ms, 'p');
// ffcm(speed, kp, cm);          ffcm(speed, kp, kd, cm);          // ตามระยะทาง (cm เป็นจำนวนเต็ม)
// ffcm(speed, kp, cm, 'p');     ffcm(speed, kp, kd, cm, 'p');
// fft(speed, kp, [kd,] ms [, 'p']);                               // ชื่อย่อ ใช้แบบเดียวกับ fftimer
// * ใส่ Kp เป็นทศนิยม เช่น 0.02  (ใส่ค่าติดลบ = ใช้ค่าจากตาราง)


/* ==========================================================================
   [5] เดินตามเส้น ถอยหลัง  bb...   (ใช้เซนเซอร์หลัง เหมือน ff ทุกอย่าง)
   ========================================================================== */
// bb(speed, 'p');
// bbc(speed, 'p');      bbc2(speed, 'p');
// bbl(speed, 'p');      bbl2(speed, 'p');
// bbr(speed, 'p');      bbr2(speed, 'p');
// bbnum(speed, 'p', n); bbn(speed, 'p', n);
// bbsum(speed, 'p', sum);                      // ผลรวม B[0]..B[7] >= sum (0-8000)
// bbwhite(speed, 'p');  bbw(speed, 'p');
// bbblack(speed, 'p');  bbb(speed, 'p');
// bbblack(L, R, 'p');   bbb(L, R, 'p');
// bb_distance(speed, 'p', dist);   bbd(speed, 'p', dist);
// bbtimer(speed, ms);   bbtimer(speed, ms, 'p');
// bbt(speed, ms);       bbt(speed, ms, 'p');
// bbcm(speed, cm);      bbcm(speed, cm, 'p');
// lb(ms);
// bb(speed, kp, 'p');   bb(speed, kp, kd, 'p');   bbn(speed, kp, kd, 'p', n);   // กำหนด Kp / Kd เอง
// bbtimer(speed, kp, [kd,] ms [, 'p']);    bbcm(speed, kp, [kd,] cm [, 'p']);
// bbt(speed, kp, [kd,] ms [, 'p']);


/* ==========================================================================
   [6] วิ่งโค้ง  cl = เบี่ยงซ้าย, cr = เบี่ยงขวา
       (ตั้ง set_position_line_l / _r ใน Setting ก่อน)
   ========================================================================== */
//   เดินหน้า                         ถอยหลัง
// ffcl(speed, 'p');                 bbcl(speed, 'p');
// ffcr(speed, 'p');                 bbcr(speed, 'p');
// fftimercl(speed, ms [, 'p']);     bbtimercl(speed, ms [, 'p']);
// fftimercr(speed, ms [, 'p']);     bbtimercr(speed, ms [, 'p']);
// ffcmcl(speed, cm [, 'p']);        bbcmcl(speed, cm [, 'p']);
// ffcmcr(speed, cm [, 'p']);        bbcmcr(speed, cm [, 'p']);
//                                   ( [ ] = ใส่หรือไม่ใส่ก็ได้ )
//
// --- กำหนด Kp / Kd เอง (ใส่ต่อจาก speed) ---
// ffcl(speed, kp, [kd,] 'p');                  bbcl / ffcr / bbcr ใช้แบบเดียวกัน
// fftimercl(speed, kp, [kd,] ms [, 'p']);      ...timercr / bbtimercl / bbtimercr แบบเดียวกัน
// ffcmcl(speed, kp, [kd,] cm [, 'p']);         ...cmcr / bbcmcl / bbcmcr แบบเดียวกัน (cm เป็นจำนวนเต็ม)
//
// * วิ่งเกาะโค้งจนหันถึงทิศ / องศาด้วยไจโร (ffdircl / ffdircr / bbdircl / ffdegcl ...) ดู [13] ท้ายหมวด


/* ==========================================================================
   [7] เข้ากึ่งกลาง / ออกตัว / เข้าเส้นชัย
   ========================================================================== */
// ToCenter();                                  // เดินหน้าเข้ากลาง (เซนเซอร์คู่กลาง)
// BackCenter();                                  // ถอยหลังเข้ากลาง
// ToFront();    ToBack();                        // จัดตัวตรงเส้น หน้า / หลัง
// gostart(speed);   gostart(L, R);               // ออกจากกล่องเริ่ม
// goend(speed);     goend(L, R);                 // วิ่งเข้าเส้นชัย
// setf(n);   setb(n);   setfc(n);   setbc(n);    // จัดตรงเส้น n รอบ (= balancef / b / fc / bc)
// set_f(n);  set_b(n);  set_fc(n);  set_bc(n);
// TrackSelectF(speed, 'p');   TrackSelectB(speed, 'p');   // สั่ง select ตรง ๆ


/* ==========================================================================
   [8] หมุน / เลี้ยว ด้วยเซนเซอร์เส้น  ([speed] ใส่หรือไม่ใส่ก็ได้)
   ========================================================================== */
// spinl([speed]);      spinr([speed]);       // หมุนอยู่กับที่ไปเส้นถัดไป (เซนเซอร์หน้า)
// spinl2([speed]);     spinr2([speed]);
// spinlb([speed]);     spinrb([speed]);      // ใช้เซนเซอร์หลัง
// spinl2b([speed]);    spinr2b([speed]);
// TurnLeft();          TurnRight();          // เลี้ยวโค้ง เซนเซอร์หน้า
// TurnLeftB();        TurnRightB();        // เลี้ยวโค้ง เซนเซอร์หลัง
// TurnLeftBackF();     TurnRightBackF();     // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หน้า
// TurnLeftBackB();     TurnRightBackB();     // เลี้ยวล้อเดียวถอยหลัง เช็คเซนเซอร์หลัง


/* ==========================================================================
   [9] Gyro พื้นฐาน / ล็อกมุม
   ========================================================================== */
// resetAngles();       // ตั้งมุมปัจจุบัน = 0 (เรียกตอนหุ่นหยุดนิ่ง)
// SetRobotAngle();     // จำมุมปัจจุบันเป็นมุมเป้าหมายวิ่งตรง
// gyroZ();             // อ่านมุม -180..180 (ซ้าย = ลบ, ขวา = บวก)
//
// SetG(ms);    setg(ms);       // ล็อกมุม current_degree (หมุนอยู่กับที่) ตามเวลา แล้ว MotorStop()
// SetGB(ms);   setgb(ms);      // ล็อกมุม current_degree (ค่า kp/kd ถอยหลัง) แล้ว MotorStop()
// SetFG(ms);   setfg(ms);      // จำมุมสด ณ ตอนเรียก แล้วล็อกนิ่งอยู่กับที่ → MotorStop() + วิ่งตรงทิศนั้นต่อ
// SetBG(ms);   setbg(ms);      // เหมือน SetFG (ค่า kp/kd ถอยหลัง)
//
// --- เบรกหยุดตรงด้วยไจโร (Gyro Anti-Skid Brake) ใช้ต่อท้ายการวิ่งเร็ว ---
// SetFG(brakeSpd, brakeMs [, holdMs]);  setfg(...);  // หลังเดินหน้า: ถอยล้อต้าน brakeSpd นาน brakeMs + PD กันหัวปัด
//                                                      แล้วล็อกมุมนิ่ง holdMs (ค่าเริ่มต้น 60) → MotorStop()
// SetBG(brakeSpd, brakeMs [, holdMs]);  setbg(...);  // หลังถอยหลัง: ดันล้อเดินหน้าต้านแทน
//     fftimerg(80, 600);  SetFG(40, 40);            // วิ่งเร็ว 80 แล้วเบรกตรงหยุดเลย
// set_hold_deadband(0.5);      // คลาดไม่เกินกี่องศา = ถือว่าตรง ไม่สั่งมอเตอร์ (กันมอเตอร์ครางสั่น)
// SetHoldAngle();              // จำมุมปัจจุบันให้ HoldAngle...
// HoldAngle();   HoldAngleB();   HoldAngleF();   HoldAngleBF();    // ใช้ใน loop เพื่อค้างมุม


/* ==========================================================================
   [10] Gyro หมุน / เลี้ยว  (มุม: ลบ = ซ้าย, บวก = ขวา | [speed] ใส่หรือไม่ใส่ก็ได้)
   ========================================================================== */
// --- หมุน/เลี้ยวตามมุม ---
// spindegree([speed,] -90);          // หมุนอยู่กับที่
// turndegree([speed,] 90);           // เลี้ยวล้อเดียว เดินหน้า
// turndegreeb([speed,] 90);          // เลี้ยวล้อเดียว ถอยหลัง
// turndegree_none([speed,] 90);      // เหมือน turndegree แต่ไม่เบรก (ต่อคำสั่งถัดไปลื่น)
// turndegreeb_none([speed,] 90);
//
// --- วิ่งโค้งกำหนดล้อเอง (Ratio-Preserving Arc) ---
// rotatedegree(L, R, deg);           // ตีโค้งด้วยล้อซ้าย L ล้อขวา R จนหมุนครบ deg แล้วเบรกหยุด + ล็อกมุม
// rotatedegree(L, R, deg, kp, kd);   // กำหนดค่าชะลอเฉพาะครั้งนี้ (ความหมายเดียวกับ SetGyroRotate)
// rotatedegree_none(L, R, deg);      // ไม่ชะลอ ไม่เบรก ถึงมุมแล้วล้อยังวิ่ง (L, R) ต่อ → ต่อโค้งตัว S / วิ่งตรงต่อ
// * ทิศเลี้ยวดูจากล้ออย่างเดียว: L > R = เลี้ยวขวา (มุม +) | L < R = เลี้ยวซ้าย (มุม -)
//   ใส่ deg บวกหรือลบก็ได้ ระบบใช้ |deg| แล้วใส่เครื่องหมายตามล้อให้เอง (กันค้างวนไม่จบ) | deg ได้ถึง ~355
// * L == R (วิ่งตรง ไม่เลี้ยว) → บี๊บเตือน แล้วข้ามคำสั่ง
// * โค้งกว้างมาก (L, R ต่างกันน้อย เช่น 60, 50 ไป 180°) อาจเกิน safety timeout 6 วินาที → หยุด + บี๊บกลางทาง
//   ให้ขยายเวลาด้วย set_gyro_timeout(200, 10000);
// * ไกลจากเป้า: ล้อวิ่ง L, R ตรงตามสั่ง → รัศมีโค้งตรง | ใกล้เป้า: ลด L, R ด้วยตัวคูณเดียวกัน (โค้งเดิม)
//     rotatedegree(60, 20, 90);       // โค้งเดินหน้าเลี้ยวขวา 90° (ล้อซ้ายนอกโค้ง)
//     rotatedegree(20, 60, 90);       // โค้งเดินหน้าเลี้ยวซ้าย 90° (มุมจริง -90)
//     rotatedegree(-60, -20, 90);     // โค้งถอยหลัง ล้อซ้ายถอยเร็วกว่า → หัวหุ่นหันซ้าย 90° (มุม -90)
//     rotatedegree(-20, -60, 90);     // โค้งถอยหลัง ล้อขวาถอยเร็วกว่า → หัวหุ่นหันขวา 90° (มุม +90)
//     rotatedegree(50, -25, 180);     // หมุนเยื้องศูนย์ (จุดหมุนค่อนไปทางล้อขวา) ขวา 180°
//     rotatedegree_none(60, 30, 45);  rotatedegree_none(30, 60, 45);  ffcmg(60, 20);   // ตัว S แล้ววิ่งตรงต่อ
//
// --- ชื่อย่อ (ใส่มุมเป็นบวกเสมอ) ---
//   ซ้าย                ขวา                 ย่อ
// spinlg([speed,] 90);  spinrg([speed,] 90);  slg / srg      // หมุนอยู่กับที่
// turnlg([speed,] 90);  turnrg([speed,] 90);  tlg / trg      // เลี้ยวเดินหน้า
// turnlbg([speed,] 90); turnrbg([speed,] 90); tlbg / trbg    // เลี้ยวถอยหลัง
//
// --- เลี้ยว 2 จังหวะต่อเนื่อง (ไม่หยุดกลางทาง) ---
// tlrg([speed,] 90 [, 45]);    trlg([speed,] 90 [, 45]);     // ซ้ายแล้วขวา / ขวาแล้วซ้าย
// tlrbg([speed,] 90 [, 45]);   trlbg([speed,] 90 [, 45]);    // แบบถอยหลัง


/* ==========================================================================
   [11] Gyro วิ่งตรง  ( [ ] = ใส่หรือไม่ใส่ก็ได้ )
   ========================================================================== */
//   เดินหน้า                              ถอยหลัง
// fftimerg(speed, ms [, 'p']);           bbtimerg(speed, ms [, 'p']);           // ตามเวลา
// fftg(speed, ms, 'p');                  bbtg(speed, ms, 'p');                  // ชื่อย่อ
// ffcmg(speed, cm [, 'p']);              bbcmg(speed, cm [, 'p']);              // ตามระยะ (เร่ง/ผ่อน)
// ffcmgs(speed, cm [, 'p']);             bbcmgs(speed, cm [, 'p']);             // ตามระยะ (ความเร็วคงที่)
// ffbg(speed, 'p');                      bbbg(speed, 'p');                      // วิ่งจนเจอเส้น
//
// * ระยะทางคำนวณจาก ความเร็ว × เวลา × speed_scale (ไม่มี encoder) → ต้องคาลิเบรต set_speed_scale_*
//   ชดเชยแรงดันแบตอัตโนมัติด้วย set_battery_comp() ดู [14]
// ffdg(speed, 'p', dist);                bbdg(speed, 'p', dist);                // จนเซนเซอร์วัดระยะถึง dist (ค่า analogRead)
// ffdgs(speed, 'p', dist);               bbdgs(speed, 'p', dist);               // แบบค่อย ๆ ชะลอ
// ToCenterG();                           BackCenterG();                         // เข้ากลางด้วย gyro
// ToFrontG();                            ToBackG();                             // วิ่งจนเจอเส้น หน้า / หลัง
// TrackSelectG(speed, 'p');              TrackSelectGB(speed, 'p');             // สั่ง select ตรง ๆ
// RunG(L, R);                            RunGB(L, R);                           // 1 รอบ PID ใช้ใน loop เอง
// * ทุกคำสั่งในหมวดนี้ใส่ทิศต่อท้ายได้ เพื่อล็อกทิศจริงแทนมุมปัจจุบัน ดู [12]


/* ==========================================================================
   [12] Gyro ตามทิศ (Absolute Heading)  ( [ ] = ใส่หรือไม่ใส่ก็ได้ )
       ทิศอ้างอิงจากตอนเรียก resetAngles() / resetAnglesDir()
         0 = หน้าเริ่มต้น,  +90 = ขวา,  -90 = ซ้าย,  180 = หลัง
       ใส่ได้ทั้ง -180..180 และ 0..360 (270 = -90) หุ่นหมุนไปด้านที่สั้นกว่าเอง
       คำสั่งวิ่ง ff... / bb...: dir = ทิศที่หุ่น "เคลื่อนที่มุ่งหน้าไป" เหมือนกันทั้งหมด
         (0 = หน้า/เหนือ, 90 = ขวา, 180 = หลัง/ใต้, 270 หรือ -90 = ซ้าย)
         เดินหน้า: หัวหุ่นชี้ dir | ถอยหลัง: ท้ายหุ่นชี้ dir (หัวหุ่นชี้ dir + 180)
       คำสั่งหมุน / เลี้ยว spindir / turndir / turndirb: dir = ทิศที่ "หัวหุ่น" จะหันไป
   ========================================================================== */
// --- ค่าคงที่ทิศ (ใช้แทนตัวเลขได้) ---
// DIR_N  =    0     DIR_NE =   45     DIR_E  =   90     DIR_SE =  135
// DIR_S  =  180     DIR_SW = -135     DIR_W  =  -90     DIR_NW =  -45
//
// --- ตั้งทิศอ้างอิง ---
// resetAngles();                     // หุ่นหันทางไหนอยู่ = ทิศ 0 (DIR_N)
// resetAnglesDir(dir);               // วางหุ่นหันทิศ dir แล้วรีเซ็ต เช่น resetAnglesDir(DIR_E);
//                                    //   เรียกตอนหุ่นหยุดนิ่ง | คืน false + บี๊บ = gyro ไม่ส่งข้อมูล
// setHeading(dir);                   // ตั้งทิศ "หัวหุ่น" ที่ RunG / RunGB จะล็อก (ไม่หมุนตัว ใช้ก่อนวิ่ง)
// set_auto_align(20);                // คำสั่งวิ่งตามทิศ: หัวหุ่นคลาดเกิน 20° → spindir ตั้งลำก่อนวิ่ง (0 = ปิด)
//
// --- หมุน 2 ล้ออยู่กับที่ เข้าหาทิศ (Spin to Heading) ---
// spindir([speed,] dir);             sdg([speed,] dir);           // ไม่ใส่ speed = maxSpd ของ SetGyroSpin
//
// --- เลี้ยวล้อเดียว เข้าหาทิศ (Turn to Heading) ---
// turndir([speed,] dir);             tdg([speed,] dir);           // เดินหน้า (maxSpd ของ SetGyroTurn)
// turndirb([speed,] dir);            tdbg([speed,] dir);          // ถอยหลัง (maxSpd ของ SetGyroTurnB) จนหัวหุ่นหันทิศ dir
// turndir_none([speed,] dir);        turndirb_none([speed,] dir); // ไม่เบรก ไหลต่อคำสั่งถัดไปลื่น
//
// --- วิ่งโค้งกำหนดล้อเอง เข้าหาทิศ (Arc to Heading) — ลอจิกเดียวกับ rotatedegree ---
// rotatedir(L, R, dir);              // ตีโค้งสัดส่วน L:R จนหัวหุ่นหันทิศ dir แล้วเบรกหยุด + ล็อกมุม
// rotatedir_none(L, R, dir);         // ถึงทิศแล้วไหลต่อ ไม่ชะลอ ไม่เบรก
// rotatedir(L, R, 'p', dir);         // ถึงทิศแล้วทำ select ต่อ:
//     'n' = ไหลต่อ | 'f' = เบรกไจโร SetFG (โค้งถอยหลัง = SetBG) | อื่น ๆ = TrackSelectG (โค้งถอยหลัง = TrackSelectGB)
//     ความเร็วที่ส่งต่อ = ล้อที่เร็วกว่า | "โค้งถอยหลัง" = L + R < 0
// * เลี้ยวเข้าทิศทางสั้นที่สุดเสมอ: ถ้าล้อพาเลี้ยวผิดฝั่ง จะสลับ L ↔ R ให้ (โค้งกระจกเงา รัศมีเท่าเดิม)
//     resetAngles();
//     rotatedir(60, 20, DIR_E);          // โค้งเดินหน้าไปทิศ 90 แล้วหยุด
//     rotatedir(60, 20, DIR_N);          // กลับทิศ 0 → ล้อพาเลี้ยวขวาผิดฝั่ง → ใช้ (20, 60) ให้อัตโนมัติ
//     rotatedir(-60, -20, DIR_W);        // โค้งถอยหลังจนหัวหุ่นหันทิศ -90
//     rotatedir(50, -25, DIR_S);         // หมุนเยื้องศูนย์ไปทิศ 180
//     rotatedir_none(60, 30, DIR_NE);  rotatedir(60, 30, 'f', DIR_E);   // โค้ง 2 ช่วงต่อกัน แล้วเบรกไจโร
//     rotatedir(60, 20, 'r', DIR_E);     // ถึงทิศ 90 แล้วทำ 'r' ของ TrackSelectG
//
// --- วิ่งตรงล็อกทิศ (ใส่ dir เป็นตัวสุดท้ายเสมอ) ---
//   เดินหน้า (dir = ทิศที่วิ่งไป)            ถอยหลัง (dir = ทิศที่ถอยไป = ทิศท้ายหุ่น)
// ffcmg(speed, cm [, 'p'], dir);         bbcmg(speed, cm [, 'p'], dir);         // ตามระยะ (เร่ง/ผ่อน)
// ffcmgs(speed, cm [, 'p'], dir);        bbcmgs(speed, cm [, 'p'], dir);        // ตามระยะ (ความเร็วคงที่)
// fftg(speed, ms [, 'p'], dir);          bbtg(speed, ms [, 'p'], dir);          // ตามเวลา
// ffbg(speed, 'p', dir);                 bbbg(speed, 'p', dir);                 // วิ่งจนเจอเส้น แล้วทำ 'p'
// RunG(L, R, dir);                       RunGB(L, R, dir);                      // 1 รอบ PID ใช้ใน loop เอง
// * หัวหุ่นคลาดจากทิศที่ต้องล็อกเกิน 20° → หมุนตั้งลำ (spindir) ให้ก่อนอัตโนมัติ (ยกเว้น RunG / RunGB)
//     ffcmg(60, 50, 's', 0);   bbcmg(60, 50, 's', 180);   // เดินหน้า 50 cm แล้วถอยตรงกลับจุดเดิม (ไม่หมุนตัว)
//
// * พารามิเตอร์ตัวที่ 3: ตัวอักษรใน ' ' = select | ตัวเลข / DIR_.. = ทิศ
//     ffcmg(60, 40, 's');      // วิ่ง 40 cm แล้วหยุด (ล็อกมุมปัจจุบันแบบเดิม)
//     ffcmg(60, 40, DIR_E);    // วิ่ง 40 cm ล็อกทิศ 90
//     ffcmg(60, 40, 's', 90);  // วิ่ง 40 cm ล็อกทิศ 90 แล้วหยุด
//
// --- คอมโบ: หมุน / เลี้ยวตั้งลำเข้าหาทิศ แล้ววิ่งตามระยะต่อทันที ---
// spin_ffcmg(speed, cm [, 'p'], dir);    spin_bbcmg(speed, cm [, 'p'], dir);    // spindir   แล้ววิ่ง
// turn_ffcmg(speed, cm [, 'p'], dir);    turn_bbcmg(speed, cm [, 'p'], dir);    // turndir   แล้ววิ่ง
// turnb_ffcmg(speed, cm [, 'p'], dir);   turnb_bbcmg(speed, cm [, 'p'], dir);   // turndirb  แล้ววิ่ง
// * speed = ความเร็ววิ่ง | ความเร็วหมุน / เลี้ยว ใช้ maxSpd ของ SetGyroSpin / SetGyroTurn / SetGyroTurnB
// * ..._bbcmg: หมุน / เลี้ยวให้ "ท้ายหุ่น" ชี้ dir แล้วถอยไปทาง dir
//
// --- ทางแยก TrackSelectG / TrackSelectGB: ล็อกแกน 90° อัตโนมัติ (snap90) ---
// 'l' 'L' 'r' 'R' 'q' 'Q' 'e' 'E' จะปัดมุมหุ่นเข้าแกนที่ใกล้สุด (0, 90, -90, 180)
// แล้วหมุน / เลี้ยวไป "แกน ± 90" แบบทิศจริง → มุมไม่เพี้ยนสะสมทีละทางแยก
// set_track_snap90(true);            // เปิด (ค่าเริ่มต้น)
// set_track_snap90(false);           // ปิด = หมุน ±90 จากมุมปัจจุบันแบบเดิม (ใช้ตอนเข้าแยกแนวทแยง 45°)
//
// --- ตัวอย่าง (Example Mission) ---
// resetAngles();                     // วางหุ่นหันหน้าสนาม = ทิศ 0
// ffcmg(60, 50, DIR_N);              // วิ่งตรง 50 cm ล็อกทิศ 0
// spindir(DIR_E);                    // หมุนไปทิศ 90 (ขวา)
// ffbg(50, 'c', DIR_E);              // ล็อกทิศ 90 วิ่งจนเจอเส้น แล้วเข้ากลางหยุด
// ffcmg(50, 30, 'r', DIR_E);         // วิ่งต่อ 30 cm แล้วหมุนขวาที่แยก (snap90 → ทิศ 180)
// bbcmg(50, 20, DIR_N);              // ถอยหลังไปทางทิศ 0 อีก 20 cm (หัวหุ่นยังหันทิศ 180)
// spin_ffcmg(60, 40, 's', DIR_W);    // หมุนไปทิศ -90 แล้ววิ่ง 40 cm หยุด
// turndirb(DIR_N);                   // เลี้ยวล้อเดียวถอยหลัง จนหัวหุ่นหันทิศ 0


/* ==========================================================================
   [13] เดินตามเส้นจนหันถึงทิศ / องศา  ffdir / bbdir / ffdeg / bbdeg
       วิ่งตามเส้นด้วย PID แล้วใช้ gyro เช็คว่าเลี้ยวพ้นโค้งหรือยัง
       เหมาะกับโค้ง 90°, โค้งตัวยู 180°, โค้งตัว S
   ========================================================================== */
// --- ไม่ใส่ select: ถึงแล้วไหลต่อทันที ไม่เบรก ---
// ffdir(speed, dir [, tol]);              bbdir(speed, dir [, tol]);              // ตามเส้นจนถึงทิศจริง dir (ดู [12])
// ffdeg(speed, deg [, tol]);              bbdeg(speed, deg [, tol]);              // ตามเส้นจนเลี้ยวครบ deg องศา
//
// --- select ไว้หน้า dir: จบด้วยโหมดเส้น TrackSelectF / TrackSelectB (ตาราง [3] ช่อง ff / bb) ---
// ffdir(speed, 'p', dir [, tol]);         bbdir(speed, 'p', dir [, tol]);
// ffdeg(speed, 'p', deg [, tol]);         bbdeg(speed, 'p', deg [, tol]);
//
// --- จบด้วยโหมดไจโร TrackSelectG / TrackSelectGB (ตาราง [3] ช่อง *g / *gb) ---
// ffdirg(speed, 'p', dir [, tol]);        bbdirg(speed, 'p', dir [, tol]);
//
// --- แบบเดิม (select ไว้หลัง dir) ยังใช้ได้ ---
// ffdir(speed, dir, 'p' [, tol]);  bbdir(...);  ffdeg(speed, deg, 'p' [, tol]);  bbdeg(...);  // → TrackSelectG / GB
// ffdirf / bbdirb / ffdegf / bbdegb(speed, dir|deg, 'p' [, tol]);                          // → TrackSelectF / B
//
//   dir    ทิศที่หุ่นเคลื่อนที่ไป: ff... = ทิศหัวหุ่น | bb... = ทิศที่ถอยไป (ท้ายหุ่น) → หัวหุ่นชี้ dir + 180
//   deg    + = หัวหุ่นหมุนขวา, - = ซ้าย  (-180..180, โค้งตัวยู = 180 / -180) นับจากมุมตอนเริ่มคำสั่ง
//   'p'    'n' หรือ 0 = ไม่หยุด ไม่เบรก ไหลต่อคำสั่งถัดไป
//          'f'        = หยุดด้วยเบรกไจโร SetFG / SetBG (แรง / เวลาเบรกคิดจาก speed อัตโนมัติ)
//          ตัวอื่น     = TrackSelect ของคำสั่งนั้น ('s','l','r','p','c','q','e',...)
//   tol    ถือว่าถึงเมื่อคลาดไม่เกิน tol องศา (ค่าเริ่มต้น 4, ใส่ 0 = ค่าเริ่มต้น)
//          ไม่มี select แล้วใส่ตัวเลขเป็นตัวที่ 3 = tol  เช่น ffdir(50, 90, 0); = ไม่หยุด
// set_line_dir_brake(spd, ms [, hold]);  // ตั้งแรง / เวลาเบรกของ 'f' เอง (0, 0 = คิดอัตโนมัติจาก speed)
// * จบคำสั่งแล้วล็อกทิศเป้าหมายไว้ → ต่อด้วย ffcmg / RunG วิ่งตรงทิศนั้นได้ทันทีไม่กระชาก
//
// ffdir(50, DIR_E);   ffcmg(60, 30, 's');      // ตามโค้งจนหันทิศ 90 แล้ววิ่งตรงต่อ 30 cm หยุด
// ffdir(50, 'r', DIR_E);                        // ตามโค้งจนหันทิศ 90 แล้วเลี้ยวขวาแบบ TrackSelectF
// ffdirg(50, 'r', DIR_E);                       // ตามโค้งจนหันทิศ 90 แล้วหมุนขวาแบบไจโร (snap90)
// ffdir(60, 'f', DIR_S);                        // ตามโค้งจนหันทิศ 180 แล้วเบรกไจโรหยุดตรง
// bbdir(50, 'f', DIR_W);                        // ถอยตามเส้นจนท้ายหุ่นชี้ทิศ -90 แล้วเบรกไจโร
// ffdeg(50, 's', 180);                          // ตามโค้งตัวยูจนกลับหัว 180° แล้วหยุดแบบ TrackSelectF
//
// --- วิ่งเกาะเส้นโค้ง cl / cr จนหันถึงทิศ / องศา (ลอจิกเดียวกับ ffcmcl / ffcmcr [6] แต่หยุดด้วยไจโร) ---
//     cl = เบี่ยงเกาะเส้นไปทางซ้าย (set_position_line_l) | cr = เบี่ยงไปทางขวา (set_position_line_r)
//   เดินหน้า                                      ถอยหลัง (dir = ทิศที่ถอยไป)
// ffdircl(speed, dir [, tol]);                    bbdircl(speed, dir [, tol]);           // ถึงทิศแล้วไหลต่อ ไม่เบรก
// ffdircr(speed, dir [, tol]);                    bbdircr(speed, dir [, tol]);
// ffdircl(speed, 'p', dir [, tol]);               bbdircl(speed, 'p', dir [, tol]);      // → TrackSelectF / B
// ffdircr(speed, 'p', dir [, tol]);               bbdircr(speed, 'p', dir [, tol]);
// ffdirclg(speed, 'p', dir [, tol]);              bbdirclg(speed, 'p', dir [, tol]);     // → TrackSelectG / GB
// ffdircrg(speed, 'p', dir [, tol]);              bbdircrg(speed, 'p', dir [, tol]);
// ffdegcl / ffdegcr(speed, [ 'p', ] deg [, tol]);  bbdegcl / bbdegcr(speed, [ 'p', ] deg [, tol]);  // องศาสัมพัทธ์ → TrackSelectF / B
//   'p'  'n' / 0 = ไม่หยุดมอเตอร์ ไหลเข้าคำสั่งถัดไป | 'f' = เบรกไจโรหยุดตรง | ตัวอื่น = TrackSelect
//
// --- กำหนด Kp / Kd เอง (ใส่ต่อจาก speed เหมือน ffcmcl(speed, kp, [kd,] cm, 'p')) ---
// ffdircl(speed, kp, [kd,] 'p', dir [, tol]);     ใช้ได้กับทุกตัวในชุดนี้ (ffdircr / bbdircl / ffdirclg / ffdegcl ...)
//   * ต้องใส่ 'p' เสมอ ถ้าไม่ต้องการให้ใส่ 'n'  | kp / kd ติดลบ = ใช้ค่าจากตารางความเร็ว
//     ffdircl(50, 0.05, 'n', DIR_W);             // kp 0.05 เกาะโค้งซ้ายจนหันทิศ -90 แล้วไหลต่อ
//     ffdegcr(50, 0.05, 0.3, 'f', 90);           // kp 0.05 kd 0.3 เกาะโค้งขวาครบ 90° แล้วเบรกไจโร
//
// --- ตัวอย่าง: โค้งหักศอก 90° ---
// resetAngles();
// ffcm(60, 40);                       // วิ่งตามเส้นตรงหันทิศ 0
// ffdircr(50, DIR_E);                 // เกาะโค้งขวาจนหันทิศ 90 แล้วไหลต่อ
// ffcmg(60, 30, 's', DIR_E);          // วิ่งตรงล็อกทิศ 90 อีก 30 cm แล้วหยุด
//
// --- ตัวอย่าง: โค้งตัว S ต่อเนื่อง (ซ้าย 90 → ขวา 90) ไม่สะดุด ---
// ffdegcl(50, 'n', -90);              // เกาะโค้งซ้ายจนเลี้ยวครบ -90° ไหลต่อทันที
// ffdegcr(50, 90);                    // เกาะโค้งขวาจนเลี้ยวกลับ +90° ไหลต่อ
// ffdir(50, 'p', DIR_N);              // ตามเส้นจนหันทิศ 0 แล้วข้ามแยก 'p' แบบ TrackSelectF
//
// --- ตัวอย่าง: เกาะโค้งตัวยูแล้วเบรกไจโรหยุดตรง / ถอยเข้าโค้ง ---
// ffdircl(60, 'f', DIR_S);            // เกาะโค้งซ้ายจนหันทิศ 180 แล้วเบรกไจโร (SetFG)
// bbdircr(50, 's', DIR_E);            // ถอยเกาะโค้งขวาจนท้ายหุ่นชี้ทิศ 90 แล้วหยุดแบบ TrackSelectB


/* ==========================================================================
   [14] มอเตอร์ / แบตเตอรี่
   ========================================================================== */
// Motor(L, R);                 // สั่งล้อซ้าย/ขวาค้างไว้
// Move(L, R, ms);              // สั่งล้อตามเวลา
// MotorStop();   MotorStop(ms);
// MotorShot();   MotorShot(ms, power);       // เบรกช็อต (power 0-100)
// fd(speed, ms);   bk(speed, ms);            // เดินหน้า / ถอยหลัง
// sl(speed, ms);   sr(speed, ms);            // หมุนซ้าย / ขวา
// tl(speed, ms);   tr(speed, ms);            // เลี้ยวซ้าย / ขวา (ล้อเดียว)
// getBatteryVoltage();                       // อ่านแรงดันแบต (V)
//
// --- ชดเชยแรงดันแบต สำหรับวิ่งตามระยะ (ffcm, bbcm, ffcmg, bbcmg, ffcmgs, bbcmgs) ---
// set_battery_comp(ref_volt [, gain, enable]);  // ค่าเริ่มต้น (11.8, 1.0, true)
//     ref_volt = แรงดันแบตตอนจูน speed_scale_* | gain = ความแรงชดเชย (0.8-1.2) | enable = false ปิด
//     ชดเชยเฉพาะส่วนที่ PWM (bat_control) ชดเชยไม่ได้ (scale ติดขอบ 0.95-1.05 / PWM ติดเพดาน) ไม่คูณซ้ำ
// print_battery_comp([speed]);               // พิมพ์ แรงดัน / ref / pwm scale / ตัวคูณระยะ ที่ความเร็ว speed
// batt_distance_factor(speed);               // ตัวคูณระยะตอนนี้ (>1 = แบตแรงกว่าตอนจูน หยุดเร็วขึ้น)


/* ==========================================================================
   [15] แขน / เซอร์โว
   ========================================================================== */
// Servo(pin, angle);                         // pin: 0, 1, 10, 28
// Servo(a, b, c);              Servo(a, b, c, speed);
// armupdown(angle [, speed]);                // แขนขึ้น-ลง
// arm_left_right(l, r [, speed]);            // มือซ้าย-ขวา
// S0_trim(x);   S1_trim(x);   S10_trim(x);   S28_trim(x);     // ชดเชยองศา
// S0_pulse(min, max);  S1_pulse(..);  S10_pulse(..);  S28_pulse(..);  // ช่วงพัลส์ µs (ค่าเริ่มต้น 500, 2500)
// SetServoPulse(pin, min, max);              // แบบระบุขาเซอร์โว
// SerialServoControl();                      // ปรับผ่าน Serial พิมพ์ "10 90"
//
// --- ท่าสำเร็จรูปใน Servo.ino (ใส่ speed ได้ เช่น arm_close(2);) ---
// arm_ready();       arm_up();          arm_up45();         arm_down();
// arm_open();        arm_close();       arm_open_l();       arm_open_r();
// arm_open_down();   arm_down_open();   arm_open_up();      arm_up_open();
// arm_down_close();  arm_close_down();  arm_up_close();     arm_close_up();
// arm_big();         arm_big_box();     arm_big_box_up();   arm_behihd();
//
// --- เซอร์โวแบบไม่บล็อก ขยับนุ่มเบื้องหลัง (ไม่หยุดมอเตอร์ ไม่ delay → สั่งแล้ววิ่งตามเส้นต่อได้ทันที) ---
// speed = องศา/วินาที (60-90 นุ่มมาก, 120-180 ปกติ) | ดูตัวอย่าง examples/ServoSmooth_LineFollow
// ServoMove(pin, angle [, speed]);                 // ตัวเดียว (ค่าเริ่มต้น 120 องศา/วินาที)
// armupdown_smooth(angle, speed);                  // แขนขึ้น-ลง
// arm_left_right_smooth(l, r, speed);              // มือซ้าย-ขวา ถึงเป้าพร้อมกัน
// Servo_smooth(a, b, c, speed);                    // 3 ตัวพร้อมกัน ลำดับเดียวกับ Servo(a, b, c)
// ServoBusy();  ServoBusy(pin);                    // ยังขยับอยู่ไหม
// ServoWait([timeoutMs]);                          // รอจนถึงเป้า (ไม่แตะมอเตอร์)
// ServoHold(pin);   ServoAngle(pin);               // หยุดค้างที่มุมปัจจุบัน / อ่านมุมปัจจุบัน
// SetServoAccel(dps2);                             // ทางลาดเร่ง/หน่วง (ค่าเริ่มต้น 400, 0 = ไม่มี)
// SetServoAutoSpeed(speed);                        // ให้ Servo()/armupdown()/arm_left_right() เดิมขยับนุ่มเอง (0 = ปิด)


/* ==========================================================================
   [16] เซนเซอร์ / Calibrate
   ========================================================================== */
// --- ตัวแปร ---
// F[0-7]  B[0-7]  C[0-1]           // ค่าเซนเซอร์ หน้า / หลัง / กลาง (หลัง calibrate 0-1000)
// Ref  RefC                        // threshold หน้า-หลัง / กลาง
//
// --- อ่านค่า ---
// ReadCalibrateF();   ReadCalibrateB();   ReadCalibrateC();     // อ่านแบบ calibrate แล้ว
// ReadF();            ReadB();            ReadC();              // อ่านค่าดิบ
// ReadSensor();       ReadSensorRaw();
//
// --- Calibrate ---
// calibrateA();        calibrateB();        calibrateC();
// saveCalibA_LOCAL();  saveCalibB_LOCAL();  saveCalibC_LOCAL();
// loadCalibration();   loadCalibration_LOCAL();   printCalibration_LOCAL();


/* ==========================================================================
   [17] Debug (Serial Monitor) — ค้าง while(1) ใช้ใน Setting() เท่านั้น
   ========================================================================== */
// Serial_FrontSensor();            Serial_BackSensor();             // ค่าดิบ
// Serial_CenterSensor();           Serial_AllSensor();
// SerialCalibrate_FrontSensor();   SerialCalibrate_BackSensor();    // ค่า calibrate
// SerialCalibrate_CenterSensor();  SerialCalibrate_AllSensor();
// SerialPositionF();   SerialPositionB();   SerialPositionFB();     // ตำแหน่งเส้น
// SerialDistance();                                                 // เซนเซอร์วัดระยะ


/* ==========================================================================
   [18] เสียง / ไฟ
   ========================================================================== */
// Beep(ms);          Beep2(freq, ms);     beep(freq, ms);
// BeepScanner();                          // ปี๊บสั้นแบบเครื่องสแกนบาร์โค้ด
// BZon();            BZoff();             // เปิด / ปิด buzzer ค้าง
// blink(n);                               // กระพริบไฟ n ครั้ง
