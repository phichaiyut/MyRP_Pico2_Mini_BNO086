/* =====================================================
   📘 คู่มือคำสั่ง MyRP_Pico2_mini_BNO086  (comment ล้วน ไม่มีผลต่อโค้ด)
   กด Ctrl+F แล้วพิมพ์ [1] [2] ... เพื่อกระโดดไปหมวด

   [1] ตั้งค่า (Setting.ino)          [7] Gyro หมุน/เลี้ยว
   [2] คำสั่งทางแยก 'select'          [8] Gyro วิ่งตรง
   [3] เดินตามเส้น หน้า ff            [9] วิ่งโค้ง cl/cr
   [4] เดินตามเส้น หลัง bb            [10] มอเตอร์
   [5] เข้ากึ่งกลาง / จัดตรงเส้น      [11] แขน / เซอร์โว
   [6] หมุน/เลี้ยว ด้วยเซนเซอร์เส้น   [12] เซนเซอร์ / Debug / เสียง

   setup():  RobotSetup();  Setting();  sw();  Mission();
   ===================================================== */


/* ===== [1] ตั้งค่า (Setting.ino) ===== */
// set_Freq("DC_Motors");            set_Freq("coreless");
// clampSensorValueF(min, max);      clampSensorValueB(min, max);      clampSensorValueC(min, max);
// RefLineValue(x);                  RefCenterLineValue(x);            // ค่าเจอเส้น 0-1000
// TrackLineColor(0);                // 0=พื้นขาวเส้นดำ 1=พื้นดำเส้นขาว
// Dottedline(0);                    // 1=มีเส้นประ
// set_line_center(0/1);             SetToCenterSpeed(speed);
// set_brake_fc(ff, fc);             set_brake_bc(bf, bc);             // เวลาเบรก (ms)
// SetTurnSpeed(speed);
// TurnSpeedLeft(l, r, delay);       TurnSpeedRight(l, r, delay);
// TurnBackSpeedLeft(l, r, delay);   TurnBackSpeedRight(l, r, delay);
// TurnSpeedLeftBackF(l, r, delay);  TurnSpeedRightBackF(l, r, delay);
// TurnSpeedLeftBackB(l, r, delay);  TurnSpeedRightBackB(l, r, delay);
// ModeSpdPID(mode, max, min);       SetPIDDeadBand(db);
// ModeSpdGyro(mode, max, min);      ModeSpdGyro(modeF, modeB, max, min);
//   mode 0 = 0..max | 1 = min..max | 2 = -Speed..Speed | 3 = ..max | 4 = 0..Speed
// set_position_line(3500);          set_position_line_l(pos);         set_position_line_r(pos);
// set_slow_kp_kd(kpf, kdf, kpb, kdb);
// SetAnalogDistance(A0);
// --- ตารางความเร็ว (ch = SPD_10 ... SPD_100) ---
// setBalanceSpeed(ch, l, r);        setBalanceBackSpeed(ch, l, r);
// Set_KP_KD(ch, kp, kd);            Set_KP_KD_Back(ch, kp, kd);


/* ===== [2] คำสั่งทางแยก 'select' (ตัวสุดท้ายของ ff / bb / *g) =====
 *
 *  ff / bb (เส้น PID)                     | *g / *gb (Gyro)
 *  's' หยุด                                | 's' 'S' หยุด
 *  'S' วิ่งถึงขอบเส้นแล้วหยุด               |
 *  'p' 'P' วิ่งผ่าน                         | 'p' 'P' วิ่งผ่าน
 *  'c' 'C' เข้ากลางแล้วหยุด                  | 'c' 'C' เข้ากลางแล้วหยุด
 *  'l' 'L' เข้ากลาง + หมุนซ้าย               | 'l' เข้ากลาง + หมุนซ้าย   'L' หมุนซ้ายทันที
 *  'r' 'R' เข้ากลาง + หมุนขวา               | 'r' เข้ากลาง + หมุนขวา   'R' หมุนขวาทันที
 *  'q' 'Q' เลี้ยวโค้งซ้าย (bb = ขวา)        | 'q' 'Q' เลี้ยวโค้งซ้าย (gb = ขวา)
 *  'e' 'E' เลี้ยวโค้งขวา (bb = ซ้าย)        | 'e' 'E' เลี้ยวโค้งขวา (gb = ซ้าย)
 *  'a' 'A' หมุนซ้ายด้วยเซนเซอร์หลัง          |
 *  'd' 'D' หมุนขวาด้วยเซนเซอร์หลัง          |
 *  'b' 'B' เข้ากลางแล้ววิ่งต่อถึงเส้นอีกฝั่ง   | 'b' 'B' เหมือนกัน
 *  'g' 'G' ปรับตรงด้วย gyro                 | ตัวอื่น = SetG(100)
 *  ตัวพิมพ์ใหญ่ (P C Q E G) = ToFront()/ToBack() ก่อน
 */


/* ===== [3] เดินตามเส้น หน้า ff ===== */
// ff(speed, 'p');
// ffc(speed, 'p');      ffc2(speed, 'p');
// ffl(speed, 'p');      ffl2(speed, 'p');     ffl0(speed, 'p');
// ffr(speed, 'p');      ffr2(speed, 'p');     ffr7(speed, 'p');
// ffnum(speed, 'p', 0-7);     ffn(speed, 'p', 0-7);
// ffwhite(speed, 'p');        ffw(speed, 'p');                 // จนไม่เจอเส้น
// ffblack(speed, 'p');        ffb(speed, 'p');                 // จนเจอเส้น (ไม่ PID)
// ffblack(L, R, 'p');         ffb(L, R, 'p');
// ff_distance(speed, 'p', dist);   ffd(speed, 'p', dist);     // เซนเซอร์วัดระยะ
// ff_distances(speed, 'p', dist);  ffds(speed, 'p', dist);    // ค่อยๆ เบรก
// fftimer(speed, ms);         fftimer(speed, ms, 'p');    fft(speed, ms, 'p');
// ffcm(speed, cm);            ffcm(speed, cm, 'p');
// lf(ms);                     // PID อยู่กับที่ จัดตัวตรงเส้น


/* ===== [4] เดินตามเส้น หลัง bb ===== */
// bb(speed, 'p');
// bbc(speed, 'p');      bbc2(speed, 'p');
// bbl(speed, 'p');      bbl2(speed, 'p');     bbl0(speed, 'p');
// bbr(speed, 'p');      bbr2(speed, 'p');     bbr7(speed, 'p');
// bbnum(speed, 'p', 0-7);     bbn(speed, 'p', 0-7);
// bbwhite(speed, 'p');        bbw(speed, 'p');
// bbblack(speed, 'p');        bbb(speed, 'p');
// bbblack(L, R, 'p');         bbb(L, R, 'p');
// bb_distance(speed, 'p', dist);   bbd(speed, 'p', dist);
// bbtimer(speed, ms);         bbtimer(speed, ms, 'p');    bbt(speed, ms, 'p');
// bbcm(speed, cm);            bbcm(speed, cm, 'p');
// lb(ms);


/* ===== [5] เข้ากึ่งกลาง / จัดตรงเส้น ===== */
// ToCenter();   ToCenterL();   ToCenterR();   ToFront();       // หน้า
// BackCenter(); ToBack();                                      // หลัง
// gostart(speed);   gostart(L, R);        // ออกจากจุดเริ่ม
// goend(speed);     goend(L, R);          // เข้าเส้นชัย
// setf(n);  setb(n);  setfc(n);  setbc(n);                       // = balancef/b/fc/bc(n)
// set_f(n); set_b(n); set_fc(n); set_bc(n);
// TrackSelectF(speed, 'p');   TrackSelectB(speed, 'p');         // สั่ง select ตรงๆ


/* ===== [6] หมุน/เลี้ยว ด้วยเซนเซอร์เส้น ===== */
// spinl();    spinr();     spinl(speed);    spinr(speed);        // หมุน 90°
// spinl2();   spinr2();    spinl2(speed);   spinr2(speed);
// spinl_B();  spinr_B();   spinl_B(speed);  spinr_B(speed);      // เซนเซอร์หลัง
// spinl2_B(); spinr2_B();  spinl2_B(speed); spinr2_B(speed);
// TurnLeft();        TurnRight();                                // เลี้ยวโค้ง
// TurnLeft_B();      TurnRight_B();
// TurnLeftBackF();   TurnRightBackF();
// TurnLeftBackB();   TurnRightBackB();


/* ===== [7] Gyro หมุน/เลี้ยว (ลบ = ซ้าย, บวก = ขวา) ===== */
// resetAngles();   SetRobotAngle();   gyroZ();   SetHoldAngle();
// SetG(ms);  setg(ms);    SetGB(ms);  setgb(ms);    SetFG(ms);  setfg(ms);   // ล็อกมุม
// HoldAngle();     HoldAngleB();      HoldAngleF();
//
// spindegree(-90);        spindegree(speed, -90);      // หมุนอยู่กับที่
// turndegree(90);         turndegree(speed, 90);       // เลี้ยวล้อเดียว
// turndegreeb(90);        turndegreeb(speed, 90);      // เลี้ยวถอยหลัง
// turndegree_none(90);    turndegree_none(speed, 90);  // ไม่เบรก
// turndegreeb_none(90);   turndegreeb_none(speed, 90);
// rotatedegree(L, R, 90);      rotatedegree(L, R, 90, kp, kd);   // โค้งกว้าง
//
// --- หันไปทิศ (0 / 90 / 180 / 270) ---
// spindirection(90);           spindirection(speed, 90);
// turndirection(90);           turndirection(speed, 90);
// turndirectionb(90);          turndirectionb(speed, 90);
// turndirection_none(90);      turndirection_none(speed, 90);
// turndirectionb_none(90);     turndirectionb_none(speed, 90);
// relativeToDirection(90);     SetDirectionG(90);
//
// --- ชื่อย่อ (ใส่มุมบวก, มี/ไม่มี speed) ---
// spinlg(90);  spinrg(90);   slg(90);  srg(90);      // หมุน
// turnlg(90);  turnrg(90);   tlg(90);  trg(90);      // เลี้ยว
// turnlbg(90); turnrbg(90);  tlbg(90); trbg(90);     // เลี้ยวถอย
// spinlg(speed, 90);  ...
//
// --- เลี้ยว 2 จังหวะ ---
// tlrg(90);   trlg(90);   tlrg(speed, 90);   trlg(speed, 90);   tlrg(speed, 90, 45);   trlg(speed, 90, 45);
// tlrbg(90);  trlbg(90);  tlrbg(speed, 90);  trlbg(speed, 90);  tlrbg(speed, 90, 45);  trlbg(speed, 90, 45);


/* ===== [8] Gyro วิ่งตรง (direction ใส่ท้ายได้ = ล็อกทิศ) ===== */
// fftimerg(speed, ms);          fftimerg(speed, ms, 'p');     fftimerg(speed, ms, 'p', dir);
// bbtimerg(speed, ms);          bbtimerg(speed, ms, 'p');     bbtimerg(speed, ms, 'p', dir);
// fftg(speed, ms, 'p');         bbtg(speed, ms, 'p');         // + dir ได้
// ffcmg(speed, cm);             ffcmg(speed, cm, 'p');        ffcmg(speed, cm, 'p', dir);    // เร่ง/ผ่อน
// bbcmg(speed, cm);             bbcmg(speed, cm, 'p');        bbcmg(speed, cm, 'p', dir);
// ffcmgs(speed, cm);            ffcmgs(speed, cm, 'p');       ffcmgs(speed, cm, 'p', dir);   // ความเร็วคงที่
// bbcmgs(speed, cm);            bbcmgs(speed, cm, 'p');       bbcmgs(speed, cm, 'p', dir);
// ffbg(speed, 'p');             bbbg(speed, 'p');             // + dir ได้  วิ่งจนเจอเส้น
// ffdg(speed, 'p', dist);       bbdg(speed, 'p', dist);       // เซนเซอร์วัดระยะ
// ffdgs(speed, 'p', dist);      bbdgs(speed, 'p', dist);
// ToCenterLG();  ToCenterRG();  ToCenterLRG();  BackCenterG();
// ToFrontG();    ToBackG();
// TrackSelectG(speed, 'p');     TrackSelectGB(speed, 'p');
// RunG(L, R);    RunGB(L, R);                                // ใช้ใน loop เอง


/* ===== [9] วิ่งโค้ง cl = ซ้าย, cr = ขวา (ตั้ง set_position_line_l/r ก่อน) ===== */
// ffcl(speed, 'p');              ffcr(speed, 'p');
// fftimercl(speed, ms);          fftimercr(speed, ms);
// fftimercl(speed, ms, 'p');     fftimercr(speed, ms, 'p');
// ffcmcl(speed, cm);             ffcmcr(speed, cm);
// ffcmcl(speed, cm, 'p');        ffcmcr(speed, cm, 'p');
// bbcl(speed, 'p');              bbcr(speed, 'p');
// bbtimercl(speed, ms);          bbtimercr(speed, ms);
// bbtimercl(speed, ms, 'p');     bbtimercr(speed, ms, 'p');
// bbcmcl(speed, cm);             bbcmcr(speed, cm);
// bbcmcl(speed, cm, 'p');        bbcmcr(speed, cm, 'p');


/* ===== [10] มอเตอร์ ===== */
// Motor(L, R);        Move(L, R, ms);
// MotorStop();        MotorStop(ms);        MotorShot();    MotorShot(ms, power);
// fd(speed, ms);      bk(speed, ms);        // หน้า / ถอย
// sl(speed, ms);      sr(speed, ms);        // หมุนซ้าย / ขวา
// tl(speed, ms);      tr(speed, ms);        // เลี้ยวซ้าย / ขวา
// getBatteryVoltage();


/* ===== [11] แขน / เซอร์โว ===== */
// Servo(pin, angle);          // pin: 0, 1, 10, 28
// Servo(a, b, c);             Servo(a, b, c, speed);
// armupdown(angle);           armupdown(angle, speed);
// arm_left_right(l, r);       arm_left_right(l, r, speed);
// S0_trim(x);  S1_trim(x);  S10_trim(x);  S28_trim(x);
// SerialServoControl();       // พิมพ์ "10 90"
// --- Servo.ino (ใส่ speed ได้ เช่น arm_close(2);) ---
// arm_ready();     arm_up();        arm_up45();       arm_down();
// arm_open();      arm_close();     arm_open_l();     arm_open_r();
// arm_open_down(); arm_down_open(); arm_open_up();    arm_up_open();
// arm_down_close();arm_close_down();arm_up_close();   arm_close_up();
// arm_big();       arm_big_box();   arm_big_box_up(); arm_behihd();


/* ===== [12] เซนเซอร์ / Debug / เสียง ===== */
// ReadSensor();   ReadSensorRaw();   ReadCalibrateF();   ReadCalibrateB();   ReadCalibrateC();
// ReadF();        ReadB();           ReadC();            // ค่าดิบ
// F[0-7]  B[0-7]  C[0-1]  Ref  RefC                      // ตัวแปรค่าเซนเซอร์
// calibrateA();   calibrateB();   calibrateC();
// saveCalibA_LOCAL();  saveCalibB_LOCAL();  saveCalibC_LOCAL();
// loadCalibration();   loadCalibration_LOCAL();   printCalibration_LOCAL();
// --- Debug (ค้าง while(1) ใช้ใน Setting() เท่านั้น) ---
// Serial_FrontSensor();   Serial_BackSensor();   Serial_CenterSensor();   Serial_AllSensor();
// SerialCalibrate_FrontSensor();   SerialCalibrate_BackSensor();
// SerialCalibrate_CenterSensor();  SerialCalibrate_AllSensor();
// SerialPositionF();   SerialPositionB();   SerialPositionFB();   SerialDistance();
// --- เสียง ---
// Beep(ms);   Beep2(freq, ms);   beep(freq, ms);   BeepScanner();   blink(n);   BZon();   BZoff();
