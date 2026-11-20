#ifndef MYRP_PICO2_MINI_MOTOR_H
#define MYRP_PICO2_MINI_MOTOR_H
// โมดูลที่ไฟล์นี้ใช้ (include ซ้ำได้ มี guard) → include ไฟล์นี้เดี่ยว ๆ หรือสลับลำดับได้
#include "MyRP_Pico2_mini_BNO086_Decl.h"
#include "MyRP_Pico2_mini_BNO086_Buzzer.h"

/*
 * MyRP_Pico2_mini_BNO086_Motor.h
 * ขับมอเตอร์ ชดเชยแบตเตอรี่ และตารางความเร็ว / PID ตามความเร็ว
 *
 * สารบัญ:
 *   [1] ขามอเตอร์ / ชดเชยแรงดันแบตเตอรี่
 *   [2] ขับมอเตอร์ (Motor / Move / MotorStop / MotorShot)
 *   [3] ตารางความเร็ว / PID / เวลาเบรก ตามช่วงความเร็ว
 *   [4] เคลื่อนที่พื้นฐาน (fd, bk, sl, sr, tl, tr)
 */

#include "BatteryMonitor.h"

// ======================================================================
// [1] ขามอเตอร์ / ชดเชยแรงดันแบตเตอรี่
// ======================================================================

#define PWMA 19
#define AIN1 20
#define AIN2 21
#define PWMB 6
#define BIN1 8
#define BIN2 7

const float VMAX = 12.6f;
const float VMIN = 7.4f;
const float VNOM = 11.55f;
bool DC_Motors = true;

BatteryMonitor bat;

inline void updateBattery() {
  bat.update();
}
inline float getBatteryVoltage() {
  return bat.getVoltage();
}

float scale = 1.0f;

void set_Freq(const String &type) {
  if (type == "Coreless_Motors" || type == "coreless") {
    DC_Motors = false;
    analogWriteFreq(20000);
    Serial.println("Motor: Coreless_Motors (20kHz)");
  } else {
    DC_Motors = true;
    analogWriteFreq(1000);
    Serial.println("Motor: DC_Motors (1kHz)");
  }
}

const int MOTOR_PWM_LIMIT = 3850;  // เพดาน PWM (จาก 4095) ของ Motor() / MotorShot()

// ตัวคูณ PWM ชดเชยแบตที่แรงดัน voltage (ใช้ทั้ง bat_control() และระบบชดเชยระยะทางด้านล่าง)
float batt_pwm_scale(float voltage) {
  float s = 1.0f;
  if (voltage > 0.5f && voltage >= VMIN && voltage <= VMAX) {
    s = powf(VNOM / voltage, 0.95f);
  }
  // ป้องกัน scale เกินสำหรับ TB6612FNG (พิกัดกระแสต่ำกว่า VNH7070ASTR มาก จึงจำกัดช่วงให้แคบลง)
  return constrain(s, 0.95f, 1.05f);
}

void bat_control() {
  scale = batt_pwm_scale(getBatteryVoltage());
}

// ---------- ชดเชยแรงดันแบต สำหรับคำสั่งวิ่งตามระยะ (ffcm / bbcm / ffcmg / bbcmg / ffcmgs / bbcmgs) ----------
// ระยะคำนวณจาก ความเร็ว × เวลา × speed_scale → ถ้าแบตต่างจากตอนจูน ระยะจริงจะเพี้ยน
// bat_control() ชดเชย PWM อยู่แล้ว แต่ไม่ครบ เพราะ
//   (1) scale ถูกจำกัด 0.95-1.05 → แบตเต็ม (> ~12.2V) ชดเชยไม่พอ หุ่นวิ่งเร็วกว่าตอนจูน
//   (2) PWM ถูกจำกัดที่ MOTOR_PWM_LIMIT → ความเร็วสูง (~90+) ชดเชยไม่ได้เลยเมื่อแบตอ่อน
//   (3) VNOM ไม่ใช่แรงดันตอนจูน speed_scale
// ระบบนี้จึงชดเชย "เฉพาะส่วนที่ PWM ชดเชยไม่ได้" ที่ตัวนับระยะ (ไม่คูณซ้ำ 2 เด้ง):
//   แรงดันเฉลี่ยที่ถึงมอเตอร์จริง  Veff(V) = V × PWM ที่จ่ายจริง (รวม scale + เพดาน PWM)
//   factor = (Veff(แบตตอนนี้) / Veff(batt_ref_voltage)) ^ batt_comp_gain
//   แบตแรงกว่าตอนจูน → factor > 1 → นับระยะเร็วขึ้น หยุดเร็วขึ้น | แบตอ่อน → factor < 1
float batt_ref_voltage = 11.8f;  // แรงดันแบต (V) ตอนจูน speed_scale_*
float batt_comp_gain = 1.0f;     // ความแรงการชดเชย (ปกติ 0.8-1.2) | 0 = ไม่ชดเชย
bool enable_batt_comp = true;    // เปิด/ปิดระบบชดเชยระยะทาง

const float BATT_COMP_V_LOW = 9.0f;      // นอกช่วงนี้ถือว่าค่าแบตผิดปกติ → ไม่ชดเชย
const float BATT_COMP_V_HIGH = 13.5f;
const float BATT_COMP_FACTOR_MIN = 0.85f;  // จำกัดตัวคูณ กันค่าแบตเพี้ยนทำระยะเพี้ยนมาก
const float BATT_COMP_FACTOR_MAX = 1.15f;

// ref_volt = แรงดันตอนจูน speed_scale (ดูจาก print_battery_comp() หรือ getBatteryVoltage() ตอนจูน)
void set_battery_comp(float ref_volt, float gain = 1.0f, bool enable = true) {
  if (ref_volt >= BATT_COMP_V_LOW && ref_volt <= BATT_COMP_V_HIGH) batt_ref_voltage = ref_volt;
  else Serial.println("set_battery_comp: ref_volt out of range (9.0-13.5 V), keep old value");
  batt_comp_gain = constrain(gain, 0.0f, 2.0f);
  enable_batt_comp = enable;
}

// แรงดันเฉลี่ยที่ถึงมอเตอร์ เมื่อสั่งความเร็ว speed ที่แบต voltage (สัดส่วนเดียวกับ Motor())
static float batt_effective_voltage(float voltage, float speed) {
  float pwm = 4095.0f * speed / 100.0f;
  float applied = min(pwm * batt_pwm_scale(voltage), (float)MOTOR_PWM_LIMIT);
  return voltage * applied / pwm;
}

// ตัวคูณระยะทาง (คูณเข้ากับ speed × speed_scale × dt) | speed = ความเร็วที่สั่งล้อ (0-100)
float batt_distance_factor(float speed) {
  if (!enable_batt_comp || batt_comp_gain <= 0.0f) return 1.0f;
  speed = fabsf(speed);
  if (!(speed > 0.5f)) return 1.0f;
  if (speed > 100.0f) speed = 100.0f;
  float v = getBatteryVoltage();
  if (!(v >= BATT_COMP_V_LOW && v <= BATT_COMP_V_HIGH)) return 1.0f;  // ไม่มีแบต / อ่านค่าผิด

  float ratio = batt_effective_voltage(v, speed) / batt_effective_voltage(batt_ref_voltage, speed);
  float f = (batt_comp_gain == 1.0f) ? ratio : powf(ratio, batt_comp_gain);
  return constrain(f, BATT_COMP_FACTOR_MIN, BATT_COMP_FACTOR_MAX);
}

// พิมพ์ค่าแบต / ตัวคูณ PWM / ตัวคูณระยะ ที่ความเร็ว speed (ไว้ตรวจสอบและหาค่า ref_volt ตอนจูน)
void print_battery_comp(int speed = 50) {
  Serial.print("Batt ");
  Serial.print(getBatteryVoltage(), 2);
  Serial.print(" V | ref ");
  Serial.print(batt_ref_voltage, 2);
  Serial.print(" V | pwm scale ");
  Serial.print(batt_pwm_scale(getBatteryVoltage()), 3);
  Serial.print(" | dist factor @");
  Serial.print(speed);
  Serial.print(" = ");
  Serial.print(batt_distance_factor(speed), 3);
  Serial.println(enable_batt_comp ? "" : " (OFF)");
}

// ======================================================================
// [2] ขับมอเตอร์ (Motor / Move / MotorStop / MotorShot)
// ======================================================================

void Motor(int left, int right) {
  updateBattery();
  static unsigned long lastBatUpdate = 0;
  unsigned long now = millis();

  if (now - lastBatUpdate > 70) {
    bat_control();
    lastBatUpdate = now;
  }
  int pwmL = map(abs(left), 0, 100, 0, 4095);
  int pwmR = map(abs(right), 0, 100, 0, 4095);

  pwmL = constrain((int)(pwmL * scale), 0, MOTOR_PWM_LIMIT);  // Safety limit
  pwmR = constrain((int)(pwmR * scale), 0, MOTOR_PWM_LIMIT);

  // LEFT MOTOR
  if (left > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (left < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    pwmL = 0;
  }

  // RIGHT MOTOR
  if (right > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (right < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwmR = 0;
  }

  analogWrite(PWMA, pwmL);
  analogWrite(PWMB, pwmR);
}

void Move(int l, int r, int t) {
  Motor(l, r);
  delay(t);
}

void MotorStop() {
  Motor(0, 0);
}

void MotorStop(int t) {
  Motor(0, 0);
  Beep(t);
}

// Active short-brake: ทั้ง IN1/IN2 เป็น HIGH -> มอเตอร์ลัดวงจรผ่าน back-EMF หยุดกะทันหัน
// power = แรงเบรก 0-100%, t = ระยะเวลาที่คงการเบรก (ms) แล้วปล่อยกลับสู่ MotorStop() ให้อัตโนมัติ
// จำกัดเวลา + จำกัด PWM + ใส่ cooldown กันเรียกรัวถี่ ๆ เพื่อไม่ให้กระแสค้างจนไดร์ฟมอเตอร์ร้อนพัง
void MotorShot(int t = 3, int power = 90) {
  static unsigned long lastShot = 0;
  unsigned long now = millis();
  if (now - lastShot < 50) {  // cooldown กันเบรกซ้อนถี่เกินไป
    MotorStop();              // ไม่ช็อตเบรกซ้ำ แต่ต้องหยุดมอเตอร์ ไม่ปล่อยวิ่งด้วยคำสั่งก่อนหน้า
    return;
  }
  lastShot = now;

  updateBattery();
  bat_control();
  int pwm = map(constrain(power, 0, 100), 0, 100, 0, 4095);
  pwm = constrain((int)(pwm * scale), 0, MOTOR_PWM_LIMIT);  // Safety limit เดียวกับ Motor()

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);

  delay(constrain(t, 1, 15));  // จำกัดเวลาเบรกสูงสุด กันค้างที่กระแสสูง
  MotorStop();
}

// ======================================================================
// [3] ตารางความเร็ว / PID / เวลาเบรก ตามช่วงความเร็ว
// ======================================================================

int BaseSpeed, LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;
float PID_KP_Front, PID_KD_Front;
float PID_KP_Back, PID_KD_Back;
int delay_break_f = 30;
int delay_break_b = 30;
int L[10], R[10];
int BL[10], BR[10];
float KP[10], KD[10];
float KP_Back[10], KD_Back[10];
int delay_break_f_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };
int delay_break_b_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };

// ดัชนีตารางความเร็ว (ใช้กับ setBalanceSpeed/Set_KP_KD ฯลฯ)
#define SPD_10 0
#define SPD_20 1
#define SPD_30 2
#define SPD_40 3
#define SPD_50 4
#define SPD_60 5
#define SPD_70 6
#define SPD_80 7
#define SPD_90 8
#define SPD_100 9

// แปลง ch เป็นดัชนีตาราง 0-9: รับ SPD_10..SPD_100 (0-9) และรับเลขความเร็วตรง ๆ 10, 20, ..., 100 ด้วย
// (เดิมใส่ 10-100 จะเขียนเกินขอบอาร์เรย์ทับตัวแปรอื่นแบบเงียบ ๆ) | ค่าอื่นคืน -1 = ไม่บันทึก
static int speedTableIndex(int ch) {
  if (ch >= SPD_10 && ch <= SPD_100) return ch;
  if (ch >= 10 && ch <= 100 && ch % 10 == 0) return ch / 10 - 1;
  Serial.print("Speed table: invalid index ");
  Serial.println(ch);
  return -1;
}

void setBalanceSpeed(int ch, int spdL, int spdR) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  L[i] = spdL;
  R[i] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  BL[i] = spdL;
  BR[i] = spdR;
}

void Set_KP_KD(int ch, float kp, float kd) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  KP[i] = kp;
  KD[i] = kd;
}

void Set_KP_KD_Back(int ch, float kp, float kd) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  KP_Back[i] = kp;
  KD_Back[i] = kd;
}

void SetDelayBreak(int ch, int delay_f, int delay_b) {
  int i = speedTableIndex(ch);
  if (i < 0) return;
  delay_break_f_table[i] = delay_f;
  delay_break_b_table[i] = delay_b;
}

// ---------- Safety timeout ของลูปวิ่งหาเส้น / เซนเซอร์ (LINE_WHILE) ----------
// ลูป while(1) ที่รอเจอเส้น / ระยะ (ff, bb, ToCenter, TurnLeft, spinl, balancef, TrackSelect ...)
// ถ้าหุ่นหลุดเส้น / ติดขอบ / ล้อฟรี จะวนไม่จบและจ่ายไฟมอเตอร์ค้าง → ตั้งเวลาสูงสุดต่อ 1 ลูปได้
// set_line_timeout(ms [, halt]) | ms = 0 ปิด (ค่าเริ่มต้น = พฤติกรรมเดิมทุกอย่าง)
//   halt = true (ค่าเริ่มต้น): หยุดมอเตอร์แล้วค้างไว้ บี๊บเตือนทุก 1 วินาที จนกด reset (ปลอดภัยสุด)
//   halt = false: หยุดมอเตอร์ บี๊บสั้น แล้วออกจากลูปไปทำคำสั่งถัดไป
// ตั้งให้นานกว่าลูปที่ยาวที่สุดในภารกิจพอสมควร เช่น 8000-15000 ms
uint32_t line_timeout_ms = 0;
bool line_timeout_halt = true;
uint32_t line_timeout_count = 0;  // จำนวนครั้งที่ timeout ทำงาน (ไว้ debug)

void set_line_timeout(uint32_t ms, bool halt = true) {
  line_timeout_ms = ms;
  line_timeout_halt = halt;
}

struct LineGuard {
  uint32_t t0;
};

LineGuard lineGuardStart() {
  LineGuard g;
  g.t0 = millis();
  return g;
}

// คืน true = หมดเวลา (หยุดมอเตอร์ให้แล้ว) | โหมด halt จะไม่คืนค่า (ค้างจนกด reset)
bool lineGuardExpired(LineGuard &g) {
  if (line_timeout_ms == 0 || millis() - g.t0 <= line_timeout_ms) return false;
  MotorStop();
  BZoff();
  line_timeout_count++;
  Serial.println("Line timeout: line / sensor not found -> motor stop");
  if (line_timeout_halt) {
    Serial.println("Robot halted. Press reset.");
    while (1) {
      MotorStop();
      digitalWrite(LED_BUILTIN, HIGH);
      Beep(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(800);
    }
  }
  Beep(100);
  return true;
}

// ใช้แทน while (1) ในลูปวิ่งหาเส้น: break / continue ใช้ได้เหมือนเดิม
// (ชื่อตัวแปร guard ต่อท้ายด้วยเลขบรรทัด → ลูปซ้อนกันแต่ละชั้นมี guard ของตัวเอง ไม่บังชื่อกัน)
#define LINE_GUARD_CAT2(a, b) a##b
#define LINE_GUARD_CAT(a, b) LINE_GUARD_CAT2(a, b)
#define LINE_GUARD_VAR LINE_GUARD_CAT(lg_, __LINE__)
#define LINE_WHILE for (LineGuard LINE_GUARD_VAR = lineGuardStart(); !lineGuardExpired(LINE_GUARD_VAR);)

int LastError_F, LastError_B;            // ค่า error ก่อนหน้าของ PIDF / PIDB
int LastError_F_none, LastError_B_none;  // ค่า error ก่อนหน้าของ PID แบบ _none

// เลือกชุดค่าความเร็ว/PID ตามช่วงของ BaseSpeed (ปัดขึ้นเป็นสิบ เช่น 35 ใช้ชุดของ 40)
// เดิมเป็น if/else 10 ชุดที่โครงสร้างเหมือนกันทุกอัน ต่างแค่ index ตาราง จึงรวมเป็นสูตรเดียว:
// ดัชนี 0-9 คำนวณจาก (BaseSpeed - 1) / 10 แล้วจำกัดไม่ให้เกิน SPD_100 (ผลลัพธ์เหมือนเดิมทุกกรณี)
void InitialSpeed() {
  int idx = constrain((BaseSpeed - 1) / 10, SPD_10, SPD_100);

  LeftBaseSpeed = BaseSpeed - L[idx];
  RightBaseSpeed = BaseSpeed - R[idx];
  BackLeftBaseSpeed = BaseSpeed - BL[idx];
  BackRightBaseSpeed = BaseSpeed - BR[idx];
  PID_KP_Front = KP[idx];  // forward PID
  PID_KD_Front = KD[idx];
  PID_KP_Back = KP_Back[idx];  // backward PID
  PID_KD_Back = KD_Back[idx];
  delay_break_f = delay_break_f_table[idx];
  delay_break_b = delay_break_b_table[idx];
  LastError_F = 0;  // รีเซ็ต D-term ไม่ให้ค้างจากคำสั่งก่อนหน้า
  LastError_B = 0;
  LastError_F_none = 0;
  LastError_B_none = 0;
  resetGyroRunError();  // previous_errorG / previous_errorGB = error ปัจจุบัน
}

// ======================================================================
// [4] เคลื่อนที่พื้นฐาน (fd, bk, sl, sr, tl, tr)
// ======================================================================

void fd(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(LeftBaseSpeed, RightBaseSpeed, time_ms);
}

void bk(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(-BackLeftBaseSpeed, -BackRightBaseSpeed, time_ms);
}

void sl(int Speed, int time_ms) {
  MotorStop(0);
  Move(-Speed, Speed, time_ms);
  MotorStop(0);
}

void sr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, -Speed, time_ms);
  MotorStop(0);
}

void tl(int Speed, int time_ms) {
  MotorStop(0);
  Move(0, Speed, time_ms);
  MotorStop(0);
}

void tr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, 0, time_ms);
  MotorStop(0);
}

#endif  // MYRP_PICO2_MINI_MOTOR_H
