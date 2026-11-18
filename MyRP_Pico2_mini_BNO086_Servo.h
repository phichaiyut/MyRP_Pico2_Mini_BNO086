#ifndef MYRP_PICO2_MINI_SERVO_H
#define MYRP_PICO2_MINI_SERVO_H
// โมดูลที่ไฟล์นี้ใช้ (include ซ้ำได้ มี guard) → include ไฟล์นี้เดี่ยว ๆ หรือสลับลำดับได้
#include "MyRP_Pico2_mini_BNO086_Motor.h"

/*
 * MyRP_Pico2_mini_BNO086_Servo.h
 * เซอร์โวและแขนกล
 *
 * สารบัญ:
 *   [1] ขาและตัวแปรเซอร์โว
 *   [2] ตั้งค่า trim และช่วงพัลส์ (SetServoPulse / S0_pulse ...)
 *   [3] สั่งเซอร์โวทีละตัว
 *   [4] ควบคุมเซอร์โวผ่าน Serial
 *   [5] สั่งแขนหลายตัวพร้อมกัน (Servo / armupdown / arm_left_right)
 *   [6] เซอร์โวแบบไม่บล็อก ขยับนุ่ม ทำงานเบื้องหลังไปพร้อมวิ่งตามเส้น (ServoMove / armupdown_smooth ...)
 */

#include <Servo.h>
#include <pico/time.h>
#include <hardware/sync.h>

// ======================================================================
// [1] ขาและตัวแปรเซอร์โว
// ======================================================================

// กำหนดขาเซอร์โว
#define Servo0 0
#define Servo1 1
#define Servo10 10
#define Servo28 28

// สร้างออบเจ็กต์เซอร์โว
Servo Servo_0;
Servo Servo_1;
Servo Servo_10;
Servo Servo_28;

// ตัวแปรสำหรับเก็บค่า trim และมุมก่อนหน้า
int Servo_tim10 = 0;
int Servo_tim0 = 0;
int Servo_tim1 = 0;
int Servo_tim28 = 0;

// ช่วงความกว้างพัลส์ (µs) ของแต่ละเซอร์โว: มุม 0° = min, มุม 180° = max
int Servo_min0 = 500, Servo_max0 = 2500;
int Servo_min1 = 500, Servo_max1 = 2500;
int Servo_min10 = 500, Servo_max10 = 2500;
int Servo_min28 = 500, Servo_max28 = 2500;

// มุมปัจจุบันของแขน: pos[0] = เซอร์โว 1 (ขึ้น-ลง), pos[1] = เซอร์โว 10 (มือซ้าย), pos[2] = เซอร์โว 0 (มือขวา)
// volatile เพราะตัวขยับเบื้องหลัง [6] อัปเดตค่าจาก timer interrupt ด้วย
volatile int pos[3] = { 90, 90, 90 };

// ---------- สถานะตัวขยับเบื้องหลัง [6] (ลำดับ index: 0 = S0, 1 = S1, 2 = S10, 3 = S28) ----------
#define SERVO_SMOOTH_TICK_MS 20  // อัปเดตทุก 20 ms = คาบพัลส์เซอร์โว 50Hz (ถี่กว่านี้เซอร์โวก็รับไม่ทัน)

struct ServoSmoothState {
  float cur;     // มุมที่สั่งออกไปล่าสุด (องศาแบบเดียวกับ Servo(servo, angle) ก่อนกลับด้าน/trim)
  float target;  // มุมเป้าหมาย
  float vmax;    // ความเร็วสูงสุด (องศา/วินาที)
  float vel;     // ความเร็วตอนนี้ (องศา/วินาที) ใช้ทำทางลาดเร่ง/หน่วง
  bool moving;   // กำลังขยับอยู่
  bool known;    // รู้ตำแหน่งแล้ว (เคยสั่งเซอร์โวตัวนี้แล้วอย่างน้อยหนึ่งครั้ง)
};

volatile ServoSmoothState servoSm[4] = {
  { 90, 90, 0, 0, false, false },
  { 90, 90, 0, 0, false, false },
  { 90, 90, 0, 0, false, false },
  { 90, 90, 0, 0, false, false },
};

float ServoSmoothAccel = 400.0f;  // อัตราเร่ง/หน่วง (องศา/วินาที²) 0 = ไม่มีทางลาด
int ServoAutoSpeed = 0;           // >0 = ให้ Servo(servo, angle) เดิมขยับนุ่มเบื้องหลังด้วยความเร็วนี้ (ปิดไว้เป็นค่าเริ่มต้น)

// ใช้ "class Servo" เพราะหลัง [3] ชื่อ Servo จะเป็นชื่อฟังก์ชัน
class Servo *const servoObj[4] = { &Servo_0, &Servo_1, &Servo_10, &Servo_28 };
const uint8_t servoPinOf[4] = { Servo0, Servo1, Servo10, Servo28 };
int *const servoMinOf[4] = { &Servo_min0, &Servo_min1, &Servo_min10, &Servo_min28 };
int *const servoMaxOf[4] = { &Servo_max0, &Servo_max1, &Servo_max10, &Servo_max28 };
int *const servoTrimOf[4] = { &Servo_tim0, &Servo_tim1, &Servo_tim10, &Servo_tim28 };
const int8_t servoPosSlot[4] = { 2, 0, 1, -1 };  // ช่องใน pos[] ของแต่ละตัว (S28 ไม่มี)

// เลขขาเซอร์โว (0, 1, 10, 28) → index 0-3 | -1 = ไม่ใช่ขาเซอร์โว
static inline int servoIdx(int servo) {
  return servo == 0 ? 0 : servo == 1 ? 1 : servo == 10 ? 2 : servo == 28 ? 3 : -1;
}

// ยกเลิกการขยับเบื้องหลังของเซอร์โวตัวนี้ (ใช้ก่อนสั่งแบบทันที กัน timer เขียนทับ)
static void servoSmoothCancel(int idx, float newCur) {
  uint32_t irq = save_and_disable_interrupts();
  servoSm[idx].moving = false;
  servoSm[idx].vel = 0;
  servoSm[idx].cur = newCur;
  servoSm[idx].target = newCur;
  servoSm[idx].known = true;
  restore_interrupts(irq);
}

// ======================================================================
// [2] ตั้งค่า trim และช่วงพัลส์
// ======================================================================

// ตั้งช่วงพัลส์ (µs) ของเซอร์โวตัวที่เลือก (servo: 0, 1, 10, 28)
// ถ้าเซอร์โวถูก attach อยู่แล้ว จะ detach ให้ แล้ว attach ใหม่ด้วยค่าใหม่ตอนสั่ง Servo() ครั้งถัดไป
void SetServoPulse(int servo, int minUs, int maxUs) {
  if (minUs >= maxUs) return;  // ค่าไม่ถูกต้อง ไม่เปลี่ยน
  int idx = servoIdx(servo);
  if (idx >= 0 && servoSm[idx].moving) servoSmoothCancel(idx, servoSm[idx].cur);  // หยุด timer ก่อน detach
  if (servo == 0) {
    Servo_min0 = minUs;
    Servo_max0 = maxUs;
    if (Servo_0.attached()) Servo_0.detach();
  } else if (servo == 1) {
    Servo_min1 = minUs;
    Servo_max1 = maxUs;
    if (Servo_1.attached()) Servo_1.detach();
  } else if (servo == 10) {
    Servo_min10 = minUs;
    Servo_max10 = maxUs;
    if (Servo_10.attached()) Servo_10.detach();
  } else if (servo == 28) {
    Servo_min28 = minUs;
    Servo_max28 = maxUs;
    if (Servo_28.attached()) Servo_28.detach();
  }
}

void S0_pulse(int minUs, int maxUs) {
  SetServoPulse(0, minUs, maxUs);
}

void S1_pulse(int minUs, int maxUs) {
  SetServoPulse(1, minUs, maxUs);
}

void S10_pulse(int minUs, int maxUs) {
  SetServoPulse(10, minUs, maxUs);
}

void S28_pulse(int minUs, int maxUs) {
  SetServoPulse(28, minUs, maxUs);
}

// ฟังก์ชันตั้งค่า trim
void S0_trim(int _s0) {
  Servo_tim0 = _s0;
}

void S1_trim(int _s1) {
  Servo_tim1 = _s1;
}

void S10_trim(int _s10) {
  Servo_tim10 = _s10;
}

void S28_trim(int _s28) {
  Servo_tim28 = _s28;
}

// ======================================================================
// [3] สั่งเซอร์โวทีละตัว
// ======================================================================

// ฟังก์ชันควบคุมเซอร์โว
// หมายเหตุ: ฟังก์ชันชื่อ Servo(...) บังชื่อคลาส Servo ของ <Servo.h> หลังบรรทัดนี้
// ถ้าต้องการสร้างออบเจ็กต์เซอร์โวเพิ่มในโค้ดตัวเอง ให้เขียน "class Servo myServo;" (มีคำว่า class นำหน้า)
void ServoMove(int servo, int angle, int degPerSec);  // ตัวจริงอยู่ใน [6]

// สั่งเซอร์โวทันที (พฤติกรรมเดิม) + ยกเลิกการขยับเบื้องหลังของตัวนั้น
static void servoWriteNow(int servo, int angle) {
  int idx = servoIdx(servo);
  if (idx >= 0) {
    servoSmoothCancel(idx, angle);
    int slot = servoPosSlot[idx];
    if (slot >= 0) pos[slot] = angle;
  }
  if (servo == 10) {
    if (!Servo_10.attached()) Servo_10.attach(Servo10, Servo_min10, Servo_max10);
    Servo_10.write(constrain(angle + Servo_tim10, 0, 180));
  } else if (servo == 0) {
    if (!Servo_0.attached()) Servo_0.attach(Servo0, Servo_min0, Servo_max0);
    Servo_0.write(constrain(180 - angle - Servo_tim0, 0, 180));
  } else if (servo == 1) {
    if (!Servo_1.attached()) Servo_1.attach(Servo1, Servo_min1, Servo_max1);
    Servo_1.write(constrain(180 - angle - Servo_tim1, 0, 180));
  } else if (servo == 28) {
    if (!Servo_28.attached()) Servo_28.attach(Servo28, Servo_min28, Servo_max28);
    Servo_28.write(constrain(angle + Servo_tim28, 0, 180));
  }
}

// ค่าเริ่มต้น (ServoAutoSpeed = 0) = สั่งทันทีเหมือนเดิมทุกประการ
// ถ้าเรียก SetServoAutoSpeed(องศา/วินาที) ไว้ จะขยับนุ่มเบื้องหลังแทน (ไม่บล็อก ไม่หยุดมอเตอร์)
void Servo(int servo, int angle) {
  int idx = servoIdx(servo);
  if (ServoAutoSpeed > 0 && idx >= 0 && servoSm[idx].known) {
    ServoMove(servo, angle, ServoAutoSpeed);
    return;
  }
  servoWriteNow(servo, angle);
}

// ======================================================================
// [4] ควบคุมเซอร์โวผ่าน Serial
// ======================================================================

int currentServo = -1;
int currentAngle = 90;

void SerialServoControl() {
  Serial.println("Serial Servo Control Mode");
  Serial.println("Type: servo angle  (servo id: 0, 1, 10, 28) (ex: 10 90)");

  while (1) {
    // อ่านคำสั่งทีละบรรทัด: "exit" = ออก | "servo angle" เช่น "10 90"
    // (เดิมเช็ก exit ด้วยการอ่านทั้งบรรทัดทิ้งก่อน ตัวเลขจึงไม่เคยถึง parseInt() → สั่งเซอร์โวไม่ได้)
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("exit")) {
        Serial.println("Exiting Serial Servo Control Mode");
        break;
      }

      int sp = command.indexOf(' ');
      if (sp > 0) {
        String angleText = command.substring(sp + 1);
        angleText.trim();
        int s = command.substring(0, sp).toInt();
        int a = angleText.toInt();

        if (s != 0 && s != 1 && s != 10 && s != 28) {
          Serial.println("Servo id must be 0, 1, 10 or 28");
        } else if (a >= 0 && a <= 180) {
          currentServo = s;
          currentAngle = a;

          Serial.print("Set Servo ");
          Serial.print(currentServo);
          Serial.print(" -> ");
          Serial.print(currentAngle);
          Serial.println(" deg");
        } else {
          Serial.println("Angle must be 0-180");
        }
      } else if (command.length() > 0) {
        Serial.println("Format: servo angle  (ex: 10 90) | exit");
      }
    }

    // เขียนค่าเดิมซ้ำเรื่อย ๆ
    if (currentServo != -1) {
      Servo(currentServo, currentAngle);
    }
    delay(50);  // ~50Hz เหมาะกับ servo
  }
}

// ======================================================================
// [5] สั่งแขนหลายตัวพร้อมกัน (Servo / armupdown / arm_left_right)
// ======================================================================

void Servo(int x, int y, int z) {
  MotorStop();
  int a[] = { x, y, z }, s[] = { Servo1, Servo10, Servo0 };
  for (int i = 0; i < 3; i++) Servo(s[i], pos[i] = a[i]);
  delay(20);
}

void Servo(int target1, int target2, int target3, int spd) {
  MotorStop();
  int target[3] = { target1, target2, target3 };
  int sv[3] = { Servo1, Servo10, Servo0 };
  // ถ้าตัวขยับเบื้องหลัง [6] ยังทำงานอยู่ ให้หยุดก่อน แล้วเริ่มทางลาดจากมุมปัจจุบันจริง (pos[] ถูกอัปเดตจาก timer)
  for (int i = 0; i < 3; i++) {
    int k = servoIdx(sv[i]);
    if (servoSm[k].moving) servoSmoothCancel(k, pos[i]);
  }

  while (pos[0] != target[0] || pos[1] != target[1] || pos[2] != target[2]) {
    for (int i = 0; i < 3; i++) {
      pos[i] = pos[i] + (pos[i] < target[i]) - (pos[i] > target[i]);
      servoWriteNow(sv[i], pos[i]);  // ทางลาดนี้คุมจังหวะเองแล้ว ไม่ผ่านโหมด auto smooth
    }
    delay(spd);
  }

  delay(20);
}

void armupdown(int x, int spd) {
  Servo(x, pos[1], pos[2], spd);
}

void arm_left_right(int l, int r, int spd) {
  Servo(pos[0], l, r, spd);
}

void armupdown(int x) {
  Servo(1, pos[0] = x);
}

void arm_left_right(int l, int r) {
  Servo(10, pos[1] = l);
  Servo(0, pos[2] = r);
}

// ======================================================================
// [6] เซอร์โวแบบไม่บล็อก ขยับนุ่ม ทำงานเบื้องหลังไปพร้อมวิ่งตามเส้น
// ======================================================================
//
// ทำงานด้วย hardware timer (repeating_timer ของ pico SDK) ทุก 20 ms บน core 0
//   - ไม่ใช้ delay() ไม่หยุดมอเตอร์ → สั่งแล้วเรียก ff()/fftimer()/turn ต่อได้ทันที
//   - ขยับทีละนิดพร้อมทางลาดเร่ง/หน่วง → กระแสกระชากต่ำ แขนไม่เหวี่ยง ADC เซนเซอร์เส้นไม่สะดุด
//   - ไม่ต้องแก้ลูป PID เดิมเลย (timer ขยับให้เองแม้ระหว่าง delay() หรือ gyro turn)
//   - interrupt ใช้เวลาไม่กี่ไมโครวินาทีต่อรอบ (เขียน FIFO ของ PIO เท่านั้น)
//
// ความเร็วเป็น องศา/วินาที (เทียบกับ spd ของ Servo(a,b,c,spd) แบบเดิมที่เป็น ms/องศา: dps = 1000 / spd)
//   ~60-90 dps = นุ่มมาก วิ่งตามเส้นไม่สะดุด | 120-180 dps = เร็วพอใช้ | >300 dps = แทบเท่าสั่งทันที
//
// หมายเหตุ: เซอร์โวไม่มีตัวบอกตำแหน่ง → ครั้งแรกที่สั่งตัวที่ยังไม่เคยสั่ง จะกระโดดไปเป้าหมายทันที
//          ควรสั่ง Servo(...) / arm_xxx() จัดท่าเริ่มต้นใน setup() ก่อน

static repeating_timer_t servoSmoothTimer;  // ต้องเป็น static/global (SDK เก็บ pointer ไว้)
static bool servoSmoothTimerOn = false;

// มุม (แบบ Servo(servo, angle)) → พัลส์ µs แบบทศนิยม ละเอียดกว่า 1 องศา ขยับช้า ๆ ได้ไม่กระตุก
static int servoAngleToUs(int idx, float angle) {
  float phys = (idx <= 1) ? (180.0f - angle - *servoTrimOf[idx])  // S0, S1 กลับด้าน (เหมือน [3])
                          : (angle + *servoTrimOf[idx]);          // S10, S28
  if (phys < 0) phys = 0;
  if (phys > 180) phys = 180;
  int mn = *servoMinOf[idx], mx = *servoMaxOf[idx];
  return (int)(mn + (mx - mn) * phys / 180.0f + 0.5f);
}

// เรียกจาก timer interrupt: ห้าม attach/delay/Serial ในนี้ — writeMicroseconds ล้าง FIFO ก่อนใส่ จึงไม่ค้าง
static bool servoSmoothISR(repeating_timer_t *) {
  const float dt = SERVO_SMOOTH_TICK_MS / 1000.0f;
  for (int i = 0; i < 4; i++) {
    volatile ServoSmoothState &s = servoSm[i];
    if (!s.moving) continue;

    float d = s.target - s.cur;
    float dist = fabsf(d);
    float v = s.vmax;
    if (ServoSmoothAccel > 0) {
      float vUp = s.vel + ServoSmoothAccel * dt;           // เร่งขึ้นทีละนิด
      float vStop = sqrtf(2.0f * ServoSmoothAccel * dist);  // ความเร็วที่ยังหน่วงหยุดทันพอดีเป้า
      if (vUp < v) v = vUp;
      if (vStop < v) v = vStop;
    }
    float step = v * dt;
    if (step >= dist || dist < 0.05f) {
      s.cur = s.target;
      s.vel = 0;
      s.moving = false;
    } else {
      s.cur = s.cur + (d > 0 ? step : -step);
      s.vel = v;
    }

    if (servoObj[i]->attached()) servoObj[i]->writeMicroseconds(servoAngleToUs(i, s.cur));
    int slot = servoPosSlot[i];
    if (slot >= 0) pos[slot] = (int)lroundf(s.cur);
  }
  return true;  // ทำซ้ำต่อไป
}

// เริ่ม timer (เรียกเองอัตโนมัติตอน ServoMove ครั้งแรก — เรียกใน setup() ล่วงหน้าก็ได้)
void ServoSmoothBegin() {
  if (servoSmoothTimerOn) return;
  // ค่าติดลบ = นับคาบจากจุดเริ่มของรอบก่อน → คาบคงที่ 20 ms ไม่ไหลตามเวลาที่ใช้ใน callback
  servoSmoothTimerOn = add_repeating_timer_ms(-SERVO_SMOOTH_TICK_MS, servoSmoothISR, nullptr, &servoSmoothTimer);
}

// ตั้งอัตราเร่ง/หน่วง (องศา/วินาที²) ค่ามาก = ออกตัวไว | 0 = ความเร็วคงที่ไม่มีทางลาด
void SetServoAccel(float degPerSec2) {
  ServoSmoothAccel = degPerSec2 < 0 ? 0 : degPerSec2;
}

// เปิดโหมดให้คำสั่งเดิม Servo(servo, angle) / armupdown(x) / arm_left_right(l, r) / Servo(a, b, c)
// ขยับนุ่มเบื้องหลังอัตโนมัติด้วยความเร็วนี้ (องศา/วินาที) | 0 = ปิด กลับเป็นสั่งทันทีแบบเดิม
// หมายเหตุ: ฟังก์ชันเดิมที่มี MotorStop()/delay() ในตัว (เช่น Servo(a,b,c) และ arm_xxx ใน Servo.ino) ยังหยุดมอเตอร์เหมือนเดิม
void SetServoAutoSpeed(int degPerSec) {
  ServoAutoSpeed = degPerSec < 0 ? 0 : degPerSec;
}

// สั่งเซอร์โว 1 ตัวไปมุม angle ด้วยความเร็ว degPerSec (องศา/วินาที) แล้ว return ทันที
// servo: 0, 1, 10, 28 | angle: องศาแบบเดียวกับ Servo(servo, angle) (รวม trim/กลับด้านให้เอง)
void ServoMove(int servo, int angle, int degPerSec) {
  int idx = servoIdx(servo);
  if (idx < 0) return;
  angle = constrain(angle, 0, 180);

  // ยังไม่รู้ตำแหน่ง หรือสั่งความเร็ว <= 0 → ไปทันที
  if (!servoSm[idx].known || degPerSec <= 0) {
    servoWriteNow(servo, angle);
    return;
  }

  // attach ใน main (ห้ามทำใน interrupt) โดยส่งพัลส์ของมุมปัจจุบัน กันแขนกระตุกตอนต่อใหม่
  if (!servoObj[idx]->attached()) {
    servoObj[idx]->attach(servoPinOf[idx], *servoMinOf[idx], *servoMaxOf[idx], servoAngleToUs(idx, servoSm[idx].cur));
  }
  ServoSmoothBegin();

  uint32_t irq = save_and_disable_interrupts();
  volatile ServoSmoothState &s = servoSm[idx];
  float newDir = (float)angle - s.cur;
  float oldDir = s.target - s.cur;
  if (!s.moving || newDir * oldDir <= 0) s.vel = 0;  // กลับทิศ → เริ่มเร่งจาก 0 ใหม่
  s.target = angle;
  s.vmax = degPerSec;
  s.moving = ((float)angle != s.cur);
  restore_interrupts(irq);
}

// ใช้ความเร็วเริ่มต้น 120 องศา/วินาที
void ServoMove(int servo, int angle) {
  ServoMove(servo, angle, 120);
}

// true = เซอร์โวตัวนี้ยังขยับไม่ถึงเป้า
bool ServoBusy(int servo) {
  int idx = servoIdx(servo);
  return idx >= 0 && servoSm[idx].moving;
}

// true = ยังมีเซอร์โวตัวใดตัวหนึ่งขยับอยู่
bool ServoBusy() {
  for (int i = 0; i < 4; i++)
    if (servoSm[i].moving) return true;
  return false;
}

// รอจนเซอร์โวทุกตัวถึงเป้า (ไม่แตะมอเตอร์ — ถ้ามอเตอร์ยังหมุนอยู่จะหมุนต่อ ให้ MotorStop() เองก่อนถ้าต้องการ)
// timeoutMs: กันค้าง (ค่าเริ่มต้น 3 วินาที)
void ServoWait(unsigned long timeoutMs = 3000) {
  unsigned long t0 = millis();
  while (ServoBusy() && millis() - t0 < timeoutMs) delay(1);
}

// หยุดเซอร์โวตัวนี้ค้างไว้ที่มุมปัจจุบัน
void ServoHold(int servo) {
  int idx = servoIdx(servo);
  if (idx >= 0 && servoSm[idx].moving) servoSmoothCancel(idx, servoSm[idx].cur);
}

// มุมปัจจุบันที่สั่งออกไป (องศา) | -1 = ขาไม่ถูกต้อง
int ServoAngle(int servo) {
  int idx = servoIdx(servo);
  return idx < 0 ? -1 : (int)lroundf(servoSm[idx].cur);
}

// ---------- แขนกลแบบไม่บล็อก (คู่กับ armupdown / arm_left_right เดิม แต่ไม่หยุดมอเตอร์ ไม่ delay) ----------

// แขนขึ้น-ลง (เซอร์โว 1)
void armupdown_smooth(int x, int degPerSec) {
  ServoMove(Servo1, x, degPerSec);
}

// มือซ้าย (เซอร์โว 10) / มือขวา (เซอร์โว 0) — ปรับความเร็วให้สองข้างถึงเป้าพร้อมกัน
void arm_left_right_smooth(int l, int r, int degPerSec) {
  float dl = fabsf(l - servoSm[2].cur), dr = fabsf(r - servoSm[0].cur);
  float dmax = dl > dr ? dl : dr;
  int vl = degPerSec, vr = degPerSec;
  if (dmax > 0.5f) {
    vl = max(1, (int)(degPerSec * dl / dmax + 0.5f));
    vr = max(1, (int)(degPerSec * dr / dmax + 0.5f));
  }
  ServoMove(Servo10, l, vl);
  ServoMove(Servo0, r, vr);
}

// สามตัวพร้อมกัน (แขน, มือซ้าย, มือขวา) ลำดับเดียวกับ Servo(a, b, c) — ทุกตัวถึงเป้าพร้อมกัน
void Servo_smooth(int x, int l, int r, int degPerSec) {
  int tgt[3] = { x, l, r };
  int idx[3] = { 1, 2, 0 };  // S1, S10, S0
  int pin[3] = { Servo1, Servo10, Servo0 };
  float dmax = 0, dist[3];
  for (int i = 0; i < 3; i++) {
    dist[i] = fabsf(tgt[i] - servoSm[idx[i]].cur);
    if (dist[i] > dmax) dmax = dist[i];
  }
  for (int i = 0; i < 3; i++) {
    int v = dmax > 0.5f ? max(1, (int)(degPerSec * dist[i] / dmax + 0.5f)) : degPerSec;
    ServoMove(pin[i], tgt[i], v);
  }
}

#endif  // MYRP_PICO2_MINI_SERVO_H
