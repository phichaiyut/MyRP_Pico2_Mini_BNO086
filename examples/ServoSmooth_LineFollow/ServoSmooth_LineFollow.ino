/*
 * ServoSmooth_LineFollow.ino
 * ตัวอย่าง: สั่งแขนกล/มือจับแบบไม่บล็อก (ขยับนุ่มเบื้องหลัง) แล้ววิ่งตามเส้นต่อทันที
 *
 * ServoMove / armupdown_smooth / arm_left_right_smooth / Servo_smooth
 *   → return ทันที ไม่หยุดมอเตอร์ ไม่ delay() เซอร์โวขยับเองทุก 20 ms ด้วย hardware timer
 *   → เรียก ff() / fftimer() / bb() / เลี้ยว gyro ต่อได้เลย ไม่ต้องแก้ลูป PID
 *
 * ความเร็วเป็น องศา/วินาที (dps): 60-90 นุ่มมาก | 120-180 ปกติ
 */
#include <MyRP_Pico2_mini_BNO086.h>
#include <hardware/pwm.h>

int servo_down = 50;    // มือจับลงขนานพื้น
int servo_up = 148;     // ยกแขนขึ้น
int servoL_open = 120;  // กางมือซ้าย
int servoR_open = 120;  // กางมือขวา
int closeL = 105, closeR = 105;

// ---------- ตรวจ PWM slice: มอเตอร์กับเซอร์โวต้องไม่อยู่ slice เดียวกัน ----------
// (เซอร์โวของ arduino-pico ใช้ PIO ไม่ใช้ PWM slice อยู่แล้ว แต่พิมพ์ไว้ยืนยัน)
void printPwmSlices() {
  const int pins[] = { PWMA, PWMB, Servo0, Servo1, Servo10, Servo28 };
  const char *names[] = { "PWMA", "PWMB", "Servo0", "Servo1", "Servo10", "Servo28" };
  for (int i = 0; i < 6; i++) {
    Serial.print(names[i]);
    Serial.print(" GP");
    Serial.print(pins[i]);
    Serial.print(" -> slice ");
    Serial.print(pwm_gpio_to_slice_num(pins[i]));
    Serial.println(pwm_gpio_to_channel(pins[i]) ? "B" : "A");
  }
}

// ---------- ทดสอบแยกสาเหตุ: วางหุ่นนิ่งบนเส้น แล้วแกว่งแขน ----------
// ถ้า Pos กระโดดทั้งที่หุ่นไม่ขยับ = ไฟตก/สัญญาณรบกวนทางไฟฟ้า (แก้ที่วงจร: C, BEC แยก)
// ถ้า Pos นิ่ง = สาเหตุคือแรงเหวี่ยงทางกลตอนวิ่ง (แก้ด้วยการขยับช้า/มีทางลาด)
void testServoNoise(bool smooth) {
  MotorStop();
  Servo(1, servo_down);
  delay(800);
  if (smooth) armupdown_smooth(servo_up, 90);
  else Servo(1, servo_up);  // แบบเดิม: กระโดดเต็มสปีด
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    Serial.print(millis() - t0);
    Serial.print("\tPos=");
    Serial.println(readPositionF(200, 50));
    delay(5);
  }
}

void setup() {
  RobotSetup();
  printPwmSlices();

  // จัดท่าเริ่มต้นแบบทันทีก่อน (เซอร์โวไม่มีตัวบอกตำแหน่ง ครั้งแรกต้องกระโดดไปก่อน)
  Servo(1, servo_down);
  Servo(10, servoL_open);
  Servo(0, servoR_open);

  SetServoAccel(400);  // ทางลาดเร่ง/หน่วง 400 องศา/วินาที² (0 = ไม่มีทางลาด)
  // SetServoAutoSpeed(120); // (ทางเลือก) ให้คำสั่งเดิม Servo()/armupdown()/arm_left_right() ขยับนุ่มเองทั้งหมด

  sw();  // รอกดปุ่มเริ่ม
  // testServoNoise(false); testServoNoise(true);  // เปิดเพื่อเทียบค่าเซนเซอร์ แบบเดิม vs แบบนุ่ม

  // ===== ภารกิจตัวอย่าง =====
  // 1) วิ่งไปหาวัตถุ
  ff(40, 's');

  // 2) หนีบวัตถุ: ต้องรอให้หนีบแน่นก่อนยก (หุ่นหยุดอยู่แล้ว ServoWait ไม่แตะมอเตอร์)
  arm_left_right_smooth(closeL, closeR, 120);
  ServoWait();

  // 3) สั่งยกแขนแบบนุ่ม แล้ววิ่งตามเส้นต่อ "ทันที" ไม่ต้องรอแขนถึงเป้า
  armupdown_smooth(servo_up, 90);
  fftimer(40, 300);  // ช่วงออกตัวช้า ๆ ระหว่างแขนยังยก
  ff(50, 's');       // แขนยกเสร็จระหว่างทางเอง

  // 4) ระหว่างวิ่งอยากรู้ว่าแขนถึงหรือยัง
  if (ServoBusy(Servo1)) Serial.println("arm still moving");

  // 5) วางวัตถุ: ลดแขนพร้อมถอยออก
  Servo_smooth(servo_down, servoL_open, servoR_open, 90);  // ทั้ง 3 ตัวถึงเป้าพร้อมกัน
  bbtimer(30, 400);
  MotorStop();
  ServoWait();
}

void loop() {
}
