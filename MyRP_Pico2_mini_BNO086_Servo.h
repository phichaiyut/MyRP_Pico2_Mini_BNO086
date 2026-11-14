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
 */

#include <Servo.h>

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

// ======================================================================
// [2] ตั้งค่า trim และช่วงพัลส์
// ======================================================================

// ตั้งช่วงพัลส์ (µs) ของเซอร์โวตัวที่เลือก (servo: 0, 1, 10, 28)
// ถ้าเซอร์โวถูก attach อยู่แล้ว จะ detach ให้ แล้ว attach ใหม่ด้วยค่าใหม่ตอนสั่ง Servo() ครั้งถัดไป
void SetServoPulse(int servo, int minUs, int maxUs) {
  if (minUs >= maxUs) return;  // ค่าไม่ถูกต้อง ไม่เปลี่ยน
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
void Servo(int servo, int angle) {
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

// ======================================================================
// [4] ควบคุมเซอร์โวผ่าน Serial
// ======================================================================

int currentServo = -1;
int currentAngle = 90;

int pos[3] = { 90, 90, 90 };

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

  while (pos[0] != target[0] || pos[1] != target[1] || pos[2] != target[2]) {
    for (int i = 0; i < 3; i++) {
      pos[i] += (pos[i] < target[i]) - (pos[i] > target[i]);
      Servo(sv[i], pos[i]);
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

#endif  // MYRP_PICO2_MINI_SERVO_H
