#ifndef MYRP_PICO2_MINI_H
#define MYRP_PICO2_MINI_H

/*
 * MyRP_Pico2_mini_BNO086.h
 * ไฟล์หลักของไลบรารี: รวมทุกโมดูล + RobotSetup() + เมนูปุ่ม sw()
 *
 * สารบัญ:
 *   [1] รวมโมดูลทั้งหมด (include)
 *   [2] เริ่มต้นระบบ (RobotSetup)
 *   [3] อ่าน ADC ภายนอก (MCP3421)
 *   [4] เมนูปุ่ม sw() — คาลิเบรต / แสดงค่าเซนเซอร์
 */

// ======================================================================
// [1] รวมโมดูลทั้งหมด (include)
// ======================================================================

#include "MyRP_Pico2_mini_BNO086_Buzzer.h"
#include "MyRP_Pico2_mini_BNO086_Sensor.h"
#include "MyRP_Pico2_mini_BNO086_Motor.h"
#include "MyRP_Pico2_mini_BNO086_PID.h"
#include "MyRP_Pico2_mini_BNO086_Servo.h"
#include "MyRP_Pico2_mini_BNO086_Gyro.h"

// ======================================================================
// [2] เริ่มต้นระบบ (RobotSetup)
// ======================================================================

void RobotSetup() {
  Serial.begin(115200);
  // USB Serial ทิ้งข้อความถ้ายังไม่ได้เปิด Serial Monitor → รอสูงสุด 3 วินาที ให้เห็นข้อความวินิจฉัยตอนบูต
  unsigned long serialWait = millis();
  while (!Serial && millis() - serialWait < 3000) delay(10);
  delay(100);

  // ---------- วินิจฉัยบัส I2C ตอนบูต ----------
  // อ่านสาย SDA/SCL ก่อนเริ่ม Wire: ปล่อยว่างแล้วต้องเป็น HIGH ทั้งคู่
  // LOW = มีชิปดึงบัสค้าง / สายช็อต / ไม่มี pull-up → ไม่ใช่ปัญหา address ในโค้ด
  bool sdaHigh, sclHigh;
  i2cReadLines(sdaHigh, sclHigh);
  Serial.print("I2C lines before init: SDA(GP4)=");
  Serial.print(sdaHigh ? "HIGH" : "LOW !!");
  Serial.print("  SCL(GP5)=");
  Serial.println(sclHigh ? "HIGH" : "LOW !!");

  // กู้บัส (9-10 clock + STOP) แล้วเริ่ม Wire พร้อม clock 100 kHz + timeout 25 ms
  if (!i2cBusRecover()) {
    Serial.println("I2C bus still stuck after recovery! SCL LOW = a chip holds the clock -> power-cycle the robot / check wiring");
  }

  bat.begin();  // เรียก Wire.begin() ซ้ำข้างใน (Wire ทำงานอยู่แล้วจึงไม่มีผล) แล้วอ่าน ADS1115
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(DC_Motors ? 1000 : 20000);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(9, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  adc.begin(14, 15, 16, 13);  // ตั้งขา clk/mosi/miso ครั้งเดียว + CS ชุด B (13)
  pinMode(17, OUTPUT);        // เตรียมขา CS ชุด A (17) ไว้ด้วย — สลับสองชุดด้วย adc.setCS()
  digitalWrite(17, HIGH);     // ใน read_sensorA()/read_sensorB() (ดู MyRP_Pico2_mini_BNO086_Sensor.h)

  loadCalibration();
  loadCalibration_LOCAL();
  // พิมพ์อุปกรณ์ I2C ที่เจอทั้งหมด (คาดว่า 0x20 PCF8574, 0x48 ADS1115, 0x4A/0x4B BNO08x, 0x50 EEPROM, 0x68 MCP3421)
  i2cScan();

  // หา BNO08x ที่ 0x4A แล้ว 0x4B (probe ก่อน ไม่เจอ = NACK ทันที) ลองซ้ำได้ไม่เกิน 3 วินาที
  // ไม่เจอ → ทำงานต่อโดยไม่มี gyro: ฟังก์ชันอ่านมุมคืนค่าเดิมทันที และลองต่อใหม่เองทุก 2 วินาที
  uint8_t imuAddr = imuDetect(3000);
  if (imuAddr == 0) {
    Serial.println("!!! BNO08x NOT detected at 0x4A or 0x4B !!!");
    Serial.println("    -> check IMU power/wiring (SDA=GP4, SCL=GP5) and the scan list above");
    Beep(500);
    delay(200);
    Beep(500);
    delay(200);
    Beep(500);
    delay(200);
    Beep(500);
  } else {
    Serial.print("BNO08x found at 0x");
    Serial.println(imuAddr, HEX);
    i2cApplyConfig();  // ต้องเป็น 100 kHz ก่อนเปิด report / อ่านมุม (400 kHz มุมไม่มา)
    // เปิด Game Rotation Vector (ค่าเริ่มต้น) + เช็ก wasReset + warm-up 500ms ให้ฟิลเตอร์นิ่ง
    // แล้วตั้ง 0° จากค่าเฉลี่ยที่นิ่งแล้ว (sw() ไม่ตั้ง 0° ใหม่ — เรียก resetAngles() เองถ้าต้องการ)
    bool gyroZeroOk = gyroBegin(500);
    Serial.println(gyro_imu_ready ? "BNO08x reports OK" : "BNO08x stopped responding during warm-up!");
    if (gyroZeroOk) SoundGyroReady();  // ตั้ง 0° สำเร็จ: 2093/2637/3136 Hz (แบบต้นฉบับ GyroReset)
  }

  // ย้ำ clock 100 kHz + timeout 25 ms (+ กู้บัสอัตโนมัติ) หลังสุด กันมีโค้ดใดตั้งทับไว้
  // timeout เริ่มต้นของ Wire คือ 1000 ms ต่อทรานแซกชัน → ถ้าบัสค้าง sw() จะค้างรอบละหลายวินาที
  i2cApplyConfig();
  blink(3);
  Serial.println("MyRP_Pico2 Class Ready for Competition!");
}

// ======================================================================
// [3] อ่าน ADC ภายนอก (MCP3421)
// ======================================================================

// อ่านค่าจาก ADC ภายนอก (MCP3421) ผ่าน I2C — ใช้ตรวจแบตเตอรี่ในเมนู sw()
int ADC_i2c() {
  long ADC01 = 0;
  int adc_01;

  Wire.requestFrom(MCP3421_ADDR, 4);
  if (Wire.available() == 4) {
    byte b1 = Wire.read();
    byte b2 = Wire.read();
    byte b3 = Wire.read();
    Wire.read();

    ADC01 = ((long)b1 << 16) | ((long)b2 << 8) | b3;
    if (b1 & 0x80) ADC01 |= 0xFF000000;

    adc_01 = map(ADC01, 524048, 282, 4000, 0);
    return adc_01;
  }

  // คืนค่า 9999 หากไม่พบเซนเซอร์ เพื่อป้องกันไม่ให้เข้าเงื่อนไข < 2500
  return 9999;
}

// ======================================================================
// [4] เมนูปุ่ม sw() — คาลิเบรต / แสดงค่าเซนเซอร์
// ======================================================================

void sw() {
  MotorStop();
  SoundWait();  // เข้าเมนู: 3500 Hz 120 ms × 2 (แบบต้นฉบับ MyMini_16Pro)

  unsigned long pressStartTime = 0;
  bool isPressed = false;
  unsigned long lastBuzz = 0;

  // สรุปสถานะ I2C/IMU อีกครั้ง (เผื่อเปิด Serial Monitor หลังบูต จึงไม่เห็นข้อความตอน RobotSetup)
  Serial.print("IMU: ");
  if (imu_connected) {
    Serial.print("BNO08x @ 0x");
    Serial.println(imu_i2c_addr, HEX);
  } else {
    Serial.println("NO IMU (auto-retry every 2 s in this menu)");
  }
  i2cScan();
  uint32_t lastYawSeen = yawSampleCount;
  unsigned long lastYawSeenMs = millis();

  // ส่วนตรวจสอบปุ่ม + แสดงค่าเซนเซอร์
  while (true) {
    bat.update_led();
    updateBattery();
    float voltage = getBatteryVoltage();  // เตือนเมื่อแบตต่ำกว่า 11.0V แต่ยังไม่ถึงขั้นวิกฤติ
    if (voltage <= 11.0f && voltage > 7.0f) {
      unsigned long now = millis();
      // ส่งเสียงทุก 3 วินาที (3000 ms)
      if (now - lastBuzz >= 3000) {
        Serial.println(voltage);  // แสดงแรงดันไฟ
        // เสียงเตือนแบบ Sci-Fi / Cyber Tech (เท่และชัดเจน)
        tone(9, 2800, 50);
        delay(30);
        tone(9, 3200, 50);
        delay(30);
        tone(9, 2400, 120);
        delay(100);
        tone(9, 1800, 80);
        lastBuzz = now;  // อัพเดทเวลาล่าสุดที่ส่งเสียง
      }
    }

    // ปุ่มบน (ปุ่ม 3) → Calibrate A
    if (digitalRead(3) == LOW) {
      digitalWrite(LED_BUILTIN, HIGH);
      calibrateA();  // ติ๊ดเริ่มทันทีใน calibrateA() (ต้นฉบับไม่มีเสียงแยกตอนกด)
      saveCalibA_LOCAL();
      blink(5);
      delay(300);
      digitalWrite(LED_BUILTIN, LOW);
    }

    if (ADC_i2c() < 2500) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      calibrateB();  // ติ๊ดเริ่มทันทีใน calibrateB()
      saveCalibB_LOCAL();
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      while (digitalRead(2) == LOW)
        ;          // รอปล่อยปุ่ม
      delay(400);  // ป้องกันการเด้งของปุ่ม
    }

    // แสดงค่าซีเรียลทุก 100ms
    Serial.print("From A: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(read_sensorA(i));
      Serial.print(" ");
    }
    Serial.print("   From B: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(read_sensorB(i));
      Serial.print(" ");
    }
    Serial.print("   From C: ");
    Serial.print(analogRead(26));
    Serial.print(" ");
    Serial.print(analogRead(27));
    Serial.print(" ");
    Serial.print("   Gyro: ");
    imuTryReconnect();                  // ไม่มี IMU → ลองต่อใหม่ทุก 2 วินาที (มอเตอร์หยุดอยู่ในเมนูนี้)
    float yawShow = angleReadLatest();  // มุม Yaw ล่าสุด (ล้างคิวที่ค้างระหว่างรอ 100ms) | ไม่มี IMU = คืนทันที
    if (yawSampleCount != lastYawSeen) {
      lastYawSeen = yawSampleCount;
      lastYawSeenMs = millis();
    }
    if (!imu_connected) Serial.print("NO IMU");
    else if (millis() - lastYawSeenMs > 300) Serial.print("NO DATA");  // ต่ออยู่แต่ไม่มีมุมใหม่ส่งมา
    else Serial.print(yawShow);
    Serial.print("   Batt: ");
    updateBattery();
    Serial.print(getBatteryVoltage(), 2);  // แรงดันแบต (V) ที่กรองแล้ว
    Serial.print("V");

    Serial.println();
    delay(100);  // อ่าน/แสดงผลทุก 100ms กันสแปม Serial/สแกน ADC รัวๆ

    // ตรวจจับกดค้างปุ่ม 2 นาน 3 วินาที → Calibrate C
    if (digitalRead(2) == LOW) {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
      if (!isPressed) {
        pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
        isPressed = true;
      } else {
        unsigned long pressDuration = millis() - pressStartTime;

        if (pressDuration >= 3000) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(200);
          digitalWrite(LED_BUILTIN, LOW);
          delay(200);
          digitalWrite(LED_BUILTIN, HIGH);
          calibrateC();  // ติ๊ดเริ่มทันทีใน calibrateC()
          saveCalibC_LOCAL();
          digitalWrite(LED_BUILTIN, LOW);
          delay(100);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(100);
          digitalWrite(LED_BUILTIN, LOW);
          delay(100);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(100);
          digitalWrite(LED_BUILTIN, LOW);
          delay(100);
          while (digitalRead(2) == LOW)
            ;          // รอปล่อยปุ่ม
          delay(400);  // ป้องกันการเด้งของปุ่ม
        }
      }
    } else {
      if (isPressed) {
        unsigned long pressDuration = millis() - pressStartTime;

        if (pressDuration >= 50 && pressDuration < 3000) {
          Serial.println("Entering Mode B");
          break;
        }
        isPressed = false;
      }
    }
  }

  // โหลดค่า calibration จาก EEPROM หลังออกจากเมนู
  loadCalibration();
  loadCalibration_LOCAL();
  SoundStart();  // ออกตัว: 3500 Hz 400 ms แล้วรอครบ 500 ms (แบบต้นฉบับ)
  // ไม่ตั้ง 0° ใหม่ที่นี่ — ถ้าต้องการให้ทิศตอนปล่อยตัวเป็น 0° ให้เรียก resetAngles() เองหลัง sw()
}

#endif  // MYRP_PICO2_MINI_H
