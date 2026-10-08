#ifndef my_BNO08x_H
#define my_BNO08x_H

/*
  BNO08x -> getZ() / zeroZ() สำหรับหุ่นยนต์ (Raspberry Pi Pico 2 / RP2350, Arduino-Pico core)

  ใช้ library sparkfun/SparkFun_BNO080_Arduino_Library (SparkFun BNO080 Cortex Based IMU)

  - ใช้ Game Rotation Vector (quaternion หมุนสัมพัทธ์ ไม่ใช้ magnetometer
    จึงไม่โดนรบกวนจากสนามแม่เหล็ก/มอเตอร์ในสนามแข่ง) แปลงเป็นมุม Yaw (องศา)
  - getZ() คืนค่า Yaw ที่หัก offset แล้ว (zeroZ() เพื่อกำหนดค่าเริ่มต้นเป็น 0)
  - เปิดแค่ report เดียว (Game Rotation Vector) เพราะ SparkFun library อ่านแค่
    report แรกในแต่ละ packet ถ้าเปิดหลาย report ที่ความถี่เท่ากัน เซนเซอร์จะรวม
    ไว้ใน packet เดียวกัน แล้ว report ที่อยู่ถัดไปจะไม่ถูกอ่านเลย (มุมค้างที่ 0)
  - getGyroZ() (ความเร็วการหมุน deg/s) คำนวณจากการเปลี่ยนแปลงของมุม Yaw แทน
*/

#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// ต่อผ่าน I2C, ไม่ใช้ INT/RESET pin แยก
BNO080 bno08x;

// ที่อยู่ I2C ของ IMU: 0x4B (ค่าเริ่มต้น) หรือ 0x4A (ถ้าต่อขา ADR/SA0 ลง GND)
// เปลี่ยนค่าได้ก่อนเรียก RobotSetup()
uint8_t imuAddress = 0x4A;

// Pico 2 (arduino-pico core): I2C0 default คือ SDA=GP4, SCL=GP5
// ถ้าต่อขาอื่น ให้ตั้งก่อน Wire.begin() เช่น
// Wire.setSDA(2); Wire.setSCL(3);

// ช่วงเวลาระหว่าง report (ms)
#define BNO08X_REPORT_INTERVAL_MS 10

static float yawOffsetDeg = 0.0f;
static float currentYawDeg = 0.0f;
static float currentGyroZ = 0.0f;  // deg/s
static unsigned long lastYawMicros = 0;
// true = ยังไม่ได้ตั้งศูนย์ ให้ใช้มุมแรกที่อ่านได้จากเซนเซอร์เป็น 0 องศา
static bool zeroPending = true;

// จำนวน report มุมที่อ่านได้ทั้งหมด ใช้ตรวจว่าเซนเซอร์ส่งข้อมูลมาจริงไหม
// (ถ้าค่าไม่เพิ่มขึ้นเลย แปลว่าไม่ได้รับข้อมูลจาก IMU)
unsigned long imuReportCount = 0;

// true = เจอ IMU ตอน RobotSetup() (ถ้าไม่เจอภายใน 10 วินาที จะเป็น false และข้ามการอ่านเซนเซอร์)
bool imuFound = false;

bool setReports() {
  // SparkFun library ไม่คืนค่าสถานะจาก enableXxx() จึงคืน true เสมอ
  bno08x.enableGameRotationVector(BNO08X_REPORT_INTERVAL_MS);
  return true;
}

// เรียกบ่อย ๆ (เช่นทุก loop) เพื่อดึง report ล่าสุดจากเซนเซอร์
void pollBNO08x() {
  if (!imuFound) return;
  if (bno08x.hasReset()) {
    setReports();
  }
  // อ่าน packet ที่ค้างอยู่ให้หมด (สูงสุด 16 packet ต่อครั้งกันค้าง)
  for (int n = 0; n < 16; n++) {
    uint16_t reportID = bno08x.getReadings();
    if (reportID == 0) break;
    if (reportID == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      float yaw = bno08x.getYaw() * RAD_TO_DEG;
      unsigned long now = micros();
      if (lastYawMicros != 0) {
        float d = yaw - currentYawDeg;
        if (d > 180.0f) d -= 360.0f;
        if (d < -180.0f) d += 360.0f;
        float dt = (now - lastYawMicros) / 1000000.0f;
        if (dt > 0) currentGyroZ = d / dt;
      }
      lastYawMicros = now;
      currentYawDeg = yaw;
      imuReportCount++;
      if (zeroPending) {
        yawOffsetDeg = yaw;
        zeroPending = false;
      }
    }
  }
}

// มุม Yaw (องศา) หลังหัก offset แล้ว, จัดให้อยู่ในช่วง -180..180
float getZ() {
  pollBNO08x();
  if (zeroPending) return 0.0f;  // ยังไม่ได้มุมจริงจากเซนเซอร์
  float z = currentYawDeg - yawOffsetDeg;
  if (z > 180.0f) z -= 360.0f;
  if (z < -180.0f) z += 360.0f;
  return z;
}

// กำหนดให้ทิศปัจจุบันเป็น 0 องศา (เรียกตอน setup หรือก่อนเริ่มวิ่ง)
// ถ้ายังไม่เคยได้มุมจากเซนเซอร์เลย จะตั้งศูนย์ให้อัตโนมัติเมื่อได้มุมแรก
void zeroZ() {
  pollBNO08x();
  if (imuReportCount == 0) {
    zeroPending = true;
    return;
  }
  yawOffsetDeg = currentYawDeg;
  zeroPending = false;
}

// (ทางเลือก) ความเร็วเชิงมุมรอบแกน Z หน่วย deg/s ถ้าต้องการทำ turn-rate PID
float getGyroZ() {
  pollBNO08x();
  return currentGyroZ;
}

#endif // my_BNO08x_H
