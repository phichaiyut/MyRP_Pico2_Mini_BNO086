// ทดสอบ BNO086 แบบแยกเดี่ยว (ไม่เรียก RobotSetup) เพื่อหาว่าทำไมไม่ได้รับข้อมูล
// เปิด Serial Monitor ที่ 115200 แล้วคัดลอกผลลัพธ์ทั้งหมดมาดู

#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#define IMU_ADDR 0x4A      // ตรงกับ imuAddress ใน my_BNO08x.h
#define I2C_CLOCK 100000   // ลอง 100000 ก่อน ถ้าได้แล้วค่อยลอง 400000

BNO080 imu;
unsigned long packets = 0, rvReports = 0, lastPrint = 0, lastEnable = 0;

void scanI2C() {
  Serial.print("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x");
      Serial.print(a, HEX);
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);
  delay(500);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(I2C_CLOCK);

  scanI2C();

  if (!imu.begin(IMU_ADDR, Wire)) {
    Serial.println("begin() FAILED");
    while (1) delay(10);
  }
  Serial.println("begin() OK");

  imu.enableGameRotationVector(10);
  lastEnable = millis();
  Serial.println("enableGameRotationVector(10) sent");
}

void loop() {
  uint16_t id = imu.getReadings();
  uint16_t len = ((uint16_t)imu.shtpHeader[1] << 8 | imu.shtpHeader[0]) & 0x7FFF;

  if (id != 0 || len != 0) {
    packets++;
    // พิมพ์ 20 packet แรกแบบละเอียด
    if (packets <= 20) {
      Serial.print("pkt len=");
      Serial.print(len);
      Serial.print(" ch=");
      Serial.print(imu.shtpHeader[2]);
      Serial.print(" data0=0x");
      Serial.print(imu.shtpData[0], HEX);
      Serial.print(" data5=0x");
      Serial.print(imu.shtpData[5], HEX);
      Serial.print(" -> id=0x");
      Serial.println(id, HEX);
    }
  }
  if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) rvReports++;

  if (imu.hasReset()) {
    Serial.println("IMU reset detected -> enable report again");
    imu.enableGameRotationVector(10);
    lastEnable = millis();
  }

  // ถ้า 2 วินาทียังไม่ได้ข้อมูลมุมเลย ลองสั่งเปิด report ใหม่
  if (rvReports == 0 && millis() - lastEnable > 2000) {
    Serial.println("no rotation data yet -> enable report again");
    imu.enableGameRotationVector(10);
    lastEnable = millis();
  }

  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("packets=");
    Serial.print(packets);
    Serial.print(" rv=");
    Serial.print(rvReports);
    Serial.print(" yaw=");
    Serial.println(imu.getYaw() * RAD_TO_DEG);
  }
}
