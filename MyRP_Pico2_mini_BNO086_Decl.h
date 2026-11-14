#ifndef MYRP_PICO2_MINI_DECL_H
#define MYRP_PICO2_MINI_DECL_H

/*
 * MyRP_Pico2_mini_BNO086_Decl.h
 * ประกาศล่วงหน้า (forward declaration) ของฟังก์ชันที่ถูกเรียกข้ามไฟล์ "ย้อนลำดับ include"
 * เช่น Motor.h / PID.h เรียกฟังก์ชันที่ตัวจริงอยู่ใน Gyro.h (include ทีหลัง)
 * รวมไว้ที่เดียว: ถ้าเพิ่มการเรียกข้ามไฟล์แบบนี้ใหม่ ให้เพิ่มบรรทัดที่นี่
 * (ห้ามใส่ค่า default argument ที่นี่ — ให้อยู่ที่ตัวจริงเท่านั้น)
 */

// ---------- ตัวจริงอยู่ใน Gyro.h ----------
void resetGyroRunError();  // ใช้ใน Motor.h InitialSpeed()
void SetRobotAngle();      // ใช้ใน PID.h balance / set
void SetFG(int totalTime);  // ใช้ใน PID.h TrackSelectF / TrackSelectB 'g' 'G'
void SetBG(int totalTime);
void spindir(int Speed, float target_dir);  // ใช้ใน Gyro.h ก่อนตัวจริง (trackSpin / alignHeading)
void turndir(int Speed, float target_dir);
void turndirb(int Speed, float target_dir);

#endif  // MYRP_PICO2_MINI_DECL_H
