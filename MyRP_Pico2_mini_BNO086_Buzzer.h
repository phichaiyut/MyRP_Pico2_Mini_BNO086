#ifndef MYRP_PICO2_MINI_BUZZER_H
#define MYRP_PICO2_MINI_BUZZER_H
// โมดูลที่ไฟล์นี้ใช้ (include ซ้ำได้ มี guard) → include ไฟล์นี้เดี่ยว ๆ หรือสลับลำดับได้
#include <Arduino.h>

/*
 * MyRP_Pico2_mini_BNO086_Buzzer.h
 * เสียง buzzer และไฟกระพริบ
 *
 * สารบัญ:
 *   [1] ขาและความถี่เสียง
 *   [2] ฟังก์ชันเสียง / ไฟ (Beep, BZon, BZoff, blink)
 *   [3] เสียงโหมดคาลิเบรต / ไจโร (SoundWait, SoundStart, SoundSuccess, SoundGyroReady, CalTick*)
 */

// ======================================================================
// [1] ขาและความถี่เสียง
// ======================================================================

#define ToNe 3000
#define buzzer_pin 9

// ======================================================================
// [2] ฟังก์ชันเสียง / ไฟ (Beep, BZon, BZoff, blink)
// ======================================================================

void Beep(int delayb) {
  tone(buzzer_pin, ToNe);
  delay(delayb);
  noTone(buzzer_pin);
}

void Beep2(int freq, int delayb) {
  tone(buzzer_pin, freq);
  delay(delayb);
  noTone(buzzer_pin);
}

void BZon() {
  tone(buzzer_pin, ToNe);
}

void BZoff() {
  noTone(buzzer_pin);
}

void beep(int freq, int dur) {
  tone(buzzer_pin, freq, dur);
  delay(dur + 50);
}

// เสียง "ปี๊บ" สั้น แหลม คล้ายเครื่องยิงบาร์โค้ดใน supermarket
void BeepScanner() {
  beep(ToNe, 80);
}

void blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

// ======================================================================
// [3] เสียงโหมดคาลิเบรต / ไจโร — ยกมาจาก MyMini_16Pro_BNO086_TEST (_Buzzer.h + calibrateGroup())
// ======================================================================
// tone() ของ arduino-pico ใช้ PIO state machine ไม่ใช้ PWM slice
// → ไม่ชนกับมอเตอร์ (PWMA GP19 / PWMB GP6) และ analogWriteFreq() ของมอเตอร์ไม่มีผลกับเสียง
// เซอร์โว (Servo.h ของ arduino-pico) ก็ใช้ PIO เช่นกัน: RP2350 มี 12 SM พอสำหรับเซอร์โว 4 + buzzer 1

#define TONE_CAL_TICK 3000       // CALIBRATION_TONE_HZ  ติ๊ดระหว่างเก็บค่า
#define TONE_COMPLETE 3500       // COMPLETE_TONE_HZ     เสียงเข้าเมนู / สำเร็จ / ออกตัว
#define CAL_TONE_MS 50           // ติ๊ดยาว 50 ms
#define CAL_TONE_INTERVAL_MS 300 // ติ๊ดทุก 300 ms

// เข้าเมนูรอปุ่ม: 3500 Hz 120 ms × 2 เว้น 100 ms (SoundWait ต้นฉบับ)
void SoundWait() {
  Beep2(TONE_COMPLETE, 120);
  delay(100);
  Beep2(TONE_COMPLETE, 120);
}

// กด START ออกจากเมนู: 3500 Hz 400 ms แล้วรอให้ครบ 500 ms (SoundStart ต้นฉบับ)
void SoundStart() {
  noTone(buzzer_pin);
  tone(buzzer_pin, TONE_COMPLETE, 400);
  delay(500);
}

// Calibrate สำเร็จ + บันทึกแล้ว: 3500 Hz 250 ms × 2 เว้น 150 ms (SoundSuccess ต้นฉบับ)
void SoundSuccess() {
  Beep2(TONE_COMPLETE, 250);
  delay(150);
  Beep2(TONE_COMPLETE, 250);
}

// ไจโรตั้งศูนย์สำเร็จ: 2093 / 2637 / 3136 Hz ยาว 65 ms ทุก 80 ms (SoundGyroReady ต้นฉบับ)
void SoundGyroReady() {
  const int notes[] = { 2093, 2637, 3136 };
  for (int i = 0; i < 3; i++) {
    tone(buzzer_pin, notes[i], 65);
    delay(80);
  }
  noTone(buzzer_pin);
}

// ---------- ติ๊ดระหว่างเก็บค่า Calibrate แบบไม่บล็อก (จังหวะเดียวกับ calibrateGroup() ต้นฉบับ) ----------
// CalTickBegin() ก่อนลูปเก็บค่า → CalTickUpdate() ในลูป → CalTickEnd() หลังลูป
// CalTickUpdate() ไม่มี delay: แค่เทียบ millis() และสั่ง tone() ทุก 300 ms
// ความยาว 50 ms ปิดเองด้วย timer alarm ของ tone() จึงไม่ต้องวนมาเช็กเวลาปิด
unsigned long calToneNext = 0;

void CalTickBegin() {
  noTone(buzzer_pin);  // ตัดเสียงเตือนแบตที่อาจค้างอยู่ก่อนเริ่ม
  calToneNext = millis();  // ติ๊ดแรกทันที (t = 0)
}

inline void CalTickUpdate() {
  unsigned long now = millis();
  if ((long)(now - calToneNext) >= 0) {
    tone(buzzer_pin, TONE_CAL_TICK, CAL_TONE_MS);
    calToneNext = now + CAL_TONE_INTERVAL_MS;
  }
}

void CalTickEnd() {
  noTone(buzzer_pin);  // ปิดเสียงสนิท + คืน PIO state machine
}

#endif  // MYRP_PICO2_MINI_BUZZER_H
