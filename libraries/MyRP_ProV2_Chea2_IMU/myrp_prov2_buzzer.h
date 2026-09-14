#ifndef MYRP_PROV2_BUZZER_H
#define MYRP_PROV2_BUZZER_H

#define ToNe 2700
#define buzzer_pin 32
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

#endif


