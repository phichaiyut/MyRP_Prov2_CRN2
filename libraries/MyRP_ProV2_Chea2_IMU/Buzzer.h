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


