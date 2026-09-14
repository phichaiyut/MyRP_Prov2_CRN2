
#ifndef MYRP_PROV2_MOTOR_H
#define MYRP_PROV2_MOTOR_H

#define PWMA 6     // PWM ซ้าย
#define AIN1 22
#define AIN2 23

#define PWMB 3     // PWM ขวา
#define BIN1 21
#define BIN2 20

String Freq_motor ;
void set_Freq(String fr_motor)
  {
    Freq_motor = fr_motor;
  }  

void Motor(int pwmL, int pwmR) {
   // ตั้งความละเอียด PWM เป็น 12 บิต (0–4095)
    analogWriteResolution(12);
    // ความถี่ PWM: มอเตอร์ธรรมดา (DC_Motors) ใช้ 1000 Hz, มอเตอร์ Coreless (ค่าอื่น) ใช้ 20000 Hz (ลดเสียงรบกวน)
    if (Freq_motor == "DC_Motors")
      {
        analogWriteFreq(1000);
      }
    else
      {
        analogWriteFreq(20000);
      }
     delayMicroseconds(50);
   
  // แปลงค่าจาก -100..100 ให้เป็น 0..4095
  int pwmValueL = map(abs(pwmL), 0, 100, 0, 4095);
  int pwmValueR = map(abs(pwmR), 0, 100, 0, 4095);

  // มอเตอร์ซ้าย
  if (pwmL > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmL < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    pwmValueL = 0;
  }

  // มอเตอร์ขวา
  if (pwmR > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (pwmR < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwmValueR = 0;
  }

  // ส่งค่า PWM (0–4095)
  analogWrite(PWMA, pwmValueL);
  analogWrite(PWMB, pwmValueR);
}

void Move(int l,int r ,int t){
    Motor(l,r);
    delay(t);
}

void MotorStop(){
    Motor(0,0);
    // delay(t);
}

void MotorStop(int t){
    Motor(0,0);
    Beep(t);
}

// เบรกแบบ active short-brake (ลัดขั้วมอเตอร์ผ่าน back-EMF) กันไถลจากแรงเฉื่อยที่ความเร็วสูง
// ต่างจาก MotorStop() ที่ปล่อยมอเตอร์ให้ไหลอิสระ (coast) โดยตั้งขา IN ทั้งคู่เป็น LOW
void MotorShot(){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMA, 4095);
    analogWrite(PWMB, 4095);
}


int BaseSpeed, LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;
float PID_KP_Front, PID_KD_Front;
float PID_KP_Back, PID_KD_Back;
// int LastError_F, LastError_B;
int L[10], R[10];
int BL[10], BR[10];
float KP[10], KD[10];
float KP_Back[10], KD_Back[10];

// ��˹� index ���ӧ���
#define SPD_10 0
#define SPD_20 1
#define SPD_30 2
#define SPD_40 3
#define SPD_50 4
#define SPD_60 5
#define SPD_70 6
#define SPD_80 7
#define SPD_90 8
#define SPD_100 9

void setBalanceSpeed(int ch, int spdL, int spdR) {
  L[ch] = spdL;
  R[ch] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  BL[ch] = spdL;
  BR[ch] = spdR;
}

void Set_KP_KD(int ch, float kp,float kd){
  KP[ch] = kp;
  KD[ch] = kd;
}

void Set_KP_KD_Back(int ch, float kp,float kd){
  KP_Back[ch] = kp;
  KD_Back[ch] = kd;
}


// เลือกชุดค่าความเร็ว/PID ตามช่วงของ BaseSpeed (ปัดขึ้นเป็นสิบ เช่น 35 ใช้ชุดของ 40)
// เดิมเป็น if/else 10 ชุดที่โครงสร้างเหมือนกันทุกอัน ต่างแค่ index ตาราง จึงรวมเป็นสูตรเดียว:
// ดัชนี 0-9 คำนวณจาก (BaseSpeed - 1) / 10 แล้วจำกัดไม่ให้เกิน SPD_100 (ผลลัพธ์เหมือนเดิมทุกกรณี)
void InitialSpeed() {
  int idx = constrain((BaseSpeed - 1) / 10, SPD_10, SPD_100);

  LeftBaseSpeed = BaseSpeed - L[idx];
  RightBaseSpeed = BaseSpeed - R[idx];
  BackLeftBaseSpeed = BaseSpeed - BL[idx];
  BackRightBaseSpeed = BaseSpeed - BR[idx];
  PID_KP_Front = KP[idx];       // forward PID
  PID_KD_Front = KD[idx];
  PID_KP_Back = KP_Back[idx];   // backward PID
  PID_KD_Back = KD_Back[idx];
}

void fd(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();

  Move(LeftBaseSpeed, RightBaseSpeed, time_ms);
}

void bk(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();

  Move(-BackLeftBaseSpeed, -BackRightBaseSpeed, time_ms);
}

void sl(int Speed, int time_ms) {
  MotorStop(0);
  Move(-Speed, Speed, time_ms);
  MotorStop(0);
}

void sr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, -Speed, time_ms);
  MotorStop(0);
}

void tl(int Speed, int time_ms) {
  MotorStop(0);
  Move(0, Speed, time_ms);
  MotorStop(0);
}

void tr(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, 0, time_ms);
  MotorStop(0);
}

#endif
