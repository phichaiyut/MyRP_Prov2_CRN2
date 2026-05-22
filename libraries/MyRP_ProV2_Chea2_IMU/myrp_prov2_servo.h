#include <Servo.h>

// กำหนดขาเซอร์โว
#define Servo39 39
#define Servo38 38
#define Servo37 37
#define Servo36 36
#define Servo35 35
#define Servo34 34

// สร้างออบเจ็กต์เซอร์โว
Servo Servo_39;
Servo Servo_38;
Servo Servo_37;
Servo Servo_36;
Servo Servo_35;
Servo Servo_34;

// ตัวแปรสำหรับเก็บค่า trim และมุมก่อนหน้า
int Servo_tim34 = 0;
int Servo_tim35 = 0;
int Servo_tim36 = 0;

// ฟังก์ชันตั้งค่า trim
void S34_trim(int _s34) {
    Servo_tim34 = _s34;
}

void S35_trim(int _s35) {
    Servo_tim35 = _s35; // แก้ไขจาก servo_tim34 เป็น servo_tim35
}

void S36_trim(int _s36) {
    Servo_tim36 = _s36; // แก้ไขจาก servo_tim34 เป็น servo_tim36
}

// ฟังก์ชันควบคุมเซอร์โว
void Servo(int servo, int angle) {      
    if (servo == 39) {
        Servo_39.attach(Servo39, 600, 2400);
        Servo_39.write(constrain(angle,0,180));        
    } else if (servo == 38) {
        Servo_38.attach(Servo38, 600, 2400);
        Servo_38.write(constrain(angle,0,180));        
    } else if (servo == 37) { 
        Servo_37.attach(Servo37, 600, 2400);
        Servo_37.write(constrain(angle,0,180));        
    } else if (servo == 36) {
        Servo_36.attach(Servo36, 600, 2400);
        Servo_36.write(constrain(angle+Servo_tim36,0,180));      
    } else if (servo == 35) {
        Servo_35.attach(Servo35, 600, 2400);
        Servo_35.write(constrain((180 - angle)-Servo_tim35,0,180));     
    } else if (servo == 34) {
        Servo_34.attach(Servo34, 600, 2400);
        Servo_34.write(constrain(angle+Servo_tim34,0,180) );   
    }
}





int currentServo = -1;
int currentAngle = 90;

int pos[3] ={90,90,90};

void SerialServoControl() {
  Serial.println("Serial Servo Control Mode");
  Serial.println("Type: servo angle  (ex: 39 90)");

  while (1) {
    // ออกจากโหมดถ้าพิมพ์ 'exit'
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("exit")) {
        Serial.println("Exiting Serial Servo Control Mode");
        break;
      }                 
    }

    // รับคำสั่งจาก Serial
    if (Serial.available()) {
      int s = Serial.parseInt();
      int a = Serial.parseInt();

      if (a >= 0 && a <= 180) {
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

      while (Serial.available()) Serial.read();
    }

    // เขียนค่าเดิมซ้ำเรื่อย ๆ
    if (currentServo != -1) {
      Servo(currentServo, currentAngle);
    }

    delay(50); // ~50Hz เหมาะกับ servo
  }
}


void Servo(int x, int y, int z) {
  MotorStop();
  int a[] = {x, y, z}, s[] = {Servo36, Servo34, Servo35};
  for (int i = 0; i < 3; i++) Servo(s[i], pos[i] = a[i]);
  delay(100);
}

void Servo(int target1, int target2, int target3, int spd) {
  MotorStop();

  int target[3] = {target1, target2, target3};
  int sv[3] = {Servo36, Servo34, Servo35};

  while (pos[0] != target[0] || pos[1] != target[1] || pos[2] != target[2]) {
    for (int i = 0; i < 3; i++) {
      pos[i] += (pos[i] < target[i]) - (pos[i] > target[i]);
      Servo(sv[i], pos[i]);
    }
    delay(spd);
  }

  delay(100);
}

void armupdown(int x, int spd) {
  Servo(x, pos[1], pos[2], spd);
}

void arm_left_right(int l, int r, int spd) {
  Servo(pos[0], l, r, spd);
}

void armupdown(int x) {
  Servo(Servo36, pos[0] = x);
}

void arm_left_right(int l, int r) {
  Servo(Servo34, pos[1] = l);
  Servo(Servo35,  pos[2] = r);
}

