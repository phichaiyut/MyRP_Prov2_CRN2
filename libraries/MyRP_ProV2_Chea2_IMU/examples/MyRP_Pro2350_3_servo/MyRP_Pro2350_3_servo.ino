
#include <myrp_prov2_chea2.h>

void setup() {
  RobotSetup();
  Setting() ;

  // *** DIAGNOSTIC: ยืนยันว่าบอร์ดโหลดค่า Kp/Kd ล่าสุดจริงหรือไม่ - ดูใน Serial Monitor ตอนบูต ***
  Serial.println("---- KP/KD Forward table ----");
  for (int i = 0; i <= SPD_100; i++) {
    Serial.print("SPD idx "); Serial.print(i);
    Serial.print(" -> Kp="); Serial.print(KP[i], 4);
    Serial.print(" Kd="); Serial.println(KD[i], 4);
  }

  //arm_up_close();  //--->> ยกแขนขึ้นหุบฝ่ามือเข้า
  // arm_down_open();
  arm_up_open();
  sw();  //--->> คำสั่งรอกดปุ่ม


  ////------------------------------------------------------------------------------>> รันคำสั่งต่าง ๆ ที่นี่

  Mission();


  ////------------------------------------------------------------------------------>> จบการรันคำสั่งต่าง ๆ

  MotorStop();
}

void loop() {
  MotorStop();
  sw();  //--->> คำสั่งรอกดปุ่ม
  Mission();
 
}
