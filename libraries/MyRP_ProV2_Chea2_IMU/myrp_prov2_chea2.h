#ifndef MYRP_PROV2_CHEA2_H
#define MYRP_PROV2_CHEA2_H

#include <Wire.h>
#include "myrp_prov2_buzzer.h"
#include "myrp_prov2_sensor.h"
#include "myrp_prov2_motor.h"
#include "myrp_prov2_pid.h"
#include "myrp_prov2_gyro.h"
#include "myrp_prov2_servo.h"


void RobotSetup(){

    Wire.begin();
    Wire1.setSDA(26); // กำหนดพิน SDA
    Wire1.setSCL(27); // กำหนดพิน SCL
    Wire1.begin();
    analogReadResolution(12);

    my.setWire(Wire1);   // GYRO160 ต่ออยู่บน Wire1 (SDA=26, SCL=27) ไม่ใช่ Wire ค่า default
   if (!my.begin()) {
    Serial.println("Failed to initialize GYRO160!");
    //while (1);
   }    
    resetAngles();
    Serial.println("GYRO160 initialized!");
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(28, OUTPUT);
    pinMode(32, OUTPUT);

    pinMode(33, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // MCP3008 (ADC) ใช้ software SPI (sck=14, mosi=15, miso=16) ตั้งขาครั้งเดียวที่นี่ + CS ชุด A (17)
    adc.begin(14, 15, 16, 17);
    pinMode(13, OUTPUT);        // เตรียมขา CS ชุด B (13) ไว้ด้วย — สลับสองชุดด้วย adc.setCS()
    digitalWrite(13, HIGH);     // ใน read_sensorA()/read_sensorB() (ดู myrp_prov2_sensor.h)

    for(int i = 0; i<2; i++)
      {
        for(int i = 0; i<3; i++)
          {
            digitalWrite(rgb[i],1);
            delay(50);
              digitalWrite(rgb[i],0);
              delay(50);
            }
        }
      digitalWrite(rgb[2],1);     
      get_EEP_Program();
      read_sensorA_program();
  }



void sw()
  { MotorStop();
    tone(32, 3000, 100);
    delay(200); // รอ 1 วินาที
    tone(32, 3000, 200);
    delay(200); // รอ 1 วินาที

    int buttonState;
    unsigned long pressStartTime = 0;
    bool isPressed = false;   

    while(1)
      {
        if(digitalRead(19) == 0)
            {
              tone(32, 950, 100);
               digitalWrite(rgb[2],0);
               digitalWrite(rgb[1],1);
               delay(200); // รอ 1 วินาที
              get_maxmin_A();
              for(int i = 0; i<2; i++)
                {
                  digitalWrite(rgb[1],1);
                  delay(100);
                  digitalWrite(rgb[1],0);
                  delay(100);
                }
              digitalWrite(rgb[1],0);
              digitalWrite(rgb[2],1);
              delay(50);

            }
        if(digitalRead(12) == 0)
            {
              tone(32, 950, 100);
              digitalWrite(rgb[2],0);
               digitalWrite(rgb[1],1);
               delay(200); // รอ 1 วินาที
              get_maxmin_B();
              for(int i = 0; i<2; i++)
                {
                  digitalWrite(rgb[1],1);
                  delay(100);
                  digitalWrite(rgb[1],0);
                  delay(100);
                }
              digitalWrite(rgb[1],0);
              digitalWrite(rgb[2],1);
              delay(50);
            }
        Serial.print("From A ");
          for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorA(i));  // ใช้ read_sensorA
            Serial.print(" ");
          }
          Serial.print("   ");
          
          // แสดงค่าจาก Nano 0x09
          Serial.print("From B ");
          for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorB(i));  // ใช้ read_sensorB
            Serial.print(" ");
          }
          Serial.print("  ");

           Serial.print("From C ");
          for (int i = 0; i < 2; i++) {
            C[i] = analogRead(C_PIN[i]);
            Serial.print(C[i]);  // ใช้ read_sensorB
            Serial.print(" ");
          }
          Serial.print("   Gyro Z ");
          Serial.print(my.gyro('z'));
          Serial.println("  ");
        // Serial.println(my_tcs('r'));
          
          delay(100);  // อัปเดตทุก 100ms
        Serial.println(" "); 
        buttonState = digitalRead(33);
        if (buttonState == LOW) 
          {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
            digitalWrite(rgb[2],0);
            if (!isPressed) \
              {
                pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
                isPressed = true;
              } 
            else 
              {
                unsigned long pressDuration = millis() - pressStartTime;    
                if (pressDuration >= 3000) 
                  {  // กดค้าง 3 วินาที
                    tone(32, 950, 100);
                    delay(200); // รอ 1 วินาที
                    tone(32, 950, 200);
                    delay(200); // รอ 1 วินาที
                    Serial.println("Entering Mode A");
                    digitalWrite(rgb[2],0);
               digitalWrite(rgb[1],1);
               delay(200); // รอ 1 วินาที
              get_maxmin_C();
              for(int i = 0; i<2; i++)
                {
                  digitalWrite(rgb[1],1);
                  delay(100);
                  digitalWrite(rgb[1],0);
                  delay(100);
                }
              digitalWrite(rgb[1],0);
              digitalWrite(rgb[2],1);
              delay(50);
                    while (digitalRead(33) == LOW);  // รอให้ปล่อยปุ่ม
                    delay(200);  // ป้องกันการเด้งของปุ่ม
                  }
              }
          } 
        else 
          {
            if (isPressed) 
              {
                unsigned long pressDuration = millis() - pressStartTime;
                
                if (pressDuration >= 50 && pressDuration < 3000) 
                  {  
                    Serial.println("Entering Mode B");
                    break;
                  }
                isPressed = false;
              }
          }
      }
    tone(32, 3000, 400);
    delay(500);
  }

#endif