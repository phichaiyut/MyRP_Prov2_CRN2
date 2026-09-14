#ifndef MYRP_PROV2_SENSOR_H
#define MYRP_PROV2_SENSOR_H

#define NUM_SENSORS 8
#include <Wire.h>
#include "my_MCP3008s.h"
my_MCP3008s adc;

#define EEPROM_ADDRESS 0x50  // ที่อยู่ I2C ของ EEPROM (24LCxx ทั่วไป) - ตรวจสอบให้ตรงกับฮาร์ดแวร์จริง
#define numSensors NUM_SENSORS
#define numSamples 400  // จำนวนตัวอย่างที่ใช้ตอน calibrate (~3 วิสำหรับ A/B, ~2 วิสำหรับ C ให้พอกวาดเซนเซอร์ผ่านเส้น)

int rgb[] = {24, 25, 28};
char led = 'b';

int F[NUM_SENSORS], B[NUM_SENSORS], C[2];
int sensorMaxA[8], sensorMinA[8];
int sensorMaxB[8], sensorMinB[8];
int sensorMaxC[2], sensorMinC[2];
int sensorValuesA[numSensors][numSamples];
int sensorValuesB[numSensors][numSamples];
int sensorValuesC[2][numSamples];

uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7 };
uint8_t B_PIN[NUM_SENSORS] = { 7, 6, 5, 4, 3, 2, 1, 0 };
uint8_t C_PIN[2] = {46, 47};
int minValueF[NUM_SENSORS], maxValueF[NUM_SENSORS];
int minValueB[NUM_SENSORS], maxValueB[NUM_SENSORS];
int minValueC[2], maxValueC[2];
#define CCL 1
#define CCR 0
int Ref = 500;
int RefC = 500;
int LineColor = 0;
int DIST = A3;
// FRONT
int FRONT_MIN = 80;
int FRONT_MAX = 900;

// CENTER
int CENTER_MIN = 200;
int CENTER_MAX = 900;

// BACK
int BACK_MIN = 0;
int BACK_MAX = 1000;

// หมายเหตุ: มี MCP3008 สองตัวอยู่บนบัส SPI (software SPI) เดียวกัน แยกกันด้วยขา CS
// (17 = ชุด A, 13 = ชุด B) เนื่องจาก my_MCP3008s เก็บสถานะ CS ไว้ในอ็อบเจกต์เดียว (adc)
// จึงต้องสลับ CS ก่อนอ่านทุกครั้ง — ใช้ adc.setCS() แทน adc.begin() เพราะ begin() ทำ
// pinMode()/digitalWrite() ซ้ำทุกครั้งโดยไม่จำเป็น (ขา clk/mosi/miso/CS ทั้งสองถูกตั้งค่า
// ไว้แล้วครั้งเดียวใน RobotSetup()) setCS() แค่สลับตัวแปรภายใน ไม่มี I/O overhead
uint16_t read_sensorA(int sensor) {
  if (sensor < 0 || sensor > 7) return 0;
  adc.setCS(17);
  return adc.readADC(sensor);
}

uint16_t read_sensorB(int sensor) {
  if (sensor < 0 || sensor > 7) return 0;
  adc.setCS(13);
  return adc.readADC(sensor);
}


void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  for (int i = 0; i < dataLength; i++) Wire.write(data[i]);
  Wire.endTransmission();
  delay(5);
}

void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));
  Wire.write((int)(eeAddress & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, dataLength);
  for (int i = 0; i < dataLength; i++) {
    if (Wire.available()) buffer[i] = Wire.read();
  }
}

// ==================== get_maxmin_A (เดิมของคุณ ใช้ได้อยู่แล้ว) ====================
void get_maxmin_A() {
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesA[sensor][sample] = read_sensorA(sensor);
      delay(1);
    }
  }
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxA[sensor] = sensorValuesA[sensor][0];
    sensorMinA[sensor] = sensorValuesA[sensor][0];
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesA[sensor][sample];
      if (value > sensorMaxA[sensor]) sensorMaxA[sensor] = value;
      if (value < sensorMinA[sensor]) sensorMinA[sensor] = value;
    }
  }
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 0, buffer, 16);
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 16, buffer, 16);

  tone(32, 950, 100); delay(200);
  tone(32, 950, 200); delay(200);
}

// ==================== get_maxmin_B (เหมือน A) ====================
void get_maxmin_B() {
  // เหมือน get_maxmin_A แต่ใช้ sensorMaxB/sensorMinB และ address 32, 48
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesB[sensor][sample] = read_sensorB(sensor);
      delay(1);
    }
  }
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxB[sensor] = sensorValuesB[sensor][0];
    sensorMinB[sensor] = sensorValuesB[sensor][0];
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesB[sensor][sample];
      if (value > sensorMaxB[sensor]) sensorMaxB[sensor] = value;
      if (value < sensorMinB[sensor]) sensorMinB[sensor] = value;
    }
  }
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 32, buffer, 16);
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinB[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinB[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 48, buffer, 16);

  tone(32, 950, 100); delay(200);
  tone(32, 950, 200); delay(200);
}

// ==================== get_maxmin_C (ของคุณเดิม + constrain) ====================
void get_maxmin_C() {
  const int PIN_C0 = 46;
  const int PIN_C1 = 47;

  for (int sample = 0; sample < numSamples; sample++) {
    sensorValuesC[0][sample] = analogRead(PIN_C0);
    sensorValuesC[1][sample] = analogRead(PIN_C1);
    delay(5);
  }

  for (int sensor = 0; sensor < 2; sensor++) {
    sensorMaxC[sensor] = 0;
    sensorMinC[sensor] = 4095;
    for (int sample = 0; sample < numSamples; sample++) {
      uint16_t value = sensorValuesC[sensor][sample];
      if (value > sensorMaxC[sensor]) sensorMaxC[sensor] = value;
      if (value < sensorMinC[sensor]) sensorMinC[sensor] = value;
    }
    sensorMaxC[sensor] = constrain(sensorMaxC[sensor], 0, 4000);
    sensorMinC[sensor] = constrain(sensorMinC[sensor], 0, 4000);
  }

  uint8_t buffer[4];
  for (int i = 0; i < 2; i++) {
    buffer[i*2]     = highByte(sensorMaxC[i]);
    buffer[i*2 + 1] = lowByte(sensorMaxC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 64, buffer, 4);

  for (int i = 0; i < 2; i++) {
    buffer[i*2]     = highByte(sensorMinC[i]);
    buffer[i*2 + 1] = lowByte(sensorMinC[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 68, buffer, 4);

  tone(32, 1200, 150); delay(200);
  tone(32, 1500, 200); delay(250);
  noTone(32);
}

// ==================== read_eepA (แก้ให้โหลดเข้า sensorMaxA/sensorMinA จริง) ====================
void read_eepA() {
  byte readBuffer[16];
  int readMaxA[numSensors], readMinA[numSensors];
  readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor A Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(readMaxA[sensor]); Serial.print(", Min: "); Serial.println(readMinA[sensor]);
    sensorMaxA[sensor] = readMaxA[sensor];   // โหลดเข้า sensorMaxA จริง
    sensorMinA[sensor] = readMinA[sensor];   // โหลดเข้า sensorMinA จริง
  }   
}

// ==================== read_eepB (เหมือนกัน) ====================
void read_eepB() {
  byte readBuffer[16];
  int readMaxB[numSensors], readMinB[numSensors];
  readEEPROM(EEPROM_ADDRESS, 32, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 48, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinB[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor B Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(readMaxB[sensor]); Serial.print(", Min: "); Serial.println(readMinB[sensor]);
    sensorMaxB[sensor] = readMaxB[sensor];   // โหลดเข้า sensorMaxB จริง
    sensorMinB[sensor] = readMinB[sensor];   // โหลดเข้า sensorMinB จริง
  }
}

// ==================== read_eepC (แก้แล้ว) ====================
void read_eepC() {
  byte readBuffer[4];
  int readMaxC[2], readMinC[2];
  readEEPROM(EEPROM_ADDRESS, 64, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMaxC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 68, readBuffer, 4);
  for (int i = 0; i < 2; i++) {
    readMinC[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor C Values read from EEPROM:");
  Serial.print("Sensor C0 (Pin 46) => Max: "); Serial.print(readMaxC[0]); Serial.print(", Min: "); Serial.println(readMinC[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: "); Serial.print(readMaxC[1]); Serial.print(", Min: "); Serial.println(readMinC[1]);

  sensorMaxC[0] = readMaxC[0]; sensorMinC[0] = readMinC[0];   // โหลดเข้า sensorMaxC/sensorMinC จริง
  sensorMaxC[1] = readMaxC[1]; sensorMinC[1] = readMinC[1];
}

// ==================== แสดงผล (ใช้ตัวแปรเดียวกัน) ====================
void read_sensorA_program() {
  Serial.println("Sensor MAX A Values read from program:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(sensorMaxA[sensor]); Serial.print(", Min: "); Serial.println(sensorMinA[sensor]);
  }   
}

void read_sensorB_program() {
  Serial.println("Sensor MAX B Values read from program:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor "); Serial.print(sensor); Serial.print(" => Max: "); Serial.print(sensorMaxB[sensor]); Serial.print(", Min: "); Serial.println(sensorMinB[sensor]);
  }   
}

void read_sensorC_program() {
  Serial.println("Sensor C Values read from program:");
  Serial.print("Sensor C0 (Pin 46) => Max: "); Serial.print(sensorMaxC[0]); Serial.print(", Min: "); Serial.println(sensorMinC[0]);
  Serial.print("Sensor C1 (Pin 47) => Max: "); Serial.print(sensorMaxC[1]); Serial.print(", Min: "); Serial.println(sensorMinC[1]);
}

void get_EEP_Program() {
  read_eepA();
  read_eepB();
  read_eepC();
  read_sensorA_program();
  read_sensorB_program();
  read_sensorC_program();
}

void ReadF() {
for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = read_sensorA(F_PIN[i]);
  }
}

void ReadB() {
    for (int i = 0; i < NUM_SENSORS; i++) {
    B[i] = read_sensorB(B_PIN[i]);
  }

}



void ReadC() {
  for (int i = 0; i < 2; i++) {  
  C[i] = analogRead(C_PIN[i]); 
 }
}

void SetAnalogDistance(int x){
  DIST = x;
}

void TrackLineColor(int Col) {
  LineColor = Col;
}

void clampSensorValueF(int x, int y){
  FRONT_MIN = x;
  FRONT_MAX = y;
}

void clampSensorValueC(int x, int y){
  CENTER_MIN = x;
  CENTER_MAX = y;
}
void clampSensorValueB(int x, int y){
  BACK_MIN = x;
  BACK_MAX = y;
}

void ReadCalibrateF() {
  ReadF();
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = constrain(F[i], sensorMinA[i], sensorMaxA[i]);
    int16_t x;
    if (LineColor == 0)
      x = map(F[i], sensorMinA[i], sensorMaxA[i], 1000, 0);
    else
      x = map(F[i], sensorMinA[i], sensorMaxA[i], 0, 1000);
    if (x < FRONT_MIN)   x = 0;
    if (x > FRONT_MAX)  x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    F[i] = x;
  }


}

void ReadCalibrateC() {
  ReadC();
  
  for (int i = 0; i < 2; i++) {
    C[i] = constrain(C[i], sensorMinC[i], sensorMaxC[i]);
    int16_t x;
    if (LineColor == 0)
      x = map(C[i], sensorMinC[i], sensorMaxC[i], 1000, 0);
    else
      x = map(C[i], sensorMinC[i], sensorMaxC[i], 0, 1000);
    if (x < CENTER_MIN)   x = 0;
    if (x > CENTER_MAX)  x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    C[i] = x;
  }
  
}

void ReadCalibrateB() {
  ReadB();

  for (int i = 0; i < NUM_SENSORS; i++) {
    B[i] = constrain(B[i], sensorMinB[B_PIN[i]], sensorMaxB[B_PIN[i]]);
    int16_t x;
    if (LineColor == 0)
      x = map(B[i], sensorMinB[B_PIN[i]], sensorMaxB[B_PIN[i]], 1000, 0);
    else
      x = map(B[i], sensorMinB[B_PIN[i]], sensorMaxB[B_PIN[i]], 0, 1000);
    if (x < BACK_MIN)   x = 0;
    if (x > BACK_MAX)  x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    B[i] = x;
  }
  
}


void ReadSensor() {
  ReadCalibrateF();
  ReadCalibrateB();
  ReadCalibrateC();
}
void ReadSensorRaw() {
  ReadF();
  ReadB();
  ReadC();
}

void RefLineValue(int x) {
  Ref = x;
}

void RefCenterLineValue(int x) {
  RefC = x;
}





///////////////////////////////////////////////////////////////////////////////////////

void Serial_FrontSensor() {
  while (1) {
    ReadSensorRaw();
    for (int _serialF = 0; _serialF < NUM_SENSORS; _serialF++) {
      Serial.print(F[_serialF]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
  }
}

void Serial_BackSensor() {
  while (1) {
    ReadSensorRaw();
    for (int _serialB = 0; _serialB < NUM_SENSORS; _serialB++) {
      Serial.print(B[_serialB]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
  }
}

void Serial_CenterSensor() {
  while (1) {
    ReadSensorRaw();
    for (int _serialA = 0; _serialA < 2; _serialA++) {
      Serial.print(C[_serialA]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
  }
}

void SerialCalibrate_FrontSensor() {
  while (1) {
    ReadSensor();
    for (int _serialF = 0; _serialF < NUM_SENSORS; _serialF++) {
      Serial.print(F[_serialF]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
  }
}

void SerialCalibrate_BackSensor() {
  while (1) {
    ReadSensor();
    for (int _serialB = 0; _serialB < NUM_SENSORS; _serialB++) {
      Serial.print(B[_serialB]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
  }
}

void SerialCalibrate_CenterSensor() {
  while (1) {
    ReadSensor();
    for (int _serialC = 0; _serialC < 2; _serialC++) {
      Serial.print(C[_serialC]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
  }
}


void Serial_AllSensor() {
  while (1) {
    ReadSensorRaw();

    Serial.print("F: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }

    Serial.print("B: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }

    Serial.print("C: ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }

    Serial.println();
    delay(50);
  }
}


void SerialCalibrate_AllSensor() {
  while (1) {
    ReadSensor();

    Serial.print("F : ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
   
    }

    Serial.print("B : ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
   
    }

    Serial.print("C : ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }

    Serial.println();
    delay(50);
  }
}

void SerialDistance() {
  while (1) {
    Serial.print("ADC : ");
    Serial.println(analogRead(DIST));
    delay(100);
  }
}

void RGB(){


   if(led == 'b')
      {
        digitalWrite(rgb[0],1);
        digitalWrite(rgb[1],0);
        digitalWrite(rgb[2],0);
        led = 'g';
      }
    else if(led == 'g')
      {
        digitalWrite(rgb[0],0);
        digitalWrite(rgb[1],1);
        digitalWrite(rgb[2],0);
        led = 'r';
      }
    else if(led == 'r')
      {
        digitalWrite(rgb[0],0);
        digitalWrite(rgb[1],0);
        digitalWrite(rgb[2],1);
        led = 'w';
      }
    else if(led == 'w')
      {
        digitalWrite(rgb[0],1);
        digitalWrite(rgb[1],1);
        digitalWrite(rgb[2],1);
        led = 'b';
      }
}

#endif