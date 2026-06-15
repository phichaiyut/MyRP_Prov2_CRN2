#define NUM_SENSORS 8



int F[NUM_SENSORS], B[NUM_SENSORS], C[2];

uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7 };
uint8_t B_PIN[NUM_SENSORS] = { 7, 6, 5, 4, 3, 2, 1, 0 };
uint8_t C_PIN[2] = {46, 47};

#define CCL 1
#define CCR 0
int Ref = 0;
int RefC = 0;
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
