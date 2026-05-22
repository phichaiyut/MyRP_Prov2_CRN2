
//  void resetAngles();
//  float gyroZ();

// void MyRP_Pico2::resetAngles() {
//     my.resetAngles();
// }

// float MyRP_Pico2::gyroZ() {
//     return my.gyro('z');   // หรือ my.getGyroZ()
//}


float current_degree = 0;
float power_factor = 1.0;
float previous_errorG = 0;
float previous_errorGB = 0;

/* ---------- sensor setup ---------- */

// void resetAngles() {
//   robot.resetAngles();
// }

// void setAngleOffset() {
//   robot.resetAngles();
// }


float gyroZ(){
    return my.gyro('z'); 
}

void SetRobotAngle() {
  current_degree = gyroZ();
}

float kpHold = 2.5;
float kdHold = 1.2;
float kpFHold = 4.5;
float kdFHold = 9.0;
float kpBHold = 2.5;
float kdBHold = 1.2;
float holdAngle = 0;
float prevErrHold = 0;
float prevErrHoldB = 0;

void HoldAngle() {
  float error = current_degree - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHold;
  int power = (error * kpHold) + (d * kdHold);
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(power, -power);   // หมุนอยู่กับที่
  prevErrHold = error;
}

void HoldAngleB() {
  float error = current_degree - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldB;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(-power, power);   // หมุนอยู่กับที่
  prevErrHoldB = error;
}

void SetHoldAngle() {
  holdAngle = gyroZ();   // มุมที่ต้องการให้หุ่น "จำ"
  MotorStop();
  prevErrHold = 0;
}

void HoldAngleF() {
  float error = holdAngle - gyroZ();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldB;
  int power = (error * kpFHold) + (d * kdFHold);
  power = constrain(power, -10, 10);   // แรงหมุน
  Motor(power, -power);   // หมุนอยู่กับที่
  prevErrHoldB = error;
}

void SetFG(int totalTime) {
BZon();
SetHoldAngle() ;
  unsigned long endTime = millis() + totalTime;
   while (millis() <= endTime) {
    HoldAngleF();
  }
  BZoff();
}

void setfg(int totalTime) {
  SetFG(totalTime);
}

void SetG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
   while (millis() <= endTime) {
    HoldAngle();
  }
  BZoff();
}


void SetGB(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
   while (millis() <= endTime) {
    HoldAngleB();
  }
  BZoff();
}

/* ---------- spin / turn ---------- */

void spinDegree(int Speed, int relative_degree) {
  int min_speed = 10;
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;
  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -min_speed);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      Motor(pd_value, -pd_value);
    }

    previous_error = error;
  }

  SetG(100);
}

void turnDegree(int Speed, int relative_degree) {
  int min_speed = 10;
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -1);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-1, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) Motor(-1, -pd_value);
      else Motor(pd_value, 1);
    }

    previous_error = error;
  }

  SetG(100);
}

void turnDegreeB(int Speed, int relative_degree) {
  int min_speed = 10;
  int max_speed = Speed;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = gyroZ() + relative_degree;

  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = gyroZ();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    

    int pd_value = (kp * error) + (kd * (error - previous_error));

    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;

    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(1, -min_speed);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, 1);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) Motor(pd_value, 1);
      else Motor(-1, -pd_value);
    }

    previous_error = error;
  }

  SetG(100);
}

void spinDegree(int relative_degree) {
  spinDegree(30, relative_degree);
}

void turnDegree(int relative_degree) {
  turnDegree(30, relative_degree);
}

void turnDegreeB(int relative_degree) {
  turnDegreeB(30, relative_degree);
}

/* ---------- gyro-guided straight move ---------- */

float kpG = 2.5;
float kdG = 1.5;
float kpGB = 2.5;
float kdGB = 1.5;

void RunG(int Speed) {
  float error = current_degree - gyroZ();

  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;

  float derivative = error - previous_errorG;
  int pd_value = (int)((error * kpG) + (derivative * kdG));
  int leftPow = Speed + pd_value;
  int rightPow = Speed - pd_value;

  if (leftPow > Speed) leftPow = Speed;
  if (leftPow < 0) leftPow = 0;
  if (rightPow > Speed) rightPow = Speed;
  if (rightPow < 0) rightPow = 0;

  Motor(leftPow, rightPow);
  previous_errorG = error;
}

void RunGB(int Speed) {
  float error = current_degree - gyroZ();
  if (error > 180.0f) error -= 360.0f;
  else if (error < -180.0f) error += 360.0f;
  float derivative = error - previous_errorGB;
  int pd_value = (int)((error * kpGB) + (derivative * kdGB));
  int leftPow = Speed - pd_value;
  int rightPow = Speed + pd_value;

  if (leftPow > Speed) leftPow = Speed;
  if (leftPow < 0) leftPow = 0;
  if (rightPow > Speed) rightPow = Speed;
  if (rightPow < 0) rightPow = 0;

  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
}



void FFtimerG(int Speed, int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(Speed);
  }
}
void BBtimerG(int Speed, int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(Speed);
  }
}

/* ---------- distance motion (gyro straight) ---------- */

void FFcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.5;

  unsigned long prevT = millis();

  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunG(Speed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    delayMicroseconds(80);
  }
}

void FFcmG(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(LeftBaseSpeed), abs(RightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  float speed_scale = 0.99;
  bool enableRamp = (distance_cm >= 30.0);

  if (!enableRamp) {
    speed_scale = 1.5;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.7f) break;

    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    RunG(target_speed);
  }
}

void BBcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.5;


  unsigned long prevT = millis();

  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunGB(Speed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
  }
}

void BBcmG(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(BackLeftBaseSpeed), abs(BackRightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  float speed_scale = 0.99;
  bool enableRamp = (distance_cm >= 30.0);

  if (!enableRamp) {
    speed_scale = 1.5;
  }

  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.8f) break;

    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    RunGB(target_speed);
  }
}

/* ---------- spin / turn helpers ---------- */

void SpinLG(int Angle) {spinDegree(-abs(Angle));}

void SpinRG(int Angle) { spinDegree(abs(Angle));}

void TurnLG(int Angle) {turnDegree(-abs(Angle));}

void TurnRG(int Angle) {turnDegree(abs(Angle));}

void TurnLBG(int Angle) { turnDegreeB(abs(Angle));}

void TurnRBG(int Angle) {turnDegreeB(-abs(Angle));}

void SpinLG(int spd, int Angle) {spinDegree(spd, -abs(Angle));}

void SpinRG(int spd, int Angle) {spinDegree(spd, abs(Angle));}

void TurnLG(int spd, int Angle) {turnDegree(spd, -abs(Angle));}

void TurnRG(int spd, int Angle) {turnDegree(spd, abs(Angle));}

void TurnLBG(int spd, int Angle) { turnDegreeB(spd, abs(Angle));}

void TurnRBG(int spd, int Angle) {turnDegreeB(spd, -abs(Angle));}

void SLG(int Angle) {spinDegree(-abs(Angle));}

void SRG(int Angle) {spinDegree(abs(Angle));}

void TLG(int Angle) {turnDegree(-abs(Angle));}

void TRG(int Angle) {turnDegree(abs(Angle));}

void TLBG(int Angle) {turnDegreeB(abs(Angle));}

void TRBG(int Angle) { turnDegreeB(-abs(Angle));}

void SLG(int spd, int Angle) {spinDegree(spd, -abs(Angle));}

void SRG(int spd, int Angle) {spinDegree(spd, abs(Angle));}

void TLG(int spd, int Angle) {turnDegree(spd, -abs(Angle));}

void TRG(int spd, int Angle) {turnDegree(spd, abs(Angle));}

void TLBG(int spd, int Angle) {turnDegreeB(spd, abs(Angle));}

void TRBG(int spd, int Angle) {turnDegreeB(spd, -abs(Angle));}



void ToCenterLG() {
   BZon();
    for(int i = 0; i < 20; i++) {
     RunG(tct);
     }
    while (1) {
      RunG(tct);
      ReadCalibrateC();
      if (C[CCL] >= RefC) {
        Motor(-tct, -tct);
        delay(5);
        MotorStop();
        BZoff();
        break;
      }
    }
}
void ToCenterRG() {
   BZon();
    for(int i = 0; i < 20; i++) {
     RunG(tct);
     }
    while (1) {
      RunG(tct);
      ReadCalibrateC();
      if (C[CCR] >= RefC) {
        Motor(-tct, -tct);
        delay(5);
        MotorStop();
        BZoff();
        break;
      }
    }
}

void BackCenterG() {
  BZon();
  for(int i = 0; i < 20; i++) {
     RunGB(bct);
     }
  while (1) {
    RunGB(bct);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

/* ---------- track select (gyro) ---------- */

void TrackSelectG(int spd, char select) {
  if (select == 'l') {
    spinDegree(-90);
  } else if(select == 'L') {
    ToCenterRG();
    spinDegree(-90);
  } else if ( select == 'r') {
    spinDegree(90);
  }else if ( select == 'R') {
    ToCenterRG();
    spinDegree(90);
  } 
   else if (select == 'q' || select == 'Q') {
    turnDegree(-90);
  }else if (select == 'e' || select == 'E') {
    turnDegree(90);
  } else if (select == 'p' || select == 'P') {
    BZon();
    ReadCalibrateF();
    while (1) {
      RunG(spd);
      ReadCalibrateF();
      if (F[0] < Ref && F[7] < Ref) break;
    }
    FFtimerG(spd, 5);
    while (1) {
      RunG(spd);
      ReadCalibrateF();
      if (F[0] < Ref && F[7] < Ref) break;
    }
    BZoff();
  } else if (select == 'c' || select == 'C') {
    BZon();
    for(int i = 0; i < 20; i++) {
     RunG(spd);
     }
    while (1) {
      RunG(spd);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(-spd, -spd);
        delay(5);
        MotorStop();
        BZoff();
        break;
      }
    }
  }
  SetG(100);
}

void TrackSelectGB(int spd, char select) {
  if ( select == 'l') {
    spinDegree(-90);
  }else if (select == 'L' ) {
    BackCenterG();
    spinDegree(-90);
  }
   else if ( select == 'r') {
    spinDegree(90);
  }else if (select == 'R' ) {
    BackCenterG();
    spinDegree(90);
  } else if (select == 'q' || select == 'Q') {
    turnDegreeB(90);
  } else if (select == 'e' || select == 'E') {
    turnDegreeB(-90);
  } else if (select == 'p' || select == 'P') {
    BZon();
    ReadCalibrateB();
    while (1) {
      RunGB(spd);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) break;
    }
    BBtimerG(spd, 5);
    while (1) {
      RunGB(spd);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) break;
    }
     BZoff();
  } else if (select == 'c' || select == 'C') {
    BZon();
    for(int i = 0; i < 20; i++) {
     RunGB(spd);
     }
    while (1) {
      RunGB(spd);
      ReadCalibrateC();
      if (C[CCL] >= RefC || C[CCR] >= RefC) {
        Motor(spd, spd);
        delay(5);
        MotorStop();
        BZoff();
        break;
      }
    }
  }else{
    SetGB(100);
  }
  
}

void FFBG(int Speed, char select) {
  while (1) {
    RunG(Speed);
    ReadCalibrateF();
    if (F[1] > Ref || F[2] > Ref || F[3] > Ref || F[4] > Ref || F[5] > Ref || F[6] > Ref) break;
  }
  TrackSelectG(Speed, select);
}

void BBBG(int Speed, char select) {
  while (1) {
    RunGB(Speed);
    ReadCalibrateB();
    if (B[1] > Ref || B[2] > Ref || B[3] > Ref || B[4] > Ref || B[5] > Ref || B[6] > Ref) break;
  }
  TrackSelectGB(Speed, select);
}

void FFdG(int Speed, char select, float distance_cm) {
  while (1) {
    RunG(Speed);
    if (analogRead(DIST) >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void BBdG(int Speed, char select, float distance_cm) {
  while (1) {
    RunG(Speed);
    if (analogRead(DIST) <= distance_cm) break;
  }
  TrackSelectGB(Speed, select);
}

void FFdGS(int Speed, char select, float distance_cm) {
  while (1) {
    RunG(Speed);
    if (analogRead(DIST) >= distance_cm) break;
  }
  TrackSelectG(Speed, select);
}

void BBdGS(int Speed, char select, float distance_cm) {
  while (1) {
    RunG(Speed);
    if (analogRead(DIST) <= distance_cm) break;
  }
  TrackSelectGB(Speed, select);
}

void FFTG(int Speed, int totalTime) {FFtimerG(Speed, totalTime);}
void BBTG(int Speed, int totalTime) {BBtimerG(Speed, totalTime);}

void FFtG(int Speed, int totalTime) {FFtimerG(Speed, totalTime);}
void BBtG(int Speed, int totalTime) {BBtimerG(Speed, totalTime);}

void FFtimerG(int Speed, int totalTime, char select) {FFtimerG(Speed, totalTime); TrackSelectG(Speed, select);}
void BBtimerG(int Speed, int totalTime, char select) {BBtimerG(Speed, totalTime);TrackSelectGB(Speed, select);}

void FFTG(int Speed, int totalTime, char select) {FFtimerG(Speed, totalTime);TrackSelectG(Speed, select);}
void BBTG(int Speed, int totalTime, char select) {BBtimerG(Speed, totalTime);TrackSelectGB(Speed, select);}

void FFtG(int Speed, int totalTime, char select) {FFtimerG(Speed, totalTime);TrackSelectG(Speed, select);}
void BBtG(int Speed, int totalTime, char select) {BBtimerG(Speed, totalTime);TrackSelectGB(Speed, select);}

void FFcmGS(int Speed, float distance_cm, char select) {FFcmGS(Speed, distance_cm);TrackSelectG(Speed, select);}
void BBcmGS(int Speed, float distance_cm, char select) {BBcmGS(Speed, distance_cm);TrackSelectGB(Speed, select);}

void FFcmG(int Speed, float distance_cm, char select) {FFcmG(Speed, distance_cm);TrackSelectG(Speed, select);}
void BBcmG(int Speed, float distance_cm, char select) {BBcmG(Speed, distance_cm);TrackSelectGB(Speed, select);}

void FFcmGyroS(int Speed, float distance_cm, char select) {FFcmGS(Speed, distance_cm);TrackSelectG(Speed, select);}
void BBcmGyroS(int Speed, float distance_cm, char select) {BBcmGS(Speed, distance_cm);TrackSelectGB(Speed, select);}
void FFcmGyro(int Speed, float distance_cm, char select) {FFcmG(Speed, distance_cm);TrackSelectG(Speed, select);}
void BBcmGyro(int Speed, float distance_cm, char select) {BBcmG(Speed, distance_cm);TrackSelectGB(Speed, select);}


void fftimerg(int Speed, int totalTime) {FFtimerG(Speed, totalTime);}
void bbtimerg(int Speed, int totalTime) {BBtimerG(Speed, totalTime);}
void fftimerg(int Speed, int totalTime, char select) {FFtimerG(Speed, totalTime, select);}
void bbtimerg(int Speed, int totalTime, char select) {BBtimerG(Speed, totalTime, select);}


// ---------- BG ----------
void ffbg(int speed, char select) { FFBG(speed, select);}
void bbbg(int speed, char select) { BBBG(speed, select);}

// ---------- Timer G ----------
void fftg(int speed, int totaltime) {FFTG(speed, totaltime);}
void bbtg(int speed, int totaltime) {BBTG(speed, totaltime);}
void fftg(int speed, int totaltime, char select) {FFTG(speed, totaltime, select);}
void bbtg(int speed, int totaltime, char select) {BBTG(speed, totaltime, select);}

// ---------- cm GS ----------
void ffcmgs(int speed, float distance_cm) {FFcmGS(speed, distance_cm);}
void bbcmgs(int speed, float distance_cm) {BBcmGS(speed, distance_cm);}

// ---------- cm G ----------
void ffcmg(int speed, float distance_cm) {FFcmG(speed, distance_cm);}
void bbcmg(int speed, float distance_cm) {BBcmG(speed, distance_cm);}
// ---------- cm GS ----------
void ffcmgs(int speed, float distance_cm, char select) {FFcmGS(speed, distance_cm, select);}
void bbcmgs(int speed, float distance_cm, char select) {BBcmGS(speed, distance_cm, select);}

// ---------- cm G ----------
void ffcmg(int speed, float distance_cm, char select) {FFcmG(speed, distance_cm, select);}
void bbcmg(int speed, float distance_cm, char select) {BBcmG(speed, distance_cm, select);}

// ---------- cm Gyro ----------
void ffcmgyros(int speed, float distance_cm, char select) {FFcmGyroS(speed, distance_cm, select);}
void bbcmgyros(int speed, float distance_cm, char select) {BBcmGyroS(speed, distance_cm, select);}
void ffcmgyro(int speed, float distance_cm, char select) {FFcmGyro(speed, distance_cm, select);}
void bbcmgyro(int speed, float distance_cm, char select) {BBcmGyro(speed, distance_cm, select);}

// ---------- distance Gyro ----------
void ffdg(int speed, char select, float distance_cm) {FFdG(speed, select, distance_cm);}
void bbdg(int speed, char select, float distance_cm) {BBdG(speed, select, distance_cm);}
void ffdgs(int speed, char select, float distance_cm) {FFdGS(speed, select, distance_cm);}
void bbdgs(int speed, char select, float distance_cm) {BBdGS(speed, select, distance_cm);}
// ---------- Spin ----------
void spinlg(int angle) { SpinLG(angle);}
void spinrg(int angle) { SpinRG(angle);}
void spinlg(int spd, int angle) { SpinLG(spd, angle); }
void spinrg(int spd, int angle) { SpinRG(spd, angle);}

// ---------- Turn ----------
void turnlg(int angle) {TurnLG(angle);}
void turnrg(int angle) {TurnRG(angle);}
void turnlbg(int angle) {TurnLBG(angle); }
void turnrbg(int angle) {TurnRBG(angle);}
void turnlg(int spd, int angle) {TurnLG(spd, angle);}
void turnrg(int spd, int angle) {TurnRG(spd, angle);}
void turnlbg(int spd, int angle) {TurnLBG(spd, angle);}
void turnrbg(int spd, int angle) {TurnRBG(spd, angle);}

// ---------- Short Name (S / T) ----------
void slg(int angle) {SLG(angle);}
void srg(int angle) {SRG(angle);}
void tlg(int angle) {TLG(angle);}
void trg(int angle) {TRG(angle);}
void tlbg(int angle) {TLBG(angle);}
void trbg(int angle) {TRBG(angle);}
void slg(int spd, int angle) {SLG(spd, angle);}
void srg(int spd, int angle) {SRG(spd, angle);}
void tlg(int spd, int angle) {TLG(spd, angle);}
void trg(int spd, int angle) {TRG(spd, angle);}
void tlbg(int spd, int angle) {TLBG(spd, angle);}
void trbg(int spd, int angle) {TRBG(spd, angle);}

void setg(int time) { SetG(time); }
void setgb(int time) { SetGB(time); }