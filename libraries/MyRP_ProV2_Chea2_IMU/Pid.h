// #define CCL 0
// #define CCR 1

float PID_KP, PID_KD;
int LastError_F, LastError_B;
int Error_F = 0, Error_B = 0;
int tct, bct, tspd;
int tctL, tctR, bctL, bctR;
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int set_position = 2500;
int set_positionL = 500;
int set_positionR = 4500;
float slow_kp = 0.005, slow_kd = 0.05;
float slow_kp_f = 0.005, slow_kd_f = 0.05;
float slow_kp_b = 0.005, slow_kd_b = 0.05;
float slow_kpf = 0.005, slow_kdf = 0.05;
float slow_kpb = 0.005, slow_kdb = 0.05;
int line_centor = 0;

int MaxSpeed = 100;
int MinSpeed = -5;
int ModePidStatus = 0;
int setsensortracklineL = 0, setsensortracklineR = 7;
int dottedline = 0;
// ---------- Config ----------
void SetFG(int time);

void SetRobotPID(float Kp, float Kd) {
  PID_KP = Kp;
  PID_KD = Kd;
}

void SetToCenterSpeed(int tctv) {
  tct = tctv;
  bct = tctv;
  BaseSpeed = tctv;
  InitialSpeed();
  // Front / back base speed
  tctL = LeftBaseSpeed;
  tctR = RightBaseSpeed;
  bctL = BackLeftBaseSpeed;
  bctR = BackRightBaseSpeed;

  // // Slow PID
  slow_kpf = PID_KP_Front;
  slow_kdf = PID_KD_Front;
  slow_kpb = PID_KP_Back;
  slow_kdb = PID_KD_Back;
}
void set_slow_kp_kd(float kp_f, float kd_f, float kp_b, float kd_b) {
  slow_kp_f = kp_f;
  slow_kd_f = kd_f;
  slow_kp_b = kp_b;
  slow_kd_b = kd_b;
}

void Dottedline(int x) {
  dottedline = x;
}

void SetTurnSpeed(int tspdv) {
  tspd = tspdv;
}

void TurnSpeedLeft(int l, int r, int de) {
  LTurnSpdL  = l;
  LTurnSpdR  = r;
  TurnDelayL = de;
}

void TurnSpeedRight(int l, int r, int de) {
  RTurnSpdL  = l;
  RTurnSpdR  = r;
  TurnDelayR = de;
}

void ModeSpdPID(int moD, int maX, int miN) {
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}

void set_position_line(int _pos) {
  if (_pos < 0) {
    set_position = 0;
  } else if (_pos > 5000) {
    set_position = 5000;
  } else {
    set_position = _pos;
  }
}

void set_position_line_l(int _pos) {
  
    if (_pos < 0) {
    set_positionL = 0;
  } else if (_pos > 5000) {
    set_positionL = 5000;
  } else {
    set_positionL = _pos;
  }
  
}

void set_position_line_r(int _pos) {
  
    if (_pos < 0) {
    set_positionR = 0;
  } else if (_pos > 5000) {
    set_positionR = 5000;
  } else {
    set_positionR = _pos;
  }
  
}

void set_sensor_track_line(int L, int R) {
  setsensortracklineL = L;
  setsensortracklineR = R;
}

void set_line_center(int x) {
  line_centor = x;
}
// ---------- Position Reading ----------


int readPositionF(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateF();
  int S[6] = {F[1], F[2], F[3], F[4], F[5], F[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (dottedline) {
      return last_value;
    }
    if (last_value < set_position) return 0 * 1000;
    else return 5 * 1000;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateB();
  int S[6] = {B[1], B[2], B[3], B[4], B[5], B[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (dottedline) {
      return last_value;
    }
    if (last_value < set_position) return 0 * 1000;
    else return 5 * 1000;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionF_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateF();
  int S[6] = {F[1], F[2], F[3], F[4], F[5], F[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (dottedline) {
      return last_value;
    }
    if (last_value < (6 - 1) * 1000 / 2) return 2500;
    else return 2500;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = ((6 - 1) * 1000) / 2;
  ReadCalibrateB();
  int S[6] = {B[1], B[2], B[3], B[4], B[5], B[6]};
  for (i = 0; i < 6; i++) {
    int values = S[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000L);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (6 - 1) * 1000 / 2) return 2500;
    else return 2500;
  }
  last_value = avg / sum;
  return last_value;
}

// ---------- PID ----------

void PIDF(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos = readPositionF(200, 50);
  float Error = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F = Error;
  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  // if (leftPow  > 100) leftPow  = 100;
  // if (leftPow  < 0)   leftPow  = -5;
  // if (rightPow > 100) rightPow = 100;
  // if (rightPow < 0)   rightPow = -5;
  switch (ModePidStatus) {
  case 0:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = MinSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = MinSpeed;
    break;
  case 1:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < MinSpeed) LeftPower = MinSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < MinSpeed) RightPower = MinSpeed;
    break;
  case 2:
    if (LeftPower > SpeedL) LeftPower = SpeedL;
    if (LeftPower < -SpeedL) LeftPower = -SpeedL;
    if (RightPower > SpeedR) RightPower = SpeedR;
    if (RightPower < -SpeedR) RightPower = -SpeedR;
    break;
  case 3:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = -BaseSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = -BaseSpeed;
    break;
  default:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = 0;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = 0;
  }
  Motor(LeftPower, RightPower);
}

void PIDB(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos      = readPositionB(200, 50);
  float Error    = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B  = Error;
  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
  // if (leftPow  > 100) leftPow  = 100;
  // if (leftPow  < 0)   leftPow  = -5;
  // if (rightPow > 100) rightPow = 100;
  // if (rightPow < 0)   rightPow = -5;
  switch (ModePidStatus) {
  case 0:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = MinSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = MinSpeed;
    break;
  case 1:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < MinSpeed) LeftPower = MinSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < MinSpeed) RightPower = MinSpeed;
    break;
  case 2:
    if (LeftPower > SpeedL) LeftPower = SpeedL;
    if (LeftPower < -SpeedL) LeftPower = -SpeedL;
    if (RightPower > SpeedR) RightPower = SpeedR;
    if (RightPower < -SpeedR) RightPower = -SpeedR;
    break;
  case 3:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = -BaseSpeed;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = -BaseSpeed;
    break;
  default:
    if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
    if (LeftPower < 0) LeftPower = 0;
    if (RightPower > MaxSpeed) RightPower = MaxSpeed;
    if (RightPower < 0) RightPower = 0;
  }
  Motor(-LeftPower, -RightPower);
}


static bool frontCenterLine() {
  return (F[2] > Ref && F[3] > Ref)
      || (F[3] > Ref && F[4] > Ref)
      || (F[4] > Ref && F[5] > Ref);
}

void PIDF_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  ReadCalibrateF();
  float Pos;
  if (frontCenterLine()) {
    Pos = 2500;   // 2500
  } else {
    Pos = readPositionF_none(200, 50);
  }

  float Error = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F = Error;

  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;

  LeftPower  = constrain(LeftPower, -100, 100);
  RightPower = constrain(RightPower, -100, 100);

  Motor(LeftPower, RightPower);
}

static bool backCenterLine() {
  return (B[2] > Ref && B[3] > Ref)
      || (B[3] > Ref && B[4] > Ref)
      || (B[4] > Ref && B[5] > Ref);
}
void PIDB_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  float Pos;
  ReadCalibrateB();
  if (backCenterLine()) {
    Pos = 2500;
  } else {
    Pos = readPositionB_none(200, 50);
  }
  // float Pos      = readPositionB_none(250, 50);
  float Error    = Pos - set_position;
  float PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B  = Error;
  float LeftPower  = SpeedL + PID_Value;
  float RightPower = SpeedR - PID_Value;
   LeftPower  = constrain(LeftPower, -100, 100);
  RightPower = constrain(RightPower, -100, 100);
  Motor(-LeftPower, -RightPower);
}


// ---------- Timed Motion ----------

void FFtimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
}

void BBtimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
}

void lf(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(0,0, 0.030, 0.60);
}

void lb(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(0,0, 0.030, 0.60);
}



void FFcm(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง
  unsigned long prevT = millis();
  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    delayMicroseconds(80);
  }
}

void BBcm(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง
  unsigned long prevT = millis();
  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;
    
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    delayMicroseconds(80);
  }
}

// ---------- ToCenter / BackCenter ----------
void ModeToCenter() {
  if (line_centor == 0) {
    Motor(tctL, tctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
  }

}

void ModeToCenterBack() {
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
    }
  }

}

void ModeToCenterLine() {
  if (line_centor == 0) {
    Motor(tctL, tctR);
  } else {
    PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
  }

}

void ModeToCenterBackLine() {
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
  } else {
    PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
  }

}

void ToCenter() {
  BZon();
  ModeToCenter();
  //   Motor(tctL, tctR);
  // delay(20);
  while (1) {
    ModeToCenterLine();
    //  Motor(tctL, tctR);
    ReadSensor();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterL() {
  BZon();
  ModeToCenter();
  // Motor(tctL, tctR);
  // delay(20);
  while (1) {
    ModeToCenterLine();
    // Motor(tctL, tctR);
    ReadSensor();
    if (C[CCL] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterR() {
  BZon();
  ModeToCenter();
  // Motor(tctL, tctR);
  // delay(20);
  while (1) {
    // Motor(tctL, tctR);
    ModeToCenterLine();
    ReadSensor();
    if (C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void BackCenter() {
  BZon();
  ModeToCenterBack();
  // Motor(-bctL, -bctR);
  // delay(20);
  while (1) {
    // Motor(-bctL, -bctR);
    ModeToCenterBackLine();
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

// ---------- Turns / Spins ----------

void TurnLeft() {
  Motor(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    Motor(-LTurnSpdL, LTurnSpdR);
    ReadCalibrateF();
    if (F[2] >= Ref) { MotorStop(); break; }
  }
}

void TurnRight() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrateF();
    if (F[5] >= Ref) { MotorStop(); break; }
  }
}

void SpinL(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[2] <= Ref) {
      break;
    }
  }

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[2] >= Ref) {
      // Motor(speed, -speed);
      // delay(5);
      lf(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinL() {
  SpinL(tspd);
}

void SpinL2(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[2] >= Ref) break;
  }

  Motor(-speed, speed);
  delay(30);

  while (1) {
    ReadCalibrateF();
    Motor(-speed, speed);
    if (F[2] >= Ref) {
      // Motor(speed, -speed);
      // delay(5);
      lf(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinL2() {
  SpinL2(tspd);
}


void SpinR(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);

  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[5] <= Ref) {
      break;
    }
  }

  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[5] >= Ref) {
      // Motor(-speed, speed);
      // delay(5);
      lf(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinR() {
  SpinR(tspd);
}

void SpinR2(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[5] >= Ref) break;
  }
  Motor(speed, -speed);
  delay(30);
  while (1) {
    ReadCalibrateF();
    Motor(speed, -speed);
    if (F[5] >= Ref) {
      // Motor(-speed, speed);
      // delay(5);
      lf(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinR2() {
  SpinR2(tspd);
}



// ==================== Back Sensor ====================

void TurnLeft_B() {
  Motor(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    Motor(-LTurnSpdL, LTurnSpdR);
    ReadCalibrateB();
    if (B[5] >= Ref) {
      //MotorStop();
      break;
    }
  }
}

void TurnRight_B() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrateB();

    if (B[2] >= Ref) {
      //MotorStop();
      break;
    }
  }
}

// ==================== Spin Left Back ====================

void SpinL_B(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[5] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);

    if (B[5] >= Ref) {
      // Motor(speed, -speed);
      // delay(5);
      lb(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinL_B() {
  SpinL_B(tspd);
}

// ==================== Spin Left 2 Back ====================

void SpinL2_B(int speed) {
  MotorStop();
  delay(10);
  Motor(-speed, speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[5] >= Ref) break;
  }
  Motor(-speed, speed);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(-speed, speed);
    if (B[5] >= Ref) {
      // Motor(speed, -speed);
      // delay(5);
      lb(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinL2_B() {
  SpinL2_B(tspd);
}

// ==================== Spin Right Back ====================

void SpinR_B(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[2] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[2] >= Ref) {
      // Motor(-speed, speed);
      // delay(5);
      lb(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinR_B() {
  SpinR_B(tspd);
}

// ==================== Spin Right 2 Back ====================

void SpinR2_B(int speed) {
  MotorStop();
  delay(10);
  Motor(speed, -speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[2] >= Ref) break;
  }
  Motor(speed, -speed);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(speed, -speed);
    if (B[2] >= Ref) {
      // Motor(-speed, speed);
      // delay(5);
      lb(tspd);
      MotorStop();
      break;
    }
  }
}

void SpinR2_B() {
  SpinR2_B(tspd);
}


// ---------- Track Select ----------

void TrackSelectF(int spd, char x) {
  if (x == 's' || x == 'S') {
    Motor(-spd, -spd);
    delay(5);
    Move(-15, -15, 5);
    Move(-10, -10, 1);
    Move(-1, -1, 1);
    MotorStop();
  } else if (x == 'p' || x == 'P') {
    BZon();
    ReadCalibrateF();
    while (1) {
      Motor(spd, spd);
      ReadCalibrateF();
      if (F[0] < Ref && F[7] < Ref) break;
    }
    delay(5);
    while (1) {
      Motor(spd, spd);
      ReadCalibrateF();
      if (F[0] < Ref && F[7] < Ref) {
        BZoff();
        break;
      }
    }
  } else if (x == 'l' || x == 'L') {
    ToCenter();
    SpinL();
    //lf(tspd);
  } else if (x == 'r' || x == 'R') {
    ToCenter();
    SpinR();
    // lf(tspd);
  } else if (x == 'q' || x == 'Q') {
     while (1) {
      PIDF(tctL,tctR,slow_kpf,slow_kdf);
      ReadCalibrateF();
      if (F[0] > Ref) break;
    }
    BZon();
    while (1) {
      Motor(tctL/2, tctR/2);
      ReadCalibrateF();
      if (F[0] < Ref ) {
        //delay(20);
        BZoff();
        break;
      }
    }
    TurnLeft();
    //lf(10);
  } else if (x == 'e' || x == 'E') {
     while (1) {
      PIDF(tctL,tctR,slow_kpf,slow_kdf);
      ReadCalibrateF();
      if (F[7] > Ref) break;
    }
    BZon();
    while (1) {
      Motor(tctL/2, tctR/2);
      ReadCalibrateF();
      if ( F[7] < Ref) {
        //delay(20);
        BZoff();
        break;
      }
    }
    TurnRight();
   // lf(10);
  } else if (x == 'c' || x == 'C') {
    ToCenter();
  } else if (x == 'd' || x == 'D') {
    ToCenter();
    SpinR_B();
    //lb(tspd);
  } else if (x == 'a' || x == 'A') {
    ToCenter();
    SpinL_B();
    //lb(tspd);
  } else if (x == 'b' || x == 'B') {
    BZon();
    ModeToCenter();
    // Motor(LeftBaseSpeed, RightBaseSpeed);
    // delay(20);
    while (1) {
      ModeToCenterLine();
      ReadCalibrateB();
      if ((B[0] > Ref || B[7] > Ref)) {
        Motor(-10, -10);
        delay(10);
        Motor(-1, -1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if(x == 'g' || x == 'G') {
    SetFG(100);
  }
  else {
    MotorStop(20);
  }
}

void TrackSelectB(int spd, char x) {
  if (x == 's' || x == 'S') {
    Motor(spd, spd);
    delay(5);
    Move(15, 15, 5);
    Move(10, 10, 1);
    Move(1, 1, 1);
    MotorStop();
  } else if (x == 'p' || x == 'P') {
    BZon();
    ReadCalibrateB();
    while (1) {
      Motor(-spd, -spd);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) break;
    }
    delay(5);
    while (1) {
      Motor(-spd, -spd);
      ReadCalibrateB();
      if (B[0] < Ref && B[7] < Ref) {
        BZoff();
        break;
      }
    }
  } else if (x == 'l' || x == 'L') {
    BackCenter();
    SpinL();
    //lf(tspd);
  } else if (x == 'r' || x == 'R') {
    BackCenter();
    SpinR();
    //lf(tspd);
  } else if (x == 'c' || x == 'C') {
    BackCenter();
  } else if (x == 'd' || x == 'D') {
    BackCenter();
    SpinR_B();
    //lb(tspd);
  } else if (x == 'a' || x == 'A') {
    BackCenter();
    SpinL_B();
    //lb(tspd);
  } else if (x == 'e' || x == 'E') {
     while (1) {
      PIDB(bctL,bctR,slow_kpb,slow_kdb);
      ReadCalibrateB();
      if (B[0] > Ref) break;
    }
    BZon();
    while (1) {
      Motor(-bctL/2, -bctR/2);
      ReadCalibrateB();
      if (B[0] < Ref) {
        //delay(50);
        BZoff();
        break;
      }
    }
    TurnLeft_B();
   //lb(tspd);
  } else if (x == 'q' || x == 'Q') {
     while (1) {
      PIDB(bctL,bctR,slow_kpb,slow_kdb);
      ReadCalibrateB();
      if (B[7] > Ref) break;
    }
    BZon();
    while (1) {
      Motor(-bctL/2, -bctR/2);
      ReadCalibrateB();
      if ( B[7] < Ref) {
        //delay(50);
        BZoff();
        break;
      }
    }
    TurnRight_B();
    //lb(tspd);
  } else if (x == 'b' || x == 'B') {
    BZon();
    // Motor(-BackLeftBaseSpeed, -BackRightBaseSpeed);
    // delay(20);
    ModeToCenterBack();
    while (1) {
      ModeToCenterBackLine();
      ReadCalibrateF();
      if ((F[0] > Ref || F[7] > Ref)) {
        Motor(10, 10);
        delay(10);
        Motor(1, 1);
        delay(1);
        MotorStop();
        BZoff();
        break;
      }
    }
  } else if(x == 'g' || x == 'G') {
    SetFG(100);
  }else {
    MotorStop(20);
  }

}


void FFtimer(int Speed, int totalTime, char select) {
  FFtimer(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void BBtimer(int Speed, int totalTime, char select) {
  BBtimer(Speed, totalTime);
  TrackSelectB(Speed, select);
}
void FFT(int Speed, int totalTime, char select) {
  FFtimer(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void BBT(int Speed, int totalTime, char select) {
  BBtimer(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void FFcm(int Speed, float distance, char select) {
  FFcm(Speed, distance);
  TrackSelectF(Speed, select);
}

void BBcm(int Speed, float distance, char select) {
  BBcm(Speed, distance);
  TrackSelectB(Speed, select);
}

// ---------- FF / BB Patterns ----------
void FF(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref || F[7] > Ref  || (F[2] > Ref && F[5] > Ref)) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}
void BB(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref || B[7] > Ref || (B[2] > Ref && B[5] > Ref)) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}


void FFC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[0] > Ref && F[7] > Ref)||(F[1] > Ref && F[6] > Ref)||(F[2] > Ref && F[3] > Ref&& F[4] > Ref&& F[5] > Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] >= Ref && F[7] >= Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[0] > Ref && B[7] > Ref) || (B[1] > Ref && B[6] > Ref) || (B[2] > Ref && B[3] > Ref && B[4] > Ref && B[5] > Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] >= Ref && B[7] >= Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref ) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}
void FFL0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}

void FFL2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref && F[1] > Ref && F[2] > Ref && F[3] > Ref && F[4] > Ref) {
      break;
    }
  }
  TrackSelectF(Speed, select);
}

void BBL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref ) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}
void BBL0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void BBL2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref && B[1] > Ref && B[2] > Ref && B[3] > Ref && B[4] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void FFR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ( F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFR7(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFR2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[3] > Ref && F[4] > Ref && F[5] > Ref && F[6] > Ref && F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ( B[7] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void BBR7(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[7] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}
void BBR2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[3] > Ref && B[4] > Ref && B[5] > Ref && B[6] > Ref && B[7] > Ref) {
      break;
    }
  }
  TrackSelectB(Speed, select);
}

void FFBlack(int SpeedL, int SpeedR, char select) {
  Move(SpeedL, SpeedR, 50);
  while (1) {
    Motor(SpeedL, SpeedR);
    ReadCalibrateF();
    if (F[1] > Ref || F[2] > Ref || F[3] > Ref || F[4] > Ref || F[5] > Ref || F[6] > Ref) break;
  }
  TrackSelectF(SpeedL, select);
}
void FFB(int SpeedL, int SpeedR, char select) {
  FFBlack(SpeedL, SpeedR, select);
}

void FFBlack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFBlack(LeftBaseSpeed, RightBaseSpeed, select);
}

void FFB(int Speed, char select) {
  FFBlack(Speed, select);
}

void BBBlack(int SpeedL, int SpeedR, char select) {
  Move(-SpeedL, -SpeedR, 50);
  while (1) {
    Motor(-SpeedL, -SpeedR);
    ReadCalibrateB();
    if (B[1] > Ref || B[2] > Ref || B[3] > Ref || B[4] > Ref || B[5] > Ref || B[6] > Ref)  break;
  }
  TrackSelectB(SpeedL, select);
}

void BBB(int SpeedL, int SpeedR, char select) {
  BBBlack(SpeedL, SpeedR, select);
}

void BBBlack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBBlack(BackLeftBaseSpeed, BackRightBaseSpeed, select);
}

void BBB(int Speed, char select) {
  BBBlack(Speed, select);
}

void FFWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[0] < Ref && F[1] < Ref && F[2] < Ref && F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFW(int Speed, char select) { FFWhite(Speed, select); }

void BBWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[0] < Ref && B[1] < Ref&& B[2] < Ref && B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBW(int Speed, char select) { BBWhite(Speed, select); }


void FFNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[numm] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFN(int Speed, char select, int numm) {
  FFNUM(Speed, select, numm);
}
void BBN(int Speed, char select, int numm) {
  BBNUM(Speed, select, numm);
}

void FF_Distance(int Speed, char select, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    if (analogRead(DIST) >= distance) break;
  }
  TrackSelectF(Speed, select);
}

void BB_Distance(int Speed, char select, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    if (analogRead(DIST) <= distance) break;
  }
  TrackSelectB(Speed, select);
}

void FFd(int Speed, char select, float distance) {
  FF_Distance(Speed, select, distance);
}
void BBd(int Speed, char select, float distance) {
  BB_Distance(Speed, select, distance);
}

// ===== Stop PID =====
float PID_KP_STOP = 0.35;
float PID_KD_STOP = 0.8;

int PID_Stop(int ir, int ir_target, int maxSpeed) {
  static int lastError = 0;

  int error = ir_target - ir;
  int derivative = error - lastError;
  lastError = error;

  int out = (PID_KP_STOP * error) +
            (PID_KD_STOP * derivative);

  return constrain(out, 0, maxSpeed);
}

void FF_DistanceS(int Speed, char select, int ir_target) {

  BaseSpeed = Speed;
  InitialSpeed();   // 🔴 จำเป็นมาก

  while (1) {
    ReadCalibrateF();
    int ir = analogRead(DIST);
    // ===== เข้าโหมดหยุด =====
    if (ir >= ir_target) {
      int newBase = PID_Stop(ir, ir_target, Speed);
      // 🔴 BaseSpeed เปลี่ยน ต้อง InitialSpeed()
      if (newBase != BaseSpeed) {
        BaseSpeed = newBase;
        InitialSpeed();
      }
    }
    // ===== PID ตามเส้น =====
    PIDF(LeftBaseSpeed, RightBaseSpeed,
         PID_KP_Front, PID_KD_Front);
    // ===== หยุดจริง =====
    if (BaseSpeed <= 1) break;
  }
  MotorStop();              // เบรกนิ่ง
  TrackSelectF(Speed, select); // คำสั่งถัดไป
}

void FFdS(int Speed, char select, int ir_target) {
  FF_DistanceS(Speed, select, ir_target);
}

// ---------- GoStart / GoEnd ----------

void GoStart(int LeftSpeed, int RightSpeed) {
  Move(LeftSpeed, RightSpeed, 100);
  while (1) {
    ReadCalibrateF();
    Motor(LeftSpeed, RightSpeed);
    if (F[0] < Ref && F[7] < Ref) {
      Move(LeftSpeed, RightSpeed, 1);
      break;
    }
  }
}

void GoStart(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  GoStart(LeftBaseSpeed, RightBaseSpeed);
}

void GoEnd(int LeftSpeed, int RightSpeed) {
  while (1) {
    ReadCalibrateC();
    Motor(LeftSpeed, RightSpeed);
    if (C[0] > RefC && C[1] > RefC) {
      Move(LeftSpeed, RightSpeed, 50);
      break;
    }
  }
  Move(-15, -15, 15);
  Move(-10, -10, 10);
  Move(-1, -1, 1);
  MotorStop();
}

void GoEnd(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  GoEnd(LeftBaseSpeed, RightBaseSpeed);
  MotorStop();
}

// ---------- Balance ----------

void BalanceF(int Counter) {
  Move(-10, -10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(-10, -10, 50);
    while (1) {
      Motor(10, 10);
      ReadCalibrateF();
      if (F[0] > Ref) {
        while (1) {
          Motor(0, 10);
          ReadCalibrateF();
          if (F[7] > Ref) { MotorStop(); break; }
        }
      }
      if (F[7] > Ref) {
        while (1) {
          Motor(10, 0);
          ReadCalibrateF();
          if (F[0] > Ref) { MotorStop(); break; }
        }
      }
      if (F[0] > Ref && F[7] > Ref) { MotorStop(); break; }
    }
    MotorStop();
    delay(50);
  }
}
void SetF(int Counter) {
  BalanceF(Counter);
}

void BalanceB(int Counter) {
  Move(10, 10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(10, 10, 50);
    while (1) {
      Motor(-12, -12);
      ReadCalibrateB();
      if (B[0] > Ref) {
        while (1) {
          Motor(0, -10);
          ReadCalibrateB();
          if (B[7] > Ref) { MotorStop(); break; }
        }
      }
      if (B[7] > Ref) {
        while (1) {
          Motor(-10, 0);
          ReadCalibrateB();
          if (B[0] > Ref) { MotorStop(); break; }
        }
      }
      if (B[0] > Ref && B[7] > Ref) { MotorStop(); break; }
    }
    MotorStop();
    delay(50);
  }
}

void SetB(int Counter) {
  BalanceB(Counter);
}


void set_f(int num) {
  for (int i=0; i< num; i++) {
    while (1) {
      ReadCalibrateF();
      delay(5);
      if (F[0] > Ref && F[7] < Ref) {
        Motor(-5, 15);
      } else if (F[0] < Ref && F[7] > Ref) {
        Motor(15, -5);
      } else if (F[0] < Ref && F[7] < Ref) {
        Motor(15, 15);
      } else {
        Motor(-1, -1);
        break;
      }
    }
    if (num > 1) {
      Motor(-15, -15);
      delay(50);
      Motor(-1, -1);
    }
  }

}

void set_b(int num) {
  for (int i=0; i< num; i++) {
    while (1) {
      ReadCalibrateB();
      delay(5);
      if (B[0] > Ref && B[7] < Ref) {
        Motor(5, -15);
      } else if (B[0] < Ref && B[7] > Ref) {
        Motor(-15, 5);
      } else if (B[0] < Ref && B[7] < Ref) {
        Motor(-15, -15);
      } else {
        Motor(1, 1);
        break;
      }
    }
    if (num > 1) {
      Motor(15, 15);
      delay(50);
      Motor(1, 1);
    }
  }
}


void set_fc(int num) {
  for (int i=0; i< num; i++) {
    while (1) {
      ReadCalibrateC();
      delay(5);
      if (C[1] > RefC && C[0] < RefC) {
        Motor(-5, 15);
      } else if (C[1] < RefC && C[0] > RefC) {
        Motor(15, -5);
      } else if (C[1] < RefC && C[0] < RefC) {
        Motor(15, 15);
      } else {
        Motor(-1, -1);
        break;
      }
    }
    if (num > 1) {
      Motor(-15, -15);
      delay(50);
      Motor(-1, -1);
    }
  }

}

void set_bc(int num) {
  for (int i=0; i< num; i++) {
    while (1) {
      ReadCalibrateC();
      delay(5);
      if (C[1] > RefC && C[0] < RefC) {
        Motor(5, -15);
      } else if (C[1] < RefC && C[0] > RefC) {
        Motor(-15, 5);
      } else if (C[1] < RefC && C[0] < RefC) {
        Motor(-15, -15);
      } else {
        Motor(1, 1);
        break;
      }
    }
    if (num > 1) {
      Motor(15, 15);
      delay(50);
      Motor(1, 1);
    }
  }
}


void SerialPositionF() {
  while (1) {
  int pos = readPositionF(200, 50);
  Serial.print("Position F : ");
  Serial.println(pos);
  delay(100);
  }
}

void SerialPositionB() {
  while (1) {
  int pos = readPositionB(200, 50);
  Serial.print("Position  B : ");
  Serial.println(pos);
  delay(100);
  }
}

void SerialPositionFB() {
  while (1) {
    int posF = readPositionF(200, 50);
    int posB = readPositionB(200, 50);
    Serial.print("Position F : ");
    Serial.print(posF);
    Serial.print("  |  Position B : ");
    Serial.println(posB);
    delay(100);
  }
}

void FFCL(int Speed,char select){
int temp = set_position;
set_position = set_positionL;
 BaseSpeed = Speed;
  InitialSpeed();
  while(1){
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp;
}

void FFCR(int Speed,char select){
int temp = set_position;
set_position = set_positionR;
 BaseSpeed = Speed;
  InitialSpeed();
  while(1){
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[7] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp;
}


//----------------------------------------------------------------------------------------- FFtimerCricleL ----------------------------------------------------------------------------------//
void FFtimerCL(int Speed, int totalTime) {
 int temp = set_position;
  set_position = set_positionL;
  FFtimer(Speed,totalTime);
  set_position = temp;
}
//----------------------------------------------------------------------------------------- FFtimerCricleR ----------------------------------------------------------------------------------//
void FFtimerCR(int Speed, int totalTime) {
 int temp = set_position;
  set_position = set_positionR;
  FFtimer(Speed,totalTime);
  set_position = temp;
}

void FFcmCL(int Speed, int distance){
   int temp = set_position;
  set_position = set_positionL;
  FFcm(Speed,distance);
  set_position = temp;
}

void FFcmCR(int Speed, int distance){
  int temp = set_position;
  set_position = set_positionR;
  FFcm(Speed,distance);
  set_position = temp;
}

//----------------------------------------------------------------------------------------- FFtimerCricleL ----------------------------------------------------------------------------------//
void FFtimerCL(int Speed, int totalTime, char select) {
 int temp = set_position;
set_position = set_positionL;
FFtimer(Speed,totalTime,select);
set_position = temp;
}
//----------------------------------------------------------------------------------------- FFtimerCricleR ----------------------------------------------------------------------------------//
void FFtimerCR(int Speed, int totalTime,char select) {
 int temp = set_position;
  set_position = set_positionR;
  FFtimer(Speed,totalTime,select);
  set_position = temp;
}

void FFcmCL(int Speed, int distance,char select){
   int temp = set_position;
  set_position = set_positionL;
  FFcm(Speed,distance,select);
  set_position = temp;
}

void FFcmCR(int Speed, int distance,char select){
  int temp = set_position;
  set_position = set_positionR;
  FFcm(Speed,distance,select);
  set_position = temp;
}



void BBCL(int Speed,char select){
int temp = set_position;
set_position = set_positionL;
 BaseSpeed = Speed;
  InitialSpeed();
  while(1){
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp;
}

void BBCR(int Speed,char select){
int temp = set_position;
set_position = set_positionR;
 BaseSpeed = Speed;
  InitialSpeed();
  while(1){
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[7] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp;
}


//----------------------------------------------------------------------------------------- FFtimerCricleL ----------------------------------------------------------------------------------//
void BBtimerCL(int Speed, int totalTime) {
 int temp = set_position;
  set_position = set_positionL;
  BBtimer(Speed,totalTime);
  set_position = temp;
}
//----------------------------------------------------------------------------------------- FFtimerCricleR ----------------------------------------------------------------------------------//
void BBtimerCR(int Speed, int totalTime) {
 int temp = set_position;
  set_position = set_positionR;
  BBtimer(Speed,totalTime);
  set_position = temp;
}

void BBcmCL(int Speed, int distance){
   int temp = set_position;
  set_position = set_positionL;
  BBcm(Speed,distance);
  set_position = temp;
}

void BBcmCR(int Speed, int distance){
  int temp = set_position;
  set_position = set_positionR;
  BBcm(Speed,distance);
  set_position = temp;
}

//----------------------------------------------------------------------------------------- FFtimerCricleL ----------------------------------------------------------------------------------//
void BBtimerCL(int Speed, int totalTime,char select) {
 int temp = set_position;
  set_position = set_positionL;
  BBtimer(Speed,totalTime,select);
  set_position = temp;
}
//----------------------------------------------------------------------------------------- FFtimerCricleR ----------------------------------------------------------------------------------//
void BBtimerCR(int Speed, int totalTime,char select) {
 int temp = set_position;
  set_position = set_positionR;
  BBtimer(Speed,totalTime,select);
  set_position = temp;
}

void BBcmCL(int Speed, int distance,char select){
   int temp = set_position;
  set_position = set_positionL;
  BBcm(Speed,distance,select);
  set_position = temp;
}

void BBcmCR(int Speed, int distance,char select){
  int temp = set_position;
  set_position = set_positionR;
  BBcm(Speed,distance,select);
  set_position = temp;
}


void spinl(int speed) { SpinL(speed); }
void spinr(int speed) { SpinR(speed); }
void spinl2(int speed) { SpinL2(speed); }
void spinr2(int speed) { SpinR2(speed); }
void spinl_b(int speed) { SpinL_B(speed); }
void spinr_b(int speed) { SpinR_B(speed); }
void spinl2_b(int speed) { SpinL_B(speed); }
void spinr2_b(int speed) { SpinR_B(speed); }

void spinl() { SpinL(tspd); }
void spinr() { SpinR(tspd); }
void spinl2() { SpinL2(tspd); }
void spinr2() { SpinR2(tspd); }
void spinl_b() { SpinL_B(tspd); }
void spinr_b() { SpinR_B(tspd); }
void spinl2_b() { SpinL_B(tspd); }
void spinr2_b() { SpinR_B(tspd); }

void fftimer(int speed, int totaltime) { FFtimer(speed, totaltime);}
void bbtimer(int speed, int totaltime) { BBtimer(speed, totaltime);}
void fft(int speed, int totaltime) { FFtimer(speed, totaltime);}
void bbt(int speed, int totaltime) { BBtimer(speed, totaltime);}
void ffcm(int speed, float distance) {FFcm(speed, distance);}
void bbcm(int speed, float distance) {BBcm(speed, distance);}

void fftimer(int speed, int totaltime, char select) { FFtimer(speed, totaltime, select);}
void bbtimer(int speed, int totaltime, char select) { BBtimer(speed, totaltime, select);}
void fft(int speed, int totaltime, char select) { FFtimer(speed, totaltime, select);}
void bbt(int speed, int totaltime, char select) { BBtimer(speed, totaltime, select);}
void ffcm(int speed, float distance, char select) {FFcm(speed, distance, select);}
void bbcm(int speed, float distance, char select) {BBcm(speed, distance, select);}


void ff(int speed, char select) { FF(speed, select); }
void bb(int speed, char select) { BB(speed, select); }

void ffl(int speed, char select)  { FFL(speed, select); }
void ffl2(int speed, char select)  { FFL2(speed, select); }
void ffl0(int speed, char select)  { FFL0(speed, select); }
void ffr(int speed, char select)  { FFR(speed, select); }
void ffr2(int speed, char select)  { FFR2(speed, select); }
void ffr7(int speed, char select)  { FFR7(speed, select); }
void ffc(int speed, char select)  { FFC(speed, select); }
void ffc2(int speed, char select) { FFC2(speed, select); }
void ffn(int speed, char select, int numm) { FFNUM(speed, select, numm); }

void bbl(int speed, char select)  { BBL(speed, select); }
void bbl2(int speed, char select)  { BBL2(speed, select); }
void bbl0(int speed, char select)  { BBL0(speed, select); }
void bbr(int speed, char select)  { BBR(speed, select); }
void bbr2(int speed, char select)  { BBR2(speed, select); }
void bbr7(int speed, char select)  { BBR7(speed, select); }
void bbc(int speed, char select)  { BBC(speed, select); }
void bbc2(int speed, char select) { BBC2(speed, select); }
void bbn(int speed, char select, int numm) { BBNUM(speed, select, numm); }

void ffblack(int speed, char select) { FFBlack(speed, select); }
void ffwhite(int speed, char select) { FFWhite(speed, select); }
void bbblack(int speed, char select) { BBBlack(speed, select); }
void bbwhite(int speed, char select) { BBWhite(speed, select); }
void ffb(int speed, char select) { FFBlack(speed, select); }
void ffw(int speed, char select) { FFWhite(speed, select); }
void bbb(int speed, char select) { BBBlack(speed, select); }
void bbw(int speed, char select) { BBWhite(speed, select); }

void ffd(int speed, char select, float distance) {FF_Distance(speed, select, distance);}
void ffds(int speed, char select, int ir_target) {FF_DistanceS(speed, select, ir_target);}
void bbd(int speed, char select, float distance) {BB_Distance(speed, select, distance);}

void gostart(int speed) { GoStart(speed); }
void goend(int speed)   { GoEnd(speed); }

void balancef(int c) { BalanceF(c); }
void balanceb(int c) { BalanceB(c); }

void setf(int c) { SetF(c); }
void setb(int c) { SetB(c); }


void ffcl(int Speed,char select){
  FFCL(Speed,select);
}

void ffcr(int Speed,char select){
  FFCR(Speed,select);
}

void fftimercl(int Speed, int totalTime){
  FFtimerCL(Speed, totalTime);
}

void fftimercr(int Speed, int totalTime){
  FFtimerCR(Speed, totalTime);
}

void ffcmcl(int Speed, int distance){
  FFcmCL(Speed, distance);
}

void ffcmcr(int Speed, int distance){
  FFcmCR(Speed, distance);
}

void fftimercl(int Speed, int totalTime,char select){
  FFtimerCL(Speed, totalTime ,select);
}

void fftimercr(int Speed, int totalTime,char select){
  FFtimerCR(Speed, totalTime ,select);
}

void ffcmcl(int Speed, int distance,char select){
  FFcmCL(Speed, distance ,select);
}

void ffcmcr(int Speed, int distance,char select){
  FFcmCR(Speed, distance ,select);
}


void bbcl(int Speed,char select){
  BBCL(Speed,select);
}

void bbcr(int Speed,char select){
  BBCR(Speed,select);
}

void bbtimercl(int Speed, int totalTime){
  BBtimerCL(Speed, totalTime);
}

void bbtimercr(int Speed, int totalTime){
  BBtimerCR(Speed, totalTime);
}

void bbcmcl(int Speed, int distance){
  BBcmCL(Speed, distance);
}

void bbcmcr(int Speed, int distance){
  BBcmCR(Speed, distance);
}

void bbtimercl(int Speed, int totalTime,char select){
  BBtimerCL(Speed, totalTime,select);
}

void bbtimercr(int Speed, int totalTime,char select){
  BBtimerCR(Speed, totalTime,select);
}

void bbcmcl(int Speed, int distance,char select){
  BBcmCL(Speed, distance,select);
}

void bbcmcr(int Speed, int distance,char select){
  BBcmCR(Speed, distance,select);
}


//---------------------

void ffcirclel(int Speed,char select){
  FFCL(Speed,select);
}

void ffcircler(int Speed,char select){
  FFCR(Speed,select);
}

void fftimercirclel(int Speed, int totalTime){
  FFtimerCL(Speed, totalTime);
}

void fftimercircler(int Speed, int totalTime){
  FFtimerCR(Speed, totalTime);
}

void ffcmcirclel(int Speed, int distance){
  FFcmCL(Speed, distance);
}

void ffcmcircler(int Speed, int distance){
  FFcmCR(Speed, distance);
}

void fftimercirclel(int Speed, int totalTime,char select){
  FFtimerCL(Speed, totalTime ,select);
}

void fftimercircler(int Speed, int totalTime,char select){
  FFtimerCR(Speed, totalTime ,select);
}

void ffcmcirclel(int Speed, int distance,char select){
  FFcmCL(Speed, distance ,select);
}

void ffcmcircler(int Speed, int distance,char select){
  FFcmCR(Speed, distance ,select);
}


void bbcirclel(int Speed,char select){
  BBCL(Speed,select);
}

void bbcircler(int Speed,char select){
  BBCR(Speed,select);
}

void bbtimercirclel(int Speed, int totalTime){
  BBtimerCL(Speed, totalTime);
}

void bbtimercircler(int Speed, int totalTime){
  BBtimerCR(Speed, totalTime);
}

void bbcmcirclel(int Speed, int distance){
  BBcmCL(Speed, distance);
}

void bbcmcircler(int Speed, int distance){
  BBcmCR(Speed, distance);
}

void bbtimercirclel(int Speed, int totalTime,char select){
  BBtimerCL(Speed, totalTime,select);
}

void bbtimercircler(int Speed, int totalTime,char select){
  BBtimerCR(Speed, totalTime,select);
}

void bbcmcirclel(int Speed, int distance,char select){
  BBcmCL(Speed, distance,select);
}

void bbcmcircler(int Speed, int distance,char select){
  BBcmCR(Speed, distance,select);
}


//_______________


void FFCircleL(int Speed,char select){
  FFCL(Speed,select);
}

void FFCircleR(int Speed,char select){
  FFCR(Speed,select);
}

void FFtimerCircleL(int Speed, int totalTime){
  FFtimerCL(Speed, totalTime);
}

void FFtimerCircleR(int Speed, int totalTime){
  FFtimerCR(Speed, totalTime);
}

void FFcmCircleL(int Speed, int distance){
  FFcmCL(Speed, distance);
}

void FFcmCircleR(int Speed, int distance){
  FFcmCR(Speed, distance);
}

void FFtimerCircleL(int Speed, int totalTime,char select){
  FFtimerCL(Speed, totalTime ,select);
}

void FFtimerCircleR(int Speed, int totalTime,char select){
  FFtimerCR(Speed, totalTime ,select);
}

void FFcmCircleL(int Speed, int distance,char select){
  FFcmCL(Speed, distance ,select);
}

void FFcmCircleR(int Speed, int distance,char select){
  FFcmCR(Speed, distance ,select);
}


void BBCircleL(int Speed,char select){
  BBCL(Speed,select);
}

void BBCircleR(int Speed,char select){
  BBCR(Speed,select);
}

void BBtimerCircleL(int Speed, int totalTime){
  BBtimerCL(Speed, totalTime);
}

void BBtimerCircleR(int Speed, int totalTime){
  BBtimerCR(Speed, totalTime);
}

void BBcmCircleL(int Speed, int distance){
  BBcmCL(Speed, distance);
}

void BBcmCircleR(int Speed, int distance){
  BBcmCR(Speed, distance);
}

void BBtimerCircleL(int Speed, int totalTime,char select){
  BBtimerCL(Speed, totalTime,select);
}

void BBtimerCircleR(int Speed, int totalTime,char select){
  BBtimerCR(Speed, totalTime,select);
}

void BBcmCircleL(int Speed, int distance,char select){
  BBcmCL(Speed, distance,select);
}

void BBcmCircleR(int Speed, int distance,char select){
  BBcmCR(Speed, distance,select);
}
