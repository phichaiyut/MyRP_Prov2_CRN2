
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


void InitialSpeed() {
//  LastError_F   = 3500, LastError_B = 3500;
if (BaseSpeed <= 10) {
    LeftBaseSpeed = BaseSpeed - L[SPD_10];
    RightBaseSpeed = BaseSpeed - R[SPD_10];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_10];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_10];
    PID_KP_Front = KP[SPD_10];  //forward PID
    PID_KD_Front = KD[SPD_10];

    PID_KP_Back = KP_Back[SPD_10];  //backward PID
    PID_KD_Back = KD_Back[SPD_10];

  }
  else if (BaseSpeed <= 20) {
    LeftBaseSpeed = BaseSpeed - L[SPD_20];
    RightBaseSpeed = BaseSpeed - R[SPD_20];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_20];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_20];
    PID_KP_Front = KP[SPD_20];  //forward PID
    PID_KD_Front = KD[SPD_20];

    PID_KP_Back = KP_Back[SPD_20];  //backward PID
    PID_KD_Back = KD_Back[SPD_20];

  }

  else if (BaseSpeed <= 30) {
    LeftBaseSpeed = BaseSpeed - L[SPD_30];
    RightBaseSpeed = BaseSpeed - R[SPD_30];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_30];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_30];
    PID_KP_Front = KP[SPD_30];  //forward PID
    PID_KD_Front = KD[SPD_30];

    PID_KP_Back = KP_Back[SPD_30];  //backward PID
    PID_KD_Back = KD_Back[SPD_30];

  } else if (BaseSpeed <= 40) {
    LeftBaseSpeed = BaseSpeed - L[SPD_40];
    RightBaseSpeed = BaseSpeed - R[SPD_40];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_40];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_40];
    PID_KP_Front = KP[SPD_40];  //forward PID
    PID_KD_Front = KD[SPD_40];  //11

    PID_KP_Back = KP_Back[SPD_50];  //backward PID
    PID_KD_Back = KD_Back[SPD_50];

  } else if (BaseSpeed <= 50) {
    LeftBaseSpeed = BaseSpeed - L[SPD_50];
    RightBaseSpeed = BaseSpeed - R[SPD_50];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_50];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_50];
    PID_KP_Front = KP[SPD_50];  //forward PID
    PID_KD_Front = KD[SPD_50]; //14

    PID_KP_Back = KP_Back[SPD_50];  //backward PID
    PID_KD_Back = KD_Back[SPD_50];

  } else if (BaseSpeed <= 60) {
    LeftBaseSpeed = BaseSpeed - L[SPD_60];
    RightBaseSpeed = BaseSpeed - R[SPD_60];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_60];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_60];
    PID_KP_Front = KP[SPD_60];  //forward PID
    PID_KD_Front = KD[SPD_60];//17

    PID_KP_Back = KP_Back[SPD_60];  //backward PID
    PID_KD_Back = KD_Back[SPD_60];

  } else if (BaseSpeed <= 70) {
    LeftBaseSpeed = BaseSpeed - L[SPD_70];
    RightBaseSpeed = BaseSpeed - R[SPD_70];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_70];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_70];
    PID_KP_Front = KP[SPD_70];  //forward PID
    PID_KD_Front = KD[SPD_70];//20

    PID_KP_Back = KP_Back[SPD_70];  //backward PID
    PID_KD_Back = KD_Back[SPD_70];

  } else if (BaseSpeed <= 80) {
    LeftBaseSpeed = BaseSpeed - L[SPD_80];
    RightBaseSpeed = BaseSpeed - R[SPD_80];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_80];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_80];
    PID_KP_Front = KP[SPD_80];  //forward PID
    PID_KD_Front = KD[SPD_80]; //20

    PID_KP_Back = KP_Back[SPD_80];  //backward PID
    PID_KD_Back = KD_Back[SPD_80];


  } else if (BaseSpeed <= 90) {
    LeftBaseSpeed = BaseSpeed - L[SPD_90];
    RightBaseSpeed = BaseSpeed - R[SPD_90];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_90];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_90];
    PID_KP_Front = KP[SPD_90];  //forward PID
    PID_KD_Front = KD[SPD_90];//22

    PID_KP_Back = KP_Back[SPD_90];  //backward PID
    PID_KD_Back = KD_Back[SPD_90];


  } else {
    LeftBaseSpeed = BaseSpeed - L[SPD_100];
    RightBaseSpeed = BaseSpeed - R[SPD_100];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_100];
    BackRightBaseSpeed = BaseSpeed - BR[SPD_100];
    PID_KP_Front = KP[SPD_100];  //forward PID
    PID_KD_Front = KD[SPD_100];//25

    PID_KP_Back = KP_Back[SPD_100];  //backward PID
    PID_KD_Back = KD_Back[SPD_100];
  }
}

void FD(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();

  Move(LeftBaseSpeed, RightBaseSpeed, time_ms);
}

void BK(int Speed, int time_ms) {
  BaseSpeed = Speed;
  InitialSpeed();

  Move(-BackLeftBaseSpeed, -BackRightBaseSpeed, time_ms);
}

void SL(int Speed, int time_ms) {
  MotorStop(0);
  Move(-Speed, Speed, time_ms);
  MotorStop(0);
}

void SR(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, -Speed, time_ms);
  MotorStop(0);
}

void TL(int Speed, int time_ms) {
  MotorStop(0);
  Move(0, Speed, time_ms);
  MotorStop(0);
}

void TR(int Speed, int time_ms) {
  MotorStop(0);
  Move(Speed, 0, time_ms);
  MotorStop(0);
}

// // ---------- คำสั่งตัวพิมพ์เล็ก ----------

// void fd(int Speed, int time_ms) {
//   FD(Speed, time_ms);
// }

// void bk(int Speed, int time_ms) {
//   BK(Speed, time_ms);
// }

// void sl(int Speed, int time_ms) {
//   SL(Speed, time_ms);
// }

// void sr(int Speed, int time_ms) {
//   SR(Speed, time_ms);
// }

// void tl(int Speed, int time_ms) {
//   TL(Speed, time_ms);
// }

// void tr(int Speed, int time_ms) {
//   TR(Speed, time_ms);
// }
