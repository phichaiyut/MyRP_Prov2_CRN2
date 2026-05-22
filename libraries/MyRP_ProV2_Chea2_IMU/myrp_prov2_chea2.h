#include "Buzzer.h"
#include "Sensor.h"
#include "Motor.h"
#include "Pid.h"
#include "myrp_prov2_bmi160.h"
#include "myrp_prov2_servo.h"
#include "myrp_prov2_imu.h"


void SW(){
    MotorStop();
    sw();
}