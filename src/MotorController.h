#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "pico4drive.h"
//#include "robot.h"


class MotorController
{
    public:
    pico4drive_t pico4drive;
    //robot_t robot;
    float v_ref, p_ref;
    
    float kp,ki;
    

    void setW(float w1r,float w2r, float w1e, float w2e);
    float compute_PI(float erro);//gives-me the u

    void driveMotor(float u1,float u2);

    
    private:
};

#endif //MotorController