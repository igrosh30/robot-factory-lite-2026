#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "pico4drive.h"
#include "config.h"



class MotorController
{
    public:
    MotorController();
    void PIDController_Update();
    void driveMotor(float u1,float u2);
    
    private:
    float kp1,ki1;
    float kp2,ki2;
    float integrator1,integrator2; //same as i[n-1]!
    float prevError1,prevError2; //same as e[n-1]!
    pico4drive_t pico4drive;
    //void driveMotor(float u1,float u2);
    MotorVoltages CalcPID(float erro1,float erro2);//returns the structure MotorVotages with u1&u2
    bool isNegative(float num);
};

#endif //MotorController