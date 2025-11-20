#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "pico4drive.h"
#include "config.h"



class MotorController
{
    public:
    MotorController();
    void driveMotor(float u);

    private:
    float kp1,ki1;
    float kp2,ki2;
    float integral1,integral2;
    float last_e1,last_e2;
    pico4drive_t pico4drive;

    void PID_to_votlage();
    MotorVoltages compute_PID(float erro1,float erro2);//returns the structure MotorVotages with u1&u2
    void setVW(MotorController motor1, MotorController motor2);
};

#endif //MotorController