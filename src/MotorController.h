#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "pico4drive.h"



class MotorController
{
    public:
    MotorController();
    void setWhellsW(float wr);
    void driveMotor(float u);

    private:
    float kp,ki;
    float integral;
    float last_e;
    pico4drive_t pico4drive;

    float PID_to_votlage(float wr, float we);
    float compute_PID(float erro);//returns u for PID
};

#endif //MotorController