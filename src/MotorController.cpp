#include "MotorController.h"
#include "pico4drive.h"
#include "math.h"
#include "robot.h"

//let's define the constructor:

/*
What I want my ControllMotors Do:
1-> Having a W1ref,W2ref -> compute the expected u1,u2 to send the pwm

*/

//send u1,u2 to motor M1&M2:
void MotorController:: driveMotor(float u1,float u2)
{
    pico4drive.set_driver_voltage(u1,p4d_drv1);
    pico4drive.set_driver_voltage(u2,p4d_drv2);
}//robot.setW(w1r,w2r) -> this will calculate the u1 and u2 to move the motors

void MotorController:: setW(float w1r,float w2r, float w1e, float w2e)
{
    //ned the w calculated to compute the error:
    float erro1 = w1r - w1e;
    float erro2 = w2r - w2e;

    float u1 = compute_PI(erro1);
    float u2 = compute_PI(erro2);

    //send u1,u2
    driveMotor(u1,u2);
    //erro1 calculate PID:
}

float MotorController:: compute_PI(float erro)
{
    return kp*erro + ki*erro*robot.dt;
}


