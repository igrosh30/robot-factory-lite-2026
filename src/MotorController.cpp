#include "MotorController.h"
#include "pico4drive.h"
#include "math.h"
#include "robot.h"



void MotorController:: driveMotor(float u1,float u2)
{
    pico4drive.set_driver_voltage(u1,p4d_drv1);
    pico4drive.set_driver_voltage(u2,p4d_drv2);
}//robot.setW(w1r,w2r) -> this will calculate the u1 and u2 to move the motors

//set wheel W
float MotorController:: setWhellW(float w1r,float w2r, float w1e, float w2e)
{
    //ned the w calculated to compute the error:
    float erro1 = w1r - w1e;
    float erro2 = w2r - w2e;

    float u1 = compute_PI(erro1);
    float u2 = compute_PI(erro2);

    return u1,u2;
}

float MotorController:: compute_PI(float erro)
{    
    return erro*kp + prev_Ierro + robot.dt*erro;
}


