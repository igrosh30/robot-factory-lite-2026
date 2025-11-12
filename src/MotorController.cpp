#include "MotorController.h"
#include "pico4drive.h"
#include "math.h"
#include "robot.h"

MotorController:: MotorController()
{
    kp = 1;
    ki = 1;
    integral = 0;
    last_e = 0;
}

void MotorController:: setWhellsW(float wr)
{
    float u= PID_to_votlage(wr,robot.w1e);
    
    driveMotor(u);
}

void MotorController:: driveMotor(float u)
{
    pico4drive.set_driver_voltage(u,p4d_drv1);
}

//returns u1&u2 calculated with PID
float MotorController:: PID_to_votlage(float wr, float we)
{
    //ned the w calculated to compute the error:
    float erro = wr - we;
    float u = compute_PID(erro);
    
    return u;
}

//PID calculation
float MotorController:: compute_PID(float erro)
{   
    integral += robot.dt * erro;
    float u = kp * erro + ki * integral;
    
    if(abs(u) >= 5.5)//5.5- saturation level
    {
        u = kp * erro; 
        integral -= robot.dt * erro;
    }
    return  u;
}


