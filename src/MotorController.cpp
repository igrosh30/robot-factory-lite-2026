#include "MotorController.h"
#include "pico4drive.h"
#include "math.h"
#include "robot.h"
#include "config.h"

MotorController:: MotorController()
{
    kp1, kp2 = 1;
    ki1, ki2 = 1;
    integral1, last_e2 = 0;
    last_e1, last_e2 = 0;
    //MotorVoltages v;
}

//sabendo o w e v do robô - robot.w_req e robot.v_req acho q não preciso disto...
void MotorController:: setVW(MotorController motor1, MotorController motor2)
{
    //Need to calculate the w1 and w2 to the PID?!
}


//returns u1&u2 calculated with PID
void MotorController:: PID_to_votlage()
{
    //ned the w calculated to compute the error:
    float v1r,v2r = 0.0;
    float w1r,w2r = 0.0;//see how to get the acctual w1r and w2r
    
    v1r = robot.v_req + robot.w_req*robot.wheel_dist*0.5;
    v2r = robot.v_req - robot.w_req*robot.wheel_dist*0.5;

    w1r = v1r / robot.wheel_radius;
    w2r = v2r / robot.wheel_radius;
    
    float erro1 = w1r - robot.w1e;
    float erro2 = w2r - robot.w2e;
    
    MotorVoltages v;
    v = compute_PID(erro1,erro2);//whould get u1&u2
    
    driveMotor(v.u1);
    driveMotor(v.u2);

}

//PID calculation
MotorVoltages MotorController:: compute_PID(float erro1,float erro2)
{   
    integral1 += robot.dt * erro1;
    float u1 = kp1 * erro1 + ki1 * integral1;
    
    if(abs(u1) >= 5.5)//5.5- saturation level
    {
        u1 = kp1 * erro1; 
        integral1 -= robot.dt * erro1;
    }

    integral2 += robot.dt * erro2;
    float u2 = kp2 * erro2 + ki2 * integral2;
    
    if(abs(u2) >= 5.5)//5.5- saturation level
    {
        u2 = kp2 * erro1; 
        integral2 -= robot.dt * erro2;
    }

    MotorVoltages v;
    v.u1 = u1;
    v.u2 = u2;
    return v;

}


void MotorController:: driveMotor(float u)
{
    pico4drive.set_driver_voltage(u,p4d_drv1);//sera q vai dar drive sempre à mesma drive?
}
