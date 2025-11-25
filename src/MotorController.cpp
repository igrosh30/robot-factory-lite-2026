#include "MotorController.h"
#include "pico4drive.h"
#include "math.h"
#include "robot.h"
#include "config.h"

MotorController:: MotorController()
{
    kp1, kp2 = 1;
    ki1, ki2 = 1;//porque é q em IO n t inhamos estes 2 parametros?
    integral1, last_e1 = 0;
    integral2, last_e2 = 0;
    //MotorVoltages v;
}



//returns u1&u2 calculated with PID to control the motors
void MotorController:: PID_to_votlage()
{
    //ned the w calculated to compute the error:
    float v1r,v2r = 0.0;
    float w1r,w2r = 0.0;
    
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
    //Motor1   
    integral1 += robot.dt * erro1;
    float u1 = kp1 * erro1 + ki1 * integral1;
    
    if(abs(u1) >= 5.5)//5.5- saturation level
    {
        if(u1 + abs(u1) == 0) integral1 += robot.dt * erro1;
        else integral1 -= robot.dt * erro1;
        u1 = kp1 * erro1;     
    }

    //Motor2
    integral2 += robot.dt * erro2;
    float u2 = kp2 * erro2 + ki2 * integral2;
    
    if(abs(u2) >= 5.5)//5.5- saturation level
    {
        if(u2 + abs(u2) == 0) integral2 += robot.dt * erro2;
        else integral2 -= robot.dt * erro2;
        u2 = kp2 * erro2;
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
