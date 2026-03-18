#include "MotorController.h"
#include "pico4drive.h"
#include "math.h"
#include "robot.h"
#include "config.h"

MotorController:: MotorController()
{
    kp1 = 0.5, kp2 = 0.5;
    ki1 = 6, ki2 = 6;
    integrator1 = 0, prevError1 = 0;
    integrator2 = 0, prevError2 = 0;
    e1 =0,e2=0;
    MotorVoltages v;
}



//returns u1&u2 calculated with PID to control the motors
void MotorController:: PIDController_Update()
{
    //ned the w calculated to compute the error:
    
    float v1r = robot.v_req + robot.w_req*robot.wheel_dist*0.5;
    float v2r = robot.v_req - robot.w_req*robot.wheel_dist*0.5;

    float w1r = v1r / robot.wheel_radius;
    float w2r = v2r / robot.wheel_radius;
    
    float erro1 = w1r - robot.w1e;
    float erro2 = w2r - robot.w2e;
    e1= erro1;
    e2= erro2;
    MotorVoltages v = CalcPID(erro1,erro2);
    u_send = v;
    
    
    driveMotor(v.u1,v.u2);

}

//PID calculation -> U(s) = P(S) + I(S)
MotorVoltages MotorController:: CalcPID(float erro1,float erro2)
{
    //Motor1   
    float p1 = kp1*erro1;  
    integrator1 = integrator1 + ki1*robot.dt*0.5 *erro1;
    float u1 = p1 + integrator1;
    
    if(abs(u1)> MAX_VOLTAGE_USAGE)
    {
        if(isNegative(u1) && isNegative(erro1))
        {
            integrator1 += ki1*robot.dt*0.5 *(erro1 + prevError1);
        }
        else if(!isNegative(u1) && !isNegative(erro1))
        {
            integrator1 -= ki1*robot.dt*0.5 *(erro1 + prevError1);
        }
        u1 = p1 + integrator1;
    }
    prevError1 = erro1;

    //Motor2   
    float p2 = kp2*erro2;  
    integrator2 = integrator2 + ki2*robot.dt*0.5 *erro2;
    float u2 = p2 + integrator2;
    
    if(abs(u2)> MAX_VOLTAGE_USAGE )//remeber to change for MAX
    {
        if(isNegative(u2) && isNegative(erro2))
        {
            integrator2 += ki2*robot.dt*0.5 *(erro2 + prevError2);
        }
        else if(!isNegative(u2) && !isNegative(erro2))
        {
            integrator2 -= ki2*robot.dt*0.5*(erro2 + prevError2);
        }
        u2 = p2 + integrator2;
    }
    prevError2 = erro2;
    
    //Serial.printf("Proportional1: %.3f    Proportional2: %.3f\n", p1, p2);
    
    MotorVoltages v;
    v.u1 = u1;
    v.u2 = u2;
    return v;
}


void MotorController:: driveMotor(float u1, float u2)
{
    pico4drive.set_driver_voltage(u1,p4d_drv1);
    pico4drive.set_driver_voltage(u2,p4d_drv2);
}
bool MotorController:: isNegative(float num){
    if(num >= 0) return false;
    else return true;
}