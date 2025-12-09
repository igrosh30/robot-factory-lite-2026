#include "MotorController.h"
#include "pico4drive.h"
#include "math.h"
#include "robot.h"
#include "config.h"

MotorController:: MotorController()
{
    kp1 = 1, kp2 = 1;
    ki1 = 1, ki2 = 1;
    integrator1 = 0, prevError1 = 0;
    integrator2 = 0, prevError2 = 0;
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
    
    float erro1 = w1r + robot.w1e;
    float erro2 = w2r + robot.w2e;
    
    MotorVoltages v = CalcPID(erro1,erro2);
    
    driveMotor(v.u1,v.u2);

    /*
    static unsigned long last_print = 0;
    if (millis() - last_print >= 200) {           // print only 10 times per second
        last_print = millis();
        Serial.println("\n=== PID DEBUG ===");
        Serial.printf("v_req: %.3f m/s    w_req: %.3f rad/s\n", robot.v_req, robot.w_req);
        Serial.printf("w1_req: %.3f m/s    w2_req: %.3f rad/s\n", w1r, w2r);
        Serial.printf("w1e: %.2f    w2e: %.2f rad/s\n", robot.w1e, robot.w2e);
        Serial.printf("erro1: %.3f    erro2: %.3f\n", erro1, erro2);
        Serial.printf("u1: %.3f V    u2: %.3f V\n", v.u1, v.u2);
        Serial.printf("integrator1: %.3f    integrator2: %.3f\n", integrator1, integrator2);
        //Serial.printf("Proportional1: %.3f    Proportional2: %.3f\n", p1, p2);
        Serial.println("==================\n");
    }
    */
}

//PID calculation -> U(s) = P(S) + I(S)
MotorVoltages MotorController:: CalcPID(float erro1,float erro2)
{
    //Motor1   
    float p1 = kp1*erro1;  
    integrator1 = integrator1 + ki1*robot.dt*0.5 *(erro1 + prevError1);
    float u1 = p1 + integrator1;
    
    if(abs(u1)>5.5)
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
    integrator2 = integrator2 + ki2*robot.dt*0.5 *(erro2 + prevError2);
    float u2 = p2 + integrator2;
    
    if(abs(u2)>5.5)
    {
        if(isNegative(u2) && isNegative(erro2))
        {
            integrator2 += ki1*robot.dt*0.5 *(erro2 + prevError2);
        }
        else if(!isNegative(u2) && !isNegative(erro2))
        {
            integrator2 -= ki1*robot.dt*0.5*(erro2 + prevError2);
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