#include "Actuators.h"
#include "config.h"
#include "Sensors.h"



// --- Phase 1: Construction ---//
Actuator::Actuator(pico4drive_t& driver, Side t) : p4d(driver),typeSide(t){
    isMagnetOn = false;
}

void Actuator::init()
{
    if(typeSide == Side::FRONT){
        pinMode(FRONT_SWITCH_PIN,INPUT_PULLUP);//inverse the logic to ensure stability
    }
    else if(typeSide == Side::BACK){
        pinMode(BACK_SWITCH_PIN,INPUT_PULLUP);
    }
}

void Actuator:: magnetOn(){

    if(typeSide == Side:: FRONT)
    {
        p4d.set_driver_voltage(4,p4d_drvSolenoid_front);
        isMagnetOn = true;
    }
    else if(typeSide == Side::BACK)
    {
        p4d.set_driver_voltage(4,p4d_drvSolenoid_back);
        isMagnetOn = true;   
    }
}

void Actuator:: magnetOff(){
    if(typeSide == Side::FRONT)
    {
        p4d.set_driver_voltage(-3,p4d_drvSolenoid_front);
        p4d.set_driver_voltage(0,p4d_drvSolenoid_front);
        isMagnetOn = false;
    }
    
}

void Actuator::update()
{
    switch (typeSide)
    {
    case Side::FRONT:
        if(digitalRead(FRONT_SWITCH_PIN) == LOW){
            isSwitchOn = true;
        } 
        else{
            isSwitchOn = false;
        } 
        if(isMagnetOn) magnetOn();
        else magnetOff();
        break;
    
    case Side::BACK:
        if(digitalRead(BACK_SWITCH_PIN) == LOW){
            isSwitchOn = true;
        } 
        else{
            isSwitchOn = false;
        } 
        if(isMagnetOn) magnetOn();
        else magnetOff();
        break;
    
    default:
        break;
    }
    
}


// --- RobotSide: Constructor ---//
RobotSide:: RobotSide(pico4drive_t& driver, Side t)
: sensor(driver,t),
  actuators(driver,t)
{   

}