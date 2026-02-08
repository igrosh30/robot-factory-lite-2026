#include "Actuators.h"
#include "config.h"


// --- Phase 1: Construction ---//
Actuator::Actuator(pico4drive_t& driver) : p4d(driver) {
    isMagnetOn = false;
}

void Actuator::init()
{
    pinMode(SWITCH_PIN,INPUT_PULLUP);//inverse the logic to ensure stability
}

void Actuator:: magnetOn(){
    p4d.set_driver_voltage(4,p4d_drvSolenoid);
    isMagnetOn = true;
}

void Actuator:: magnetOff(){
    p4d.set_driver_voltage(-3,p4d_drvSolenoid);
    
    p4d.set_driver_voltage(0,p4d_drvSolenoid);
    isMagnetOn = false;
}

void Actuator::update()
{
    if(digitalRead(SWITCH_PIN) == LOW){
        isSwitchOn = true;
    } 
    else{
        isSwitchOn = false;
    } 
    if(isMagnetOn) magnetOn();
    else magnetOff();
}

