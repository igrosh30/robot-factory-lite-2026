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
    p4d.set_driver_voltage(3,p4d_drvSolenoid);
    isMagnetOn = true;
}

void Actuator:: magnetOff(){
    p4d.set_driver_voltage(-2,p4d_drvSolenoid);
    delay(50);
    p4d.set_driver_voltage(0,p4d_drvSolenoid);
    isMagnetOn = false;
}

bool Actuator:: isSwitchOn()
{
    if(digitalRead(SWITCH_PIN) == LOW) switchOn = true;
    else switchOn = false;
    return switchOn;
}

