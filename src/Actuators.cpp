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
        pinMode(SWITCHL_PIN,INPUT_PULLUP);//inverse the logic to ensure stability
        pinMode(SWITCHR_PIN,INPUT_PULLUP);//inverse the logic to ensure stability
       
    }
    /*
    else if(typeSide == Side::BACK){
        pinMode(BACK_R_SWITCH_PIN,INPUT_PULLUP);
        pinMode(BACK_L_SWITCH_PIN,INPUT_PULLUP);
    }*/
}

void Actuator:: magnetOn(){
    
    if(typeSide == Side:: FRONT)
    {
        p4d.set_driver_voltage(5,p4d_drvSolenoid_front);
        isMagnetOn = true;
    }
    else if(typeSide == Side::BACK)
    {
        p4d.set_driver_voltage(5,p4d_drvSolenoid_back);
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
    if(digitalRead(SWITCHL_PIN) == LOW){
        isSwitch_left_On = true;
    } 
    else{
        isSwitch_left_On = false;
    } 
    if(digitalRead(SWITCHR_PIN) == LOW){
        isSwitch_right_On = true;
    } 
    else{
        isSwitch_right_On = false;
    } 
    if(isMagnetOn) magnetOn();
    else magnetOff();
    
    /*  
    switch (typeSide)
    {
    case Side::FRONT:
        
        break;
    
    case Side::BACK:
        if(digitalRead(BACK_L_SWITCH_PIN) == LOW){
            isSwitch_left_On = true;
        } 
        else{
            isSwitch_left_On = false;
        } 
        if(isMagnetOn) magnetOn();
        else magnetOff();
        break;
    
    default:
        break;
    }
    */  
}


// --- RobotSide: Constructor ---//
RobotSide:: RobotSide(pico4drive_t& driver, Side t)
: sensor(driver,t),
  actuators(driver,t)
{   

}