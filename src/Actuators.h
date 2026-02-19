#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include "pico4drive.h"
#include "Sensors.h"

//#define NUM_SOLENOIDS 2 



class Actuator
{
    public:
        pico4drive_t& p4d;
        Side typeSide;

        bool isMagnetOn;
        bool isSwitchOn;
        bool lastTouch;
        
        Actuator(pico4drive_t& driver, Side t);
        void init();
        void magnetOn();
        void magnetOff();
        void update();
        //bool isSwitchOn();
};
class RobotSide{
public:
    Sensor sensor;
    Actuator actuators;
    RobotSide(pico4drive_t& p4d, Side t);  // constructor sets up both
};

#endif