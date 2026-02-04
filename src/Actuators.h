#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include "pico4drive.h"

//#define NUM_SOLENOIDS 2 

class Actuator
{
    public:
        pico4drive_t& p4d;

        bool isMagnetOn;
        bool switchOn;

        Actuator(pico4drive_t& driver);
        void init();
        void magnetOn();
        void magnetOff();
        bool isSwitchOn();

};

#endif