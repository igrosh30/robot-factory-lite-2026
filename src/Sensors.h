#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "pico4drive.h"

#define NUM_SENSORS 5

//brack captures more light so we get a smaller current, lower value 

const int weights[NUM_SENSORS]     = {-8,-4,0,4,8};
const int weightsTarg[NUM_SENSORS] = {-8,-4,1,0,8};

typedef enum
{
    FRONT_SENSOR,
    BACK_SENSOR   
} SensorType;

class Sensor
{
    public:    
        pico4drive_t& p4d;
        SensorType type;

        //we need to store the max&min values for each sensor to then calibrate:
        uint16_t minValues[NUM_SENSORS]; //pico returns a value from 0 and 1023
        uint16_t maxValues[NUM_SENSORS];

        //stores last read values for debug
        uint16_t IR_Values[NUM_SENSORS];

        Sensor(pico4drive_t& driver, SensorType t);
        void init();
        void calibrate(); //calibrate the values from ADC before using them, call this in setup
        void setCalibration(const uint16_t* min, const uint16_t* max);
        void readRaw(uint16_t* values);
        
        void readValues();

        float getLineError();
        float getLineErrorTarget();

};


#endif