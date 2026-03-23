#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "pico4drive.h"
#include "config.h"

#define NUM_SENSORS 5

//brack captures more light so we get a smaller current, lower value 

const int weights[NUM_SENSORS]     = {-8,-4,0,4,8};
const int weightsTarg[NUM_SENSORS] = {-8,-4,1,0,8};
const float sensorDist[NUM_SENSORS] = {3.4,1.7,0,-1.7,-3.4};

/*
typedef enum
{
    FRONT_SENSOR,
    BACK_SENSOR   
} SensorType;
*/


class Sensor
{
    public:    
        pico4drive_t& p4d;
        Side2Follow side2follow;
        Side typeSide;
        float erro;
        bool flagFound;
        bool flagInters;
        bool flagType; //0- Front working , 1- back working
        bool wasIntersection;
        int intersections;


        //we need to store the max&min values for each sensor to then calibrate:
        uint16_t minValues[NUM_SENSORS]; //pico returns a value from 0 and 1023
        uint16_t maxValues[NUM_SENSORS];

        //stores last read values for debug
        uint16_t IR_Values[NUM_SENSORS];

        uint16_t IR_norm[NUM_SENSORS];
        uint16_t IR_norm1[NUM_SENSORS];

        float kl ;

        Sensor(pico4drive_t& driver, Side t);
        void init();
        void calibrate(); //calibrate the values from ADC before using them, call this in setup
        void setCalibration(const uint16_t* min, const uint16_t* max);
        void readRaw(uint16_t* values);
        
        void readValues();

        void getLineError(Side2Follow side2follow);

        bool activated(float normVal);

};


#endif