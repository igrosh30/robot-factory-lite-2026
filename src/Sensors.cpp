#include "Sensors.h"

// The Pico has specific pins for Analog Inputs. 
// Based on standard wiring for this competition board:
//we wire last 2 values to be read to  2 ADC pins (GPIO 26 & 27) because the MUX was full
#define BACK_SENSOR_PIN_3 26
#define BACK_SENSOR_PIN_4 27

// --- Phase 1: Construction ---//
Sensor::Sensor(pico4drive_t& driver, SensorType t) : p4d(driver), type(t) {
    kl = 0.7;
    erro = 0;
}

// --- Phase 2: Set boundaries && newPins ---//
void Sensor :: init()
{
    //set min&Max Values
    for(int i = 0; i<NUM_SENSORS; i++)
    {
        minValues[i] = 1023;
        maxValues[i] = 0;
        IR_Values[i] = 0;
    }
    if(type == BACK_SENSOR)
    {
        pinMode(BACK_SENSOR_PIN_3, INPUT);
        pinMode(BACK_SENSOR_PIN_4, INPUT);
    }
}

void Sensor :: calibrate(){//get the max readed value for each sensor & min value - so then we can normalize values

    uint16_t tempValues[NUM_SENSORS];
    readRaw(tempValues);

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if(minValues[i] > tempValues[i]) minValues[i] = tempValues[i];
        if(maxValues[i] < tempValues[i]) maxValues[i] = tempValues[i];
    }
}

void Sensor::setCalibration(const uint16_t* min, const uint16_t* max) {
    for(int i = 0; i < NUM_SENSORS; i++) {
        minValues[i] = min[i];
        maxValues[i] = max[i];
    }
}

void Sensor ::readRaw(uint16_t* values)//bridge to pass the values readed to an array
{
    if(type == FRONT_SENSOR)
    {
        for(int i = 0; i<NUM_SENSORS;i++)
        {
            values[i] = p4d.read_adc(i);
        }
    }
    else //read backarwrds sensor
    {
        //complete the remaning mux values:
        for(int i = 0; i<NUM_SENSORS-2; i++)
        {
            values[i] = p4d.read_adc(NUM_SENSORS+i);
        }
        //read manualy the remaining values:
        values[3] = analogRead(BACK_SENSOR_PIN_3);
        values[4] = analogRead(BACK_SENSOR_PIN_4);
    }
}

void Sensor :: readValues(){
    readRaw(IR_Values);
}

float Sensor :: getLineError() //Compute how much do I need to rotate my robot! 
{
    readValues(); // refresh the debug data- Only to check the last readed values! 

    uint16_t tempValues[NUM_SENSORS];
    readRaw(tempValues);

    flagFound = false;
    //1- need to normalize the values- each sensor can have different high/low values
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int min = minValues[i];int max = maxValues[i];

        if(min == max ) return 0;
        int tempVal = constrain(tempValues[i],min,max);
        IR_norm1[i] = tempVal;
        tempVal = map(tempVal,min,max,1000,0);//maping values we will consider white as 0 - no interes in rotating, only back capturing! 

        if(tempVal<50) IR_norm[i] =0; //if less than 5%
        IR_norm[i] = tempVal;

        if(!flagFound)
        {
            if(i==0){
                if(activated(IR_norm[0]))
                {
                    erro = sensorDist[0];
                    flagFound = true;
                } 
            }
            else{
                if(activated(IR_norm[i]) && (!activated(IR_norm[i-1])))//IR[i] is black, IR[i-1] needs to be white
                {
                    erro = sensorDist[i];
                    flagFound = true;
                }
            }
        }
    }
    return erro;
}

bool Sensor:: activated(float normVal)
{
    //0 - is white (deactivated) / 1000 - is black (activated) 
    if(normVal <= 500) return false;
    else return true;
}


float Sensor:: getLineErrorTarget(){
    readValues();

    uint16_t tempValues[NUM_SENSORS];
    readRaw(tempValues);

    //long weightedSum = 0;
    long totalActivity = 0;

    //normalization process:
    uint16_t ir2 = 0.0f;
    for(int i = 0; i< NUM_SENSORS; i++)
    {
        int min = minValues[i]; int max = maxValues[i];
        if(min==max) return 0;
        
        int val = constrain(tempValues[i],min,max);
        val = map(val,min,max,1000,0);

        if(val < 50) val = 0;

        if(i == 2) ir2 = val;

        //weightedSum += val * weightsTarg[i];
        totalActivity += val;
    }
    static float lastError = 0;
    if (totalActivity == 0)//all sensors saw white,e.g.:maybe lost....
    {
        return lastError;
    }

    float rawError = float(ir2) - SENSOR_TARGET;
    float normalizedError = rawError / 500;
    lastError = normalizedError;
    
    return normalizedError;

}