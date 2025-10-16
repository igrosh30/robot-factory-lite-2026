/* Copyright (c) 2025  Paulo Costa, Paulo Marques
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#ifndef PICO4DRIVE_H
#define PICO4DRIVE_H

#include "Arduino.h"

#define ADC_IN_PIN	28

#define MUXA_PIN	18
#define MUXB_PIN	19
#define MUXC_PIN	20

#define TINY_CTRL_PIN	21

#define DRIVER_1A_PIN 11
#define DRIVER_1B_PIN 10

#define DRIVER_2A_PIN 13
#define DRIVER_2B_PIN 12

#define DRIVER_3A_PIN 15
#define DRIVER_3B_PIN 14

#define DRIVER_4A_PIN 17
#define DRIVER_4B_PIN 16

typedef enum { 
  p2d_drv1 = 0,
  p2d_drv2,
  p2d_drv3,
  p2d_drv4
} driver_num_t;



class pico4drive_t
{
  public:
    int analogWriteBits; 
    int analogWriteMax; 
    int PWM_limit;

    float battery_voltage;
    int button_state;

    pico4drive_t();
    void init(uint32_t PWM_freq = 16000);
    
    uint16_t read_adc(int channel);
    void set_driver_PWM(int new_PWM, int pin_a, int pin_b);
    void set_driver_PWM(int new_PWM, driver_num_t driver_num);
    int voltage_to_PWM(float u);

    int set_driver_voltage(float new_voltage, driver_num_t driver_num);

    void update(void);
};



#endif // PICO4DRIVE_H
