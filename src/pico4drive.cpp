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

#include "Arduino.h"
#include "pico4drive.h"

#define MAX_VOLTAGE_USAGE 5.5

//VERIFICAR A DEFINICAO DOS DRV! FALTA B não?!
int p4d_pwm_pins[4][2] = {{DRIVER_1A_PIN, DRIVER_1A_PIN},
                          {DRIVER_2A_PIN, DRIVER_2A_PIN},
                          {DRIVER_3A_PIN, DRIVER_3A_PIN},
                          {DRIVER_4A_PIN, DRIVER_4A_PIN}};

SerialPIO SerialTiny(NOPIN, 21);

//set the levels allowed in the use of PWM to control when initializing a pico4drive
pico4drive_t::pico4drive_t()
{
  //resolution for representation of our PWM with 10 bits
  analogWriteBits = 10; 
  analogWriteMax = (1 << analogWriteBits) - 1; 
  
  battery_voltage = 7.4;
  
  PWM_limit = MAX_VOLTAGE_USAGE/battery_voltage * analogWriteMax;//limit my pwm signal to 5.5V MAX
}

void pico4drive_t::init(uint32_t PWM_freq)
{
// ADC mux pins
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  
  SerialTiny.begin();

  //configures the PWM System(how many bites/cycle & freq)
  analogWriteResolution(analogWriteBits);
  analogWriteFreq(PWM_freq); //16000 

  for (int i = 0; i < 4; i++) {
    pinMode(p4d_pwm_pins[i][0], OUTPUT);
    pinMode(p4d_pwm_pins[i][1], OUTPUT);
  }
}

void pico4drive_t::update(void)
{
  if (SerialTiny.available()) { 
    int b = SerialTiny.read();  // Read battery voltage and On button state from the Tiny controller
    battery_voltage = 1e-3 * ((b >> 1) * 50 + 4800);
    button_state = b & 1;
  }
}

static void adc_set_channel(int channel)
{
	gpio_put_masked(digitalPinToBitMask(MUXA_PIN) | digitalPinToBitMask(MUXB_PIN) | digitalPinToBitMask(MUXC_PIN), channel << MUXA_PIN);
  //digitalWrite(MUXA_PIN, channel & 1);
  //digitalWrite(MUXB_PIN, (channel >> 1) & 1);
  //digitalWrite(MUXC_PIN, (channel >> 2) & 1);
}

uint16_t pico4drive_t::read_adc(int channel)
{
	adc_set_channel(channel); // Switch external MUX to the desired channel
	delayMicroseconds(10);
	return analogRead(A2);    // The mux connects to analog input A2
}

int pico4drive_t::voltage_to_PWM(float u)
{
  return (u / battery_voltage) * analogWriteMax;
}

void pico4drive_t::set_driver_PWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_max = analogWriteMax;
  if (new_PWM >  PWM_limit) new_PWM =  PWM_limit;
  if (new_PWM < -PWM_limit) new_PWM = -PWM_limit;
  
  if (new_PWM == 0) {  // Both outputs 0 -> A = H, B = H
    //the config is that HIGH PWM - OFF
    analogWrite(pin_a, PWM_max);
    analogWrite(pin_b, PWM_max);
  } else if (new_PWM > 0) {
    analogWrite(pin_a, PWM_max - new_PWM);
    analogWrite(pin_b, PWM_max);
  } else {
    analogWrite(pin_a, PWM_max);
    analogWrite(pin_b, PWM_max + new_PWM);
  }
}

void pico4drive_t::set_driver_PWM(int new_PWM, driver_num_t driver_num)
{
  //p4d_pwm_pins tem todos os pins definidos, tenho de meter driver_num - 1 -> nº do DRV
  //DRV1 -> p4d_pwm_pins[0][]
  set_driver_PWM(new_PWM, p4d_pwm_pins[driver_num -1][0], p4d_pwm_pins[driver_num-1][1]);
}


int pico4drive_t::set_driver_voltage(float new_voltage, driver_num_t driver_num)
{
  int pwm = voltage_to_PWM(new_voltage);
  set_driver_PWM(pwm, driver_num);
  return pwm;
}
