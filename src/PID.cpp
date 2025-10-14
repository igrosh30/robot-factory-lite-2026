/* Copyright (c) 2023  Paulo Costa
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
#include "PID.h"

PID_t::PID_t()
{
  ppars = &pars;  // Use the internal pars
  
  // Some typical values
  pars.Kfd = 0;
  pars.Kf = 0.33;
  pars.Kc = 0.15;
  pars.Ki = 1;
  pars.Kd = 0;
  pars.dt = 0.04;
  pars.dead_zone = 0.2;

  m_max = 5.8;
  m_min = -5.8;

  Se = 0;
  e = 0;
  last_e = 0;
  last_pe = 0;
  y = 0;
  y_ref = 0;
}

void PID_t::init_pars(PID_pars_t* appars)
{
  ppars = appars;
}

float PID_t::calc(float new_y_ref, float new_y)
{
  float de, dy_ref;
  y = new_y;
  dy_ref = (new_y_ref - y_ref) / ppars->dt;
  y_ref = new_y_ref;

  last_e = e;
  e = y_ref - y;
  
  // Integral and derivative of the error
  Se += e * ppars->dt;
  de = (e - last_e) / ppars->dt;

  // Limit the Integral to the case where it would saturate the output only with this component
  float KiSe = ppars->Ki * Se;
  if (KiSe > m_max) Se = m_max / ppars->Ki;
  if (KiSe < m_min) Se = m_min / ppars->Ki;
  
  // Calc PID output
  m = ppars->Kc * e + ppars->Ki * Se + ppars->Kd * de + ppars->Kf * y_ref + ppars->Kfd * dy_ref;
 
  // Anti windup
  //if (m > m_max || m < m_min) {
  if ((m > m_max && e > 0) || (m < m_min && e < 0)) {
    // undo integration
    Se -= e * ppars->dt;
  }
  
  // Saturate the output
  if (m > m_max) {
    m = m_max;
  } else if (m < m_min) {
    m = m_min;
  }

  return m;
}
