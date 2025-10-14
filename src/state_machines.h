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

#ifndef STATE_MACHINES_H
#define STATE_MACHINES_H

#include <stdint.h>

class state_machine_t
{
  public:

  int state, new_state, prev_state;

  // tes - time entering state
  // tis - time in state
  uint32_t tes_ms, tis_ms;
  float tis; 
  uint32_t actions_count;

  state_machine_t();
  void set_new_state(int astate);
  void update_state(void);
  void force_new_state(int astate);

  
  float time_since(uint32_t when);
  
  void calc_next_state(void);
  virtual void next_state_rules(void) {};

  void do_enter_state_actions(void);
  virtual void enter_state_actions_rules(void) {};

  void do_state_actions(void);
  virtual void state_actions_rules(void) {};

  void step(void);
};


#ifndef MAX_STATE_MACHINES
#define MAX_STATE_MACHINES 32
#endif 

typedef state_machine_t* pstate_machine_t;

class state_machines_t
{
  public:
  pstate_machine_t psm[MAX_STATE_MACHINES];
  int count;

  int register_state_machine(pstate_machine_t new_psm);

  void calc_next_states(void);
  void update_states(void);

  void do_enter_states_actions(void);
  void do_states_actions(void);

  void step(void);
};


extern state_machines_t state_machines;

#endif // STATE_MACHINES_H
