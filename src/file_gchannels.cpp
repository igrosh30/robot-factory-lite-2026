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

#include <Arduino.h>
#include <LittleFS.h>
#include "gchannels.h"
#include "file_gchannels.h"

void send_file(const char* filename, gchannels_t& gchannels, bool log_high)
{
  File f;
  f = LittleFS.open(filename, "r");
  if (!f) {
    gchannels.send_command("err", filename);
    return;
  }

  gchannels.flush();
  Serial.flush();

  int c;
  byte b, mask;
  if (log_high) mask = 0x80;
  else mask = 0;

  while(1) {
    c = f.read(&b, 1);
    if (c != 1) break;
    gchannels.send_char(b | mask);
  }
  f.close();

  gchannels.flush();
  Serial.flush();  
}

int save_commands(const char* filename, commands_list_t& cl, gchannels_t& gchannels)
{
  File f;

  f = LittleFS.open(filename, "w+");
  if (!f) {
    gchannels.send_command("err", filename);
    return 0;
  }

  int i;
  for (i = 0; i < cl.count; i++) {
    if(!cl.items[i].store) continue;
    if (cl.items[i]._type == ct_float) f.printf("%s %.6g\n", cl.items[i].command, *(cl.items[i].pfvalue));
    else if (cl.items[i]._type == ct_int) f.printf("%s %ld\n", cl.items[i].command, *(cl.items[i].pivalue));
    else if (cl.items[i]._type == ct_string) f.printf("%s %s\n", cl.items[i].command, cl.items[i].psvalue);
  }
  f.close();
  
  return 1;
}


int load_commands(const char* filename, gchannels_t& gchannels)
{
  File f;
  f = LittleFS.open(filename, "r");
  if (!f) {
    gchannels.send_command("err", filename);
    return 0;
  }

  int c;
  byte b;

  while(1) {
    c = f.read(&b, 1);
    if (c != 1) break;
    gchannels.process_char(b);
  }
  f.close();
  return 1;
}
