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
#include "gchannels.h"
#include <math.h>

int command_frame_t::command_is(const char* c)
{
  return !strncmp(command, c, GCHANNELS_BUF_IN_SIZE);
}

int command_frame_t::command_prefix_is(const char* c)
{
  suffix = 0;
  int i = min(strlen(c), GCHANNELS_BUF_IN_SIZE - 1);
  if (!strncmp(command, c, i)) {
    suffix = atoi(command + i);
    return 1;
  } else {
    return 0;
  }
}


gchannels_t::gchannels_t()
{
  count = 0;
  state = gs_wait_for_data;
  memset(buffer, 0, sizeof(buffer));
  memset(last_command, 0, sizeof(last_command));
    
  process_command = NULL;
  send_buffer = NULL;
  
  frame.command = buffer;
  frame.text = buffer;
  frame.value = 0;
}


void gchannels_t::init(void (*process_command_function)(command_frame_t frame), void (*send_buffer_funtion)(const char *buffer, size_t size))
{
  process_command = process_command_function;
  send_buffer = send_buffer_funtion;
}

void gchannels_t::flush(void)
{
  if (send_buffer) (*send_buffer)(out_buffer, out_count);
  out_count = 0;
  memset(out_buffer, 0, sizeof(out_buffer));
}

void gchannels_t::send_char(char b)
{
  // If possible store the byte in the buffer
  if (out_count < GCHANNELS_BUF_OUT_SIZE) {
    out_buffer[out_count] = b;
    out_count++;
  }
  // If the buffer is full then send it
  if (out_count == GCHANNELS_BUF_OUT_SIZE) {  
    flush();
  }

}

void gchannels_t::send_string(const char* str, bool eight_bit_on)
{
  byte mask = 0;
  if (eight_bit_on) mask = 0x80;
  
  int i = 0;
  while(str[i]) {
    send_char(str[i] | mask);
    i++;
  }  
}

void gchannels_t::send_command(const char* command, const char* par)
{
  send_string(command);
  send_char(' ');
  
  send_string(par);
  send_char(';');
  send_char(' ');
}


void gchannels_t::send_command(const char* command, float par)
{
  char scratch[32];
  snprintf(scratch, 32, "%.6g", par);
  send_command(command, scratch);
}


void gchannels_t::send_token(const char* token)
{
  send_string(token);
  send_char(';');
}


void gchannels_t::send_token(float token)
{
  char scratch[32];
  snprintf(scratch, 32, "%.6g", token);
  send_string(scratch);
  send_char(';');
}



void gchannels_t::process_char(char b)
{
  if (state == gs_wait_for_data && isalpha(b))  { // A command allways starts with a letter
    state = gs_reading_command;
    buffer[0] = b;
    count = 1;
    reading_token = false;

  } else if (state == gs_wait_for_data && (isdigit(b) || b == '-'))  { // A token is a number and starts with a digit or a minus  
    state = gs_reading_token;
    buffer[0] = b;
    count = 1;
    reading_token = true;
  
  } else if ((state == gs_reading_command || state == gs_reading_token) && b == 0x08)  { // BS (Backspace key received)  
    if (count > 0) {
      count--;
      buffer[count] = 0;  
    }    

  } else if (state == gs_reading_command && (b == 0x0A || b == 0x0D || b == ';'))  { // LF or CR (enter key received) or ';'
    // Now we can process the buffer
    if (count != 0) {
      buffer[count] = 0; // Guarantee to null terminate the string
      frame.command = buffer; // The command starts at the begining
      
      // Find the first space to separate the command from the text/value
      frame.text = buffer;
      while(*frame.text) {
        if (*frame.text == ' ' || *frame.text == ':') {  // If the first space found or a ':'
          *frame.text = 0; // Command ends here;
          frame.text++;    // "text" starts here
          while(*frame.text == ' ') frame.text++; // or not, we should skip spaces
          break;  // And we can stop scanning
        }
        frame.text++;
      }
      frame.index = 0;
      strlcpy(last_command, buffer, COMMAND_SIZE); // Store the command

      //Serial.print(" frame.text: ");
      //Serial.print(frame.text);
      // for the numerical parameter try to get the value from the text
      frame.value = atof((const char*)frame.text);

      if (process_command)         // If "process_command" is not null
        (*process_command)(frame); // Do something with the pair (command, value)      

      // Reset the buffer
      count = 0;
      memset(buffer, 0, sizeof(buffer));
      frame.text = buffer;
    } 
    state = gs_wait_for_data;

  } else if (state == gs_reading_token && (b == 0x0A || b == 0x0D || b == ';'))  { // LF or CR (enter key received) or ';'
    // Now we can process the buffer
    if (count != 0) {
      buffer[count] = 0; // Guarantee to null terminate the string
      
      frame.command = last_command; // Recycle the last command
      frame.text = buffer; // The token starts at the begining
      frame.index++;
      
      //Serial.print(" frame.text: ");
      //Serial.print(frame.text);
      // for the numerical parameter try to get the value from the text
      frame.value = atof((const char*)frame.text);

      if (process_command)         // If "process_command" is not null
        (*process_command)(frame); // Do something with the pair (command, value)      

      // Reset the buffer
      count = 0;
      memset(buffer, 0, sizeof(buffer));
    } 
    state = gs_wait_for_data;
  
  } else if ((state == gs_reading_command || state == gs_reading_token )  && count < GCHANNELS_BUF_IN_SIZE - 1)  { // A new char can be read
    buffer[count] = b;  // Store byte in the buffer
    count++;
  }

}


static uint32_t MurmurOAAT_32(const char* str, uint32_t h)
{
    // One-byte-at-a-time hash based on Murmur's mix
    // Source: https://github.com/aappleby/smhasher/blob/master/src/Hashes.cpp
    for (; *str; ++str) {
        h ^= *str;
        h *= 0x5bd1e995;
        h ^= h >> 15;
    }
    return h;
}


commands_list_t::commands_list_t()
{
  count = 0;
  max_sparce_send = 2; 
  cur_sparce_send = 0;  

  // Default values
  send = true;
  sparse_send = false;
  recv = true;
  store = false;
}

command_item_t* commands_list_t::register_command(const char* command_name, float* pvalue)
{
  if (count >= COMMAND_LIST_SIZE) return NULL;

  items[count]._type = ct_float;
  items[count].pfvalue = pvalue;

  return register_command_common(command_name);
}


command_item_t* commands_list_t::register_command(const char* command_name, int* pvalue)
{
  if (count >= COMMAND_LIST_SIZE) return NULL;

  items[count]._type = ct_int;
  items[count].pivalue = pvalue;
  
  return register_command_common(command_name);
}

command_item_t* commands_list_t::register_command(const char* command_name, char* pvalue, int max_size)
{
  if (count >= COMMAND_LIST_SIZE) return NULL;

  items[count]._type = ct_string;
  items[count].psvalue = pvalue;
  items[count].max_string_size = max_size;

  return register_command_common(command_name);
}


command_item_t* commands_list_t::register_command_common(const char* command_name)
{

  strlcpy(items[count].command, command_name, COMMAND_SIZE);
  items[count].command_hash = MurmurOAAT_32(items[count].command, 0x5EED);

  items[count].send = send;
  items[count].recv = recv;
  items[count].sparse_send = sparse_send;
  items[count].store = store;

  count++;
  return &(items[count - 1]);
}

command_item_t* commands_list_t::process_read_command(command_frame_t& frame_data)
{
  int i, idx;
  char* endptr;

  uint32_t hash = MurmurOAAT_32(frame_data.command, 0x5EED);

  // Find if command is in the list
  idx = -1;
  for (i = 0; i < count; i++) {
    if (!items[i].recv) continue;      // Skip if is not a read command
    if (items[i].command_hash == hash) { 
      if (frame_data.command_is(items[i].command)) { // Test if the command is the right one (no blind trust in the hash)
        idx = i;
        break;
      }
    }
  }
  if (idx == -1) return NULL;
  debug = 1;

  // Process it: assign the command argument to the respetive pointed variable
  if (items[idx]._type == ct_float) {
    *(items[idx].pfvalue) = frame_data.value;
  } else if (items[idx]._type == ct_int) {
    debug = 2;
    *(items[idx].pivalue) = strtol(frame_data.text, &endptr, 10);
  } else if (items[idx]._type == ct_string) {
    strlcpy(items[idx].psvalue, frame_data.text, items[idx].max_string_size);
  }

  return &(items[idx]); // return the command from the list
}


int commands_list_t::send_commands(gchannels_t& gchannels)
{
  int i;
  for (i = 0; i < count; i++) {
    if(!items[i].send) continue;
    if (items[i]._type == ct_float) gchannels.send_command(items[i].command, *(items[i].pfvalue));
    else if (items[i]._type == ct_int) gchannels.send_command(items[i].command, *(items[i].pivalue));
    else if (items[i]._type == ct_string) gchannels.send_command(items[i].command, items[i].psvalue);
    send_count++;
  }
  return send_count;
}


int commands_list_t::send_sparse_commands(gchannels_t& gchannels)
{
  int i, loop;

  loop = 0;
  sparse_send_count = 0;
  
  while(1) {
    i = cur_sparce_send;
    if(items[i].sparse_send) {
      if (items[i]._type == ct_float) gchannels.send_command(items[i].command, *(items[i].pfvalue));
      else if (items[i]._type == ct_int) gchannels.send_command(items[i].command, *(items[i].pivalue));
      else if (items[i]._type == ct_string) gchannels.send_command(items[i].command, items[i].psvalue);
      
      sparse_send_count++;
      if (sparse_send_count >= max_sparce_send) break;  // Already sent sparse_send quota
    }  
    cur_sparce_send++;                                    // Prepare next possible command
    if (cur_sparce_send >= count) cur_sparce_send = 0;    // in a circular way

    loop++;
    if (loop >= count) break;  // Already did a full loop
  }
  
  return sparse_send_count;
}


