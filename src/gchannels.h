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

#ifndef GCHANNELS_H
#define GCHANNELS_H

#include <Arduino.h>

#define GCHANNELS_BUF_IN_SIZE 500U
#define GCHANNELS_BUF_OUT_SIZE 500U

#define COMMAND_SIZE 32


typedef struct { 
  char* command;
  float value;
  char* text;
  int index, suffix;
  int command_is(const char* c);
  int command_prefix_is(const char* c);
} command_frame_t;

typedef enum { 
  gs_wait_for_data,
  gs_reading_command,
  gs_reading_token,
} gchannels_state_t;


class gchannels_t
{
  public:
    // Pointer to the function that processes commands
    void (*process_command)(command_frame_t frame);
    void (*send_buffer)(const char *buffer, size_t size);

    char buffer[GCHANNELS_BUF_IN_SIZE];
    char out_buffer[GCHANNELS_BUF_OUT_SIZE];
    int count, out_count;
    
    bool reading_token;
    char last_command[COMMAND_SIZE];

    gchannels_state_t state;
    command_frame_t frame;

    gchannels_t();
    void init(void (*process_command_function)(command_frame_t frame), 
              void (*send_buffer_funtion)(const char* buffer, size_t size)) ;
    void process_char(char b);
    
    void send_char(char b);
    void send_string(const char* str, bool eight_bit_on = false);
    void flush(void);
    void send_command(const char* command, float par);
    void send_command(const char* command, const char* par);

    void send_token(const char* token);
    void send_token(float token);
};


typedef enum { 
  ct_float,
  ct_int,
  ct_string
} command_type_t;

typedef struct {
  command_type_t _type;
  char command[COMMAND_SIZE];
  uint32_t command_hash;
  float* pfvalue;
  int* pivalue;
  char* psvalue;
  int max_string_size;
  bool send, sparse_send, recv, store;
} command_item_t;


#ifndef COMMAND_LIST_SIZE
  #define COMMAND_LIST_SIZE 64
#endif


class commands_list_t
{
  private:
    command_item_t* register_command_common(const char* command_name);
  public:
    command_item_t items[COMMAND_LIST_SIZE];
    int count;
    bool send, sparse_send, recv, store;  // Default values for items
    int send_count, sparse_send_count, recv_count;
    int max_sparce_send, cur_sparce_send;

    commands_list_t();
    command_item_t* register_command(const char* command_name, float* pvalue);
    command_item_t* register_command(const char* command_name, int* pvalue);
    command_item_t* register_command(const char* command_name, char* pvalue, int max_size);

    command_item_t* process_read_command(command_frame_t& frame_data);

    int send_commands(gchannels_t& gchannels);
    int send_sparse_commands(gchannels_t& gchannels);
};

extern int debug;

#endif // GCHANNELS_H