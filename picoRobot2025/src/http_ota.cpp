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
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <hardware/flash.h>

#include "http_ota.h"

http_ota_t http_ota;

void update_started(void) 
{
  Serial.println("CALLBACK:  HTTP update process started");
  Serial.flush();
}

void update_finished(void) 
{
  Serial.println("CALLBACK:  HTTP update process finished");
  Serial.flush();
}

void update_progress(int cur, int total) 
{
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
  Serial.flush();
}

void update_error(int err) 
{
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
  Serial.flush();
}

http_ota_t::http_ota_t()
{
  requested = false;
}


void http_ota_t::handle(void)
{
  if (requested) {
    WiFiClient otaclient;
    httpUpdate.onStart(update_started);
    httpUpdate.onEnd(update_finished);
    httpUpdate.onProgress(update_progress);
    httpUpdate.onError(update_error);
    Serial.println("msg: HTTP update process starting");
    Serial.print("msg: ");
    Serial.println(host);
    Serial.print("msg: ");
    Serial.println(uri);
    Serial.flush();

    // t_httpUpdate_return ret = httpUpdate.update(client, "http://server/file.bin");
    //t_httpUpdate_return ret = httpUpdate.update(otaclient, host, 80, uri, "");
    t_httpUpdate_return ret = httpUpdate.update(otaclient, host + uri);

    switch(ret) {
      case HTTP_UPDATE_FAILED:
          Serial.println("msg: Update failed.");
          Serial.flush();
          requested = false;
          break;
      case HTTP_UPDATE_NO_UPDATES:
          Serial.println("msg: Update no Update.");
          Serial.flush();
          requested = false;
          break;
      case HTTP_UPDATE_OK:
          Serial.println("msg: Update ok."); // may not be called since we reboot the RP2040
          Serial.flush();
          break; 
    }   
  }
}
