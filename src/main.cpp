#include <Arduino.h>
#include <WiFi.h>
#include "robot.h"
#include "MotorController.h"
#include "pico4drive.h"
#include "PicoEncoder.h"
#include "config.h"

#define NUM_ENCODERS 2
PicoEncoder encoders[NUM_ENCODERS];
pin_size_t encoder_pins[NUM_ENCODERS] = {ENC1_PIN_A, ENC2_PIN_A};



/*------------------------------------------------------------------------------------------------------------ 
                                           VARIABLES DECLARATIONS    
------------------------------------------------------------------------------------------------------------*/
pico4drive_t pico4drive;
void init_control(robot_t &robot);
void control(robot_t &robot);

int count = 0;
unsigned long last_cycle;
char command = 's';//default stopped


/*------------------------------------------------------------------------------------------------------------
                                              Funcoes Auxiliares
-------------------------------------------------------------------------------------------------------------*/

void read_PIO_encoders(void)
{
  encoders[0].update();
  encoders[1].update();
  robot.enc1 = -encoders[0].speed;
  robot.enc2 = encoders[1].speed;
}

void SetWheelsPWM(void)
{
  
}

const char *encToString(uint8_t enc)
{
  switch (enc)
  {
  case ENC_TYPE_NONE:
    return "NONE";
  case ENC_TYPE_TKIP:
    return "WPA";
  case ENC_TYPE_CCMP:
    return "WPA2";
  case ENC_TYPE_AUTO:
    return "AUTO";
  }
  return "UNKN";
}

void wifi_list(void)
{
  Serial.printf("Beginning scan at %d\n", millis());
  int cnt = WiFi.scanNetworks();
  if (!cnt)
  {
    Serial.printf("No networks found\n");
  }
  else
  {
    Serial.printf("Found %d networks\n\n", cnt);
    Serial.printf("%32s %5s %2s %4s\n", "SSID", "ENC", "CH", "RSSI");
    for (int i = 0; i < cnt; i++)
    {
      uint8_t bssid[6];
      WiFi.BSSID(i, bssid);
      Serial.printf("%32s %5s %2d %4d\n", WiFi.SSID(i), encToString(WiFi.encryptionType(i)), WiFi.channel(i), WiFi.RSSI(i));
    }
  }
}

void processSerialCommands()
{
    static String input = "";

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {           // Enter pressed
            if (input.length() > 0) {
                float value = input.toFloat();

                switch (input[0]) {
                    case 'v': case 'V':  robot.setRobotVW(value, 0.0f);
                                         Serial.printf(">>> v = %.3f m/s\n", value); break;
                    case 'w': case 'W':  robot.setRobotVW(0.0f, value);
                                         Serial.printf(">>> ω = %.3f rad/s\n", value); break;
                    case 's': case 'S':  robot.setRobotVW(0.0f, 0.0f);
                                         Serial.println(">>> STOP"); break;
                    default:             robot.setRobotVW(value, 0.0f);
                                         Serial.printf(">>> v = %.3f m/s\n", value); break;
                }
                input = "";
            }
        }
        else {
            input += c;
        }
    }
}


/*------------------------------------------------------------------------------------------------------------
                                              Setup e loop
------------------------------------------------------------------------------------------------------------*/


void setup()
{
  //add Serial Communication:
  Serial.begin();
  Serial.print("System ready");
  
  // Set the pins as input or output as needed
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ENC1_PIN_A, INPUT_PULLUP);
  pinMode(ENC1_PIN_B, INPUT_PULLUP);
  pinMode(ENC2_PIN_A, INPUT_PULLUP);
  pinMode(ENC2_PIN_B, INPUT_PULLUP);

  // Motor driver pins
  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);

  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);


  //pinMode(SOLENOID_PIN_A, OUTPUT);
  //pinMode(SOLENOID_PIN_B, OUTPUT);

  encoders[0].begin(encoder_pins[0]);
  encoders[1].begin(encoder_pins[1]);

  last_cycle = millis();
  pico4drive.init();

  analogReadResolution(10);
}

void loop(){

  //checks how many bytes we have in serial buffer
  if(Serial.available())
  {
    command = Serial.read();//either m(move)/s(stop)
  }

  
  uint32_t curr_time = millis();
  uint32_t cycle_duration = curr_time - last_cycle;

  if(cycle_duration >= (robot.dt*1000))
  {
    last_cycle = curr_time;

    
    if (command == 's')
    {
      //robot.motors.driveMotor(0,0);
      robot.state.setState(STATE_STOP);
      Serial.println("-----------------------Robot Variables------------------------");
      Serial.print("V_req: ");
      Serial.println(robot.v_req);
      Serial.print("W_req:");
      Serial.println(robot.w_req);
    }
    else if(command == 'm')
    {
      //robot.motors.driveMotor(1,1);
      robot.state.setState(STATE_FORWARD);//v_req & w_req
    }
    else if (command == 'o'){
      robot.state.setState(STATE_OUTPUT);
    }

    //definition of robot variables...
    read_PIO_encoders();//enc values
    robot.odometry();//calculate v and w
    
    robot.motors.PIDController_Update();//takes v&w req and computes the PID
  }
  
}