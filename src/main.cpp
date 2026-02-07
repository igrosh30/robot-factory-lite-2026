#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <LittleFS.h>

// Your robot includes
#include "robot.h"
#include "MotorController.h"
#include "pico4drive.h"
#include "PicoEncoder.h"
#include "config.h"
#include "state_machines.h"
#include "gchannels.h"
#include "file_gchannels.h"

// ================================================================
//                      YOUR ROBOT CONFIGURATION
// ================================================================
#define NUM_ENCODERS 2
PicoEncoder encoders[NUM_ENCODERS];
pin_size_t encoder_pins[NUM_ENCODERS] = {ENC1_PIN_A, ENC2_PIN_A};

// Global robot instance
pico4drive_t pico4drive;
robot_t robot(pico4drive);

// ================================================================
//                      WIFI/UDP CONFIGURATION
// ================================================================
#define max_wifi_str 32
char ssid[max_wifi_str];
char password[max_wifi_str];
const char* fname_wifi = "/wifi.txt";

int udp_on = 0, ip_on = 0;
WiFiUDP Udp;
unsigned int localUdpPort = 4224;

#define UDP_MAX_SIZE 512
uint8_t UdpInPacket[UDP_MAX_SIZE];
uint8_t UdpOutPacket[UDP_MAX_SIZE];
int UdpBufferSize = UDP_MAX_SIZE;

// ================================================================
//                      COMROBOT COMMUNICATION
// ================================================================
gchannels_t udp_commands;
gchannels_t serial_commands;
commands_list_t pars_list;

const char* pars_fname = "pars.cfg";
bool load_pars_requested = false;
static int state_cmd_value; 

// ================================================================
//                      TIMING & CONTROL
// ================================================================
uint32_t interval, last_cycle;
uint32_t loop_micros;
uint32_t cycle_count;
int debug_level = 1;

// ================================================================
//                      HELPER FUNCTIONS
// ================================================================
void read_PIO_encoders(void) {
    encoders[0].update();
    encoders[1].update();
    robot.enc1 = -encoders[0].speed;
    robot.enc2 = encoders[1].speed;
}


// ================================================================
//                      COMROBOT HELPER FUNCTIONS
// ================================================================
void serial_write(const char *buffer, size_t size) {
    Serial.write(buffer, size);
    if (udp_on) {
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(buffer, size);
        Udp.endPacket();  
    }
}

// ================================================================
//                      SET INTERVAL FUNCTION
// ================================================================
void set_interval(float new_interval) {
    interval = new_interval * 1000000L;   // In microseconds
    robot.dt = new_interval;   // In seconds
}

// ================================================================
//                      COMMAND PROCESSING - SIMPLIFIED
// ================================================================
void process_command(command_frame_t frame)
{
  pars_list.process_read_command(frame);

  if (frame.command_is("stp")) {
    robot.stoped = true;
    robot.setRobotVW(0, 0);
    serial_commands.send_command("msg", "Robot stopped");
  }
  else if (frame.command_is("strt")) {
    robot.stoped = false;
    serial_commands.send_command("msg", "Robot started");
  }
  else if (frame.command_is("st")) { // Update your state machine with the value that pars_list already set
    if (state_cmd_value >= 0 ) {//add && state_cmd_value <= 7 if I want to restrit the states
      state_machine.robotState = (currentState)state_cmd_value;
      serial_commands.send_command("msg", "State changed");
    }
  }
  else if (frame.command_is("wifi")) { 
    if (frame.value == 1) {
      if (WiFi.connected()) WiFi.end();
      WiFi.begin(ssid, password); 
    } else if (frame.value == 0) {
      WiFi.end();
      ip_on = 0;
      udp_on = 0;
    }
  } 
  else if (frame.command_is("scan")) { 
    int n = WiFi.scanNetworks();
    serial_commands.send_command("msg", (String("Found ") + n + " networks").c_str());
    for (int i = 0; i < n; i++) {
      String ssid_info = String(i+1) + ": " + WiFi.SSID(i) + 
                       " Ch:" + WiFi.channel(i) + 
                       " RSSI:" + WiFi.RSSI(i);
      serial_commands.send_command("msg", ssid_info.c_str());
    }
  }
  else if (frame.command_is("cat")) { 
    send_file(frame.text, serial_commands, true);
  } 
  else if (frame.command_is("udp")) {  
    if (udp_on) {
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(UdpInPacket, frame.value);
      Udp.endPacket();  
      serial_commands.send_command("msg", "UDP echo sent");
    }
  } 
  else if (frame.command_is("dl")) { 
    debug_level = frame.value;
    serial_commands.send_command("msg", (String("Debug level: ") + debug_level).c_str());
  }
  // Add YOUR specific commands here...
}

// ================================================================
//                      SETUP FUNCTION
// ================================================================
void setup() {
    // ========== PIN SETUP ==========
    pinMode(LED_BUILTIN, OUTPUT);

    // Encoder pins
    pinMode(ENC1_PIN_A, INPUT_PULLUP);
    pinMode(ENC1_PIN_B, INPUT_PULLUP);
    pinMode(ENC2_PIN_A, INPUT_PULLUP);
    pinMode(ENC2_PIN_B, INPUT_PULLUP);

    // Motor driver pins
    pinMode(MOTOR1A_PIN, OUTPUT);
    pinMode(MOTOR1B_PIN, OUTPUT);
    pinMode(MOTOR2A_PIN, OUTPUT);
    pinMode(MOTOR2B_PIN, OUTPUT);

    // ========== INITIALIZE HARDWARE ==========
    pico4drive.init();  
    analogReadResolution(10);

    // Initialize YOUR robot
    robot.frontSensor.init();
    robot.actuators.init();

    // Initialize state machine
    state_machine.robotState = Start;  
    state_cmd_value = (int)Start;    
    
    
    // Initialize encoders
    encoders[0].begin(encoder_pins[0]);
    encoders[1].begin(encoder_pins[1]);

    // ========== COMROBOT COMMUNICATION SETUP ==========
    Serial.begin(115200);
    delay(1000);  // Give time for serial to initialize

    // ========== REGISTER COMMANDS ==========
    pars_list.max_sparce_send = 4;
    
    // Version (important for ComRobot)
    int version = 1001;  // Your version number
    //pars_list.register_command("ver", &version);
    
    // Robot control commands
    pars_list.register_command("vreq", &robot.v_req);
    pars_list.register_command("wreq", &robot.w_req);
    //pars_list.register_command("mode", (int*)&robot.control_mode);
    
    // PID parameters
    //pars_list.register_command("kp1", &robot.motors.kp1);
    //pars_list.register_command("ki1", &robot.motors.ki1);
    //pars_list.register_command("kp2", &robot.motors.kp2);
    //pars_list.register_command("ki2", &robot.motors.ki2);

    
    // Line following parameters
    pars_list.register_command("kl", &robot.frontSensor.kl);
    //pars_list.register_command("fv", &robot.follow_v);
    //pars_list.register_command("fk", &robot.follow_k);
    
    // State machine control - use the separate variable
    pars_list.register_command("st", &state_cmd_value);

    // Actuator control
    //pars_list.register_command("mg", (int*)&robot.actuators.isMagnetOn);
    
    // WiFi configuration
    // CHANGE THESE TO YOUR WIFI!
    strcpy(ssid, "5DPO-NETWORK");      
    strcpy(password, "5dpo5dpo");      
    pars_list.register_command("ssid", ssid, max_wifi_str);
    pars_list.register_command("pass", password, max_wifi_str)->sparse_send = false;

    // Initialize communication channels
    udp_commands.init(process_command, serial_write);
    serial_commands.init(process_command, serial_write);
    robot.pchannels = &serial_commands;  // Important for robot to send messages

    // ========== FILESYSTEM ==========
    LittleFS.begin();
    load_commands(pars_fname, serial_commands);

    // ========== WIFI SETUP ==========
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    for (int i = 0; i < 200 && WiFi.status() != WL_CONNECTED; i++) {
        Serial.print(".");
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connection failed. Running offline.");
    }

    Serial.println();

    // ========== TIMING SETUP ==========
    float control_interval = 0.04;  // In seconds
    set_interval(control_interval);
    
    Serial.println("Setup complete. Ready for commands.");
    serial_commands.send_command("msg", "Robot ready");
    serial_commands.send_command("IP", WiFi.localIP().toString().c_str());
}

// ================================================================
//                      LOOP FUNCTION
// ================================================================
void loop() { 
    // ========== WIFI/UDP HANDLING ==========
    if (WiFi.connected() && !ip_on) {
        serial_commands.send_command("msg", 
            (String("Connected to WiFi: ") + WiFi.SSID()).c_str());
        serial_commands.send_command("msg", 
            (String("IP: ") + WiFi.localIP().toString()).c_str());
        
        ip_on = Udp.begin(localUdpPort);
        if (ip_on) {
            serial_commands.send_command("msg", 
                (String("UDP port: ") + localUdpPort).c_str());
        }
    }

    // Handle incoming UDP packets
    if (ip_on) {
        int packetSize = Udp.parsePacket();
        if (packetSize) {
            udp_on = 1;
            int len = Udp.read(UdpInPacket, UDP_MAX_SIZE - 1);
            if (len > 0) {
                UdpInPacket[len] = 0;
                // Process each character
                for (int i = 0; i < len; i++) {
                    udp_commands.process_char(UdpInPacket[i]);
                }
            }
        }      
    }

    // ========== SERIAL COMMANDS ==========
    while (Serial.available()) {  
        uint8_t b = Serial.read();    
        serial_commands.process_char(b);
    }

    // ========== LOAD PARAMETERS IF REQUESTED ==========
    if (load_pars_requested) {
        load_commands(pars_fname, serial_commands);
        load_pars_requested = false;
        serial_commands.send_command("msg", "Parameters loaded");
    }

    // ========== YOUR MAIN CONTROL LOOP ==========
    uint32_t now = micros();
    uint32_t delta = now - last_cycle; 
    
    if (delta >= interval ){//&& !robot.stoped need to add?
        last_cycle = now;
        loop_micros = micros();
        cycle_count++;

        // Read Data
        read_PIO_encoders();
        robot.odometry();
        robot.actuators.update();//update magnet and read switch 
        //float ir_error = robot.frontSensor.getLineError();
        
        
        // Run YOUR state machine
        state_machine.runStateMachine4Testing(robot);

        // Update motors
        robot.motors.PIDController_Update();

        // ========== SEND data TO COMROBOT ==========
        if (debug_level > 0) {
            // Timing
            //serial_commands.send_command("dte", delta / 1000.0f);  // in ms
            
            // Robot state
            //serial_commands.send_command("xe", robot.xe);
            //serial_commands.send_command("ye", robot.ye);
            //serial_commands.send_command("te", robot.thetae);
            serial_commands.send_command("ve", robot.ve);
            serial_commands.send_command("we", robot.we);
            serial_commands.send_command("vreq", robot.v_req);
            serial_commands.send_command("wreq", robot.w_req);
            
            // Motor voltages
            //serial_commands.send_command("u1", robot.u1);
            //serial_commands.send_command("u2", robot.u2);
            
            // Sensor data
            serial_commands.send_command("ir0", robot.frontSensor.IR_Values[0]);
            serial_commands.send_command("ir1", robot.frontSensor.IR_Values[1]);
            serial_commands.send_command("ir2", robot.frontSensor.IR_Values[2]);
            serial_commands.send_command("ir3", robot.frontSensor.IR_Values[3]);
            serial_commands.send_command("ir4", robot.frontSensor.IR_Values[4]);

            serial_commands.send_command("irn0", robot.frontSensor.IR_norm[0]);
            serial_commands.send_command("irn1", robot.frontSensor.IR_norm[1]);
            serial_commands.send_command("irn2", robot.frontSensor.IR_norm[2]);
            serial_commands.send_command("irn3", robot.frontSensor.IR_norm[3]);
            serial_commands.send_command("irn4", robot.frontSensor.IR_norm[4]);

            //serial_commands.send_command("irn01", robot.frontSensor.IR_norm1[0]);
            //serial_commands.send_command("irn11", robot.frontSensor.IR_norm1[1]);
            //serial_commands.send_command("irn21", robot.frontSensor.IR_norm1[2]);
            //serial_commands.send_command("irn31", robot.frontSensor.IR_norm1[3]);
            //serial_commands.send_command("irn41", robot.frontSensor.IR_norm1[4]);

            serial_commands.send_command("irma0", robot.frontSensor.maxValues[0]);
            serial_commands.send_command("irma1", robot.frontSensor.maxValues[1]);
            serial_commands.send_command("irma2", robot.frontSensor.maxValues[2]);
            serial_commands.send_command("irma3", robot.frontSensor.maxValues[3]);
            serial_commands.send_command("irma4", robot.frontSensor.maxValues[4]);

            serial_commands.send_command("irmi0", robot.frontSensor.minValues[0]);
            serial_commands.send_command("irmi1", robot.frontSensor.minValues[1]);
            serial_commands.send_command("irmi2", robot.frontSensor.minValues[2]);
            serial_commands.send_command("irmi3", robot.frontSensor.minValues[3]);
            serial_commands.send_command("irmi4", robot.frontSensor.minValues[4]);
            
            serial_commands.send_command("erl", robot.frontSensor.erro);
            serial_commands.send_command("fl_ir", robot.frontSensor.flagFound);
            
            // Encoder data
            //serial_commands.send_command("e1", robot.enc1);
            //serial_commands.send_command("e2", robot.enc2);
            //serial_commands.send_command("w1", robot.w1e);
            //serial_commands.send_command("w2", robot.w2e);
            
            // State machine - sync the variable
            state_cmd_value = (int)state_machine.robotState;
            serial_commands.send_command("st", (float)state_cmd_value);
            
            // Actuators
            //serial_commands.send_command("mg", robot.actuators.isMagnetOn ? 1 : 0);
            //serial_commands.send_command("sw", digitalRead(SWITCH_PIN));
            //serial_commands.send_command("fl", state_machine.flag);

            
            // WiFi status (like teacher's code)
            if (cycle_count % 5 == 0) {  // Send periodically
                if (WiFi.connected()) {
                    serial_commands.send_command("IP", WiFi.localIP().toString().c_str());
                    serial_commands.send_command("rssi", (float)WiFi.RSSI());
                }
                if (udp_on) {
                    serial_commands.send_command("IPR", Udp.remoteIP().toString().c_str());
                }
            }            

            // Send sparse parameters
            pars_list.send_sparse_commands(serial_commands);

            // Debug info
            serial_commands.send_command("loop", micros() - loop_micros);
            serial_commands.flush();
        }
    }
    
    // Heartbeat LED
    static unsigned long last_blink = 0;
    if (millis() - last_blink > 500) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        last_blink = millis();
    }
}