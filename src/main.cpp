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
#include "fsm_round1.h"
#include "fsm_round2.h"
#include "fsm_round3.h"
#include "COM_fsm.h"
// ================================================================
//                      YOUR ROBOT CONFIGURATION
// ================================================================
#define NUM_ENCODERS 2
PicoEncoder encoders[NUM_ENCODERS];
pin_size_t encoder_pins[NUM_ENCODERS] = {ENC1_PIN_A, ENC2_PIN_A};

// Global robot instance
pico4drive_t pico4drive;
robot_t robot(pico4drive);
//fsm_round1 fsm(robot);
//fsm_round2 fsm(robot);
fsm_round3 fsm(robot);
//fsm_COM fsm(robot);


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
#ifdef CONFIG_H12_CANNELS
bool hc12_configured = false; 
void forceHC12Config() {
    Serial.println("\n--- STARTING HC-12 AUTO-CONFIG ---");
    
    // HC-12 AT mode always communicates at 9600 baud
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(9600);
    delay(1000); // Give the module a second to wake up

    // 1. Factory Reset
    Serial.println("Sending AT+DEFAULT...");
    Serial1.print("AT+DEFAULT");
    delay(500);
    while(Serial1.available()) Serial.write(Serial1.read());
    Serial.println();

    // 2. Force 9600 Baud
    Serial.println("Sending AT+B9600...");
    Serial1.print("AT+B9600");
    delay(500);
    while(Serial1.available()) Serial.write(Serial1.read());
    Serial.println();

    // 3. Set Channel 86
    Serial.println("Sending AT+C080...");
    Serial1.print("AT+C080");
    delay(500);
    while(Serial1.available()) Serial.write(Serial1.read());
    Serial.println();

    // 4. Verify Settings
    Serial.println("Sending AT+RX...");
    Serial1.print("AT+RX");
    delay(500);
    while(Serial1.available()) Serial.write(Serial1.read());
    Serial.println("\n--- CONFIGURATION COMPLETE! ---");
}
#endif

void read_PIO_encoders(void) {
    encoders[0].update();
    encoders[1].update();
    robot.enc1 = -encoders[0].speed;// why menos?
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
      fsm.force_state((fsm_state)(int)frame.value);
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
    // ========== COMROBOT COMMUNICATION SETUP ==========
    Serial.begin(115200);
    delay(1000);// Give time for serial to initialize


    pico4drive.init();  
    analogReadResolution(10);

    // Initialize YOUR robot
    robot.front.sensor.init();
    //robot.back.sensor.init();
    robot.front.actuators.init();
    
    // Initialize encoders
    encoders[0].begin(encoder_pins[0]);
    encoders[1].begin(encoder_pins[1]);

   //COM init:
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(9600);
    robot.init_COM(&Serial1);


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
    pars_list.register_command("xe", &robot.xe);
    pars_list.register_command("ye", &robot.ye);
    pars_list.register_command("te", &robot.thetae);

    pars_list.register_command("kp1", &robot.motors.kp1);
    pars_list.register_command("ki1", &robot.motors.ki1);
    pars_list.register_command("kp2", &robot.motors.kp2);
    pars_list.register_command("ki2", &robot.motors.ki2);

    
    //pars_list.register_command("redpath", &fsm.path_strategy);
    //pars_list.register_command("d_slot", &fsm.currentBox);
    
    // Line following parameters
    pars_list.register_command("kl", &robot.front.sensor.kl);
    
    //COM debug:
    pars_list.register_command("s_debug", &robot.send_debug);
    pars_list.register_command("cmdID", &robot.cmdID_send_debug);

    //pars_list.register_command("dosend", &robot.send_debug);    

    //pars_list.register_command("fv", &robot.follow_v);
    //pars_list.register_command("fk", &robot.follow_k);
    
    // State machine control - use the separate variable
    //pars_list.register_command("st", &fsm.state);

    // Actuator control
    pars_list.register_command("mgf", (int*)&robot.front.actuators.isMagnetOn);
    
    // WiFi configuration
    // CHANGE THESE TO YOUR WIFI!
    strcpy(ssid, "FEUP-I-108");      
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
    
    for (int i = 0; i < 100 && WiFi.status() != WL_CONNECTED; i++) {
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
        
        //robot pose update
        read_PIO_encoders();
        robot.odometry();
        
        //Actuators Update, Switches&Magnets
        robot.front.actuators.update();
        //robot.back.actuators.update();
        robot.front.sensor.readValues();
        //robot.back.sensor.readValues();
        
        if(DEBUG_LEVEL == 3) // let's only do COM DEBUG! 
        {
            robot.updateComState();
            fsm.step();
        }
        else{
            robot.updateComState();
            fsm.step();
            robot.motors.PIDController_Update();
        } 
        // Put this at the very top of your loop!
        

        #ifdef CONFIG_H12_CANNELS
        if (!hc12_configured) {
            delay(4000); // Wait 4 full seconds to let you open the Serial Monitor!
            forceHC12Config();
            hc12_configured = true; // Never run this again
        }
        #endif
            


        // ========== SEND data TO COMROBOT ==========
        if (debug_level > 0) {
            // Timing
            //serial_commands.send_command("dte", delta / 1000.0f);  // in ms
            
            // Robot state
            serial_commands.send_command("xe", robot.xe);
            serial_commands.send_command("ye", robot.ye);
            serial_commands.send_command("te", robot.thetae);
            
            serial_commands.send_command("vreq", robot.v_req);
            serial_commands.send_command("wreq", robot.w_req);
            serial_commands.send_command("theta", robot.thetae);
    

            

            #if (DEBUG_LEVEL != 3)
            #ifdef  ROBOT_MASTER
            serial_commands.send_command("sBlBox", fsm.SLAVE_blueBox);    
            #endif   
            serial_commands.send_command("b_idx", fsm.current_box_index);
            serial_commands.send_command("p_slot", fsm.currentBox.pick_slot);
            serial_commands.send_command("d_slot", fsm.currentBox.drop_slot);
            serial_commands.send_command("boxColor", fsm.currentBox.color);

            serial_commands.send_command("b_box", fsm.total_blues);
            serial_commands.send_command("g_box", fsm.total_greens);
            #endif    
            #ifdef ROBOT_SLAVE_01
            serial_commands.send_command("boxA", fsm.processBox_MachineA);
            #endif
            #ifdef ROBOT_SLAVE_00
            serial_commands.send_command("boxB", fsm.processBox_MachineB);
            #endif
            
            serial_commands.send_command("ve", robot.ve);
            serial_commands.send_command("we", robot.we);
            //serial_commands.send_command("redpath", fsm.path_strategy);

            //COM debug:
            serial_commands.send_command("st_com", uint8_t(robot.currentComState));

            //serial_commands.send_command("d1_slot", fsm.drop_slot);
            //serial_commands.send_command("p1_slot", fsm.pick_slot);
            
            //Send the u1 and u2 to drive the motors:
             
            /*
            serial_commands.send_command("e1", robot.enc1);
            serial_commands.send_command("e2", robot.enc2);

            serial_commands.send_command("w1e",robot.w1e);
            serial_commands.send_command("w2e",robot.w2e);

            serial_commands.send_command("v1e",robot.v1e);
            serial_commands.send_command("v2e",robot.v2e);

            serial_commands.send_command("u1", robot.motors.u_send.u1);
            serial_commands.send_command("u2", robot.motors.u_send.u1);
            serial_commands.send_command("kp1", robot.motors.kp1);
            serial_commands.send_command("ki1", robot.motors.ki1);
            serial_commands.send_command("kp2", robot.motors.kp2);
            serial_commands.send_command("ki2", robot.motors.ki2);

            serial_commands.send_command("er1", robot.motors.e1);
            serial_commands.send_command("er2", robot.motors.e2);*/




            //Back Sensor data
            /*
*/
            
            
            //Front Sensor data
            
            serial_commands.send_command("erf", robot.front.sensor.erro);
            //serial_commands.send_command("cntf", robot.front.sensor.countIntersections);
            serial_commands.send_command("fl_irf", robot.front.sensor.flagFound);
            serial_commands.send_command("ir0", robot.front.sensor.IR_Values[0]);
            serial_commands.send_command("ir1", robot.front.sensor.IR_Values[1]);
            serial_commands.send_command("ir2", robot.front.sensor.IR_Values[2]);
            serial_commands.send_command("ir3", robot.front.sensor.IR_Values[3]);
            serial_commands.send_command("ir4", robot.front.sensor.IR_Values[4]);

           
            serial_commands.send_command("irn0", robot.front.sensor.IR_norm[0]);
            serial_commands.send_command("irn1", robot.front.sensor.IR_norm[1]);
            serial_commands.send_command("irn2", robot.front.sensor.IR_norm[2]);
            serial_commands.send_command("irn3", robot.front.sensor.IR_norm[3]);
            serial_commands.send_command("irn4", robot.front.sensor.IR_norm[4]);

            serial_commands.send_command("irma0", robot.front.sensor.maxValues[0]);
            serial_commands.send_command("irma1", robot.front.sensor.maxValues[1]);
            serial_commands.send_command("irma2", robot.front.sensor.maxValues[2]);
            serial_commands.send_command("irma3", robot.front.sensor.maxValues[3]);
            serial_commands.send_command("irma4", robot.front.sensor.maxValues[4]);

            serial_commands.send_command("irmi0", robot.front.sensor.minValues[0]);
            serial_commands.send_command("irmi1", robot.front.sensor.minValues[1]);
            serial_commands.send_command("irmi2", robot.front.sensor.minValues[2]);
            serial_commands.send_command("irmi3", robot.front.sensor.minValues[3]);
            serial_commands.send_command("irmi4", robot.front.sensor.minValues[4]);
            
            serial_commands.send_command("cnt",robot.front.sensor.intersections);

            
            serial_commands.send_command("st", (float)fsm.state);
            
            // Actuators
            
            serial_commands.send_command("swl", digitalRead(SWITCHL_PIN));
            serial_commands.send_command("swr", digitalRead(SWITCHR_PIN));
            

            
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

/*
*/