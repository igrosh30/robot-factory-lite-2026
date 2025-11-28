#include "communication.h"
#include "gchannels.h"
#include "robot.h"
#include <Arduino.h>
#include <LittleFS.h>

#define LOOPMICROS 40000UL

// External dependencies
#include "pico4drive.h"
extern pico4drive_t pico4drive;

// WiFi Configuration
#define MAX_WIFI_STR 32
char wifi_ssid[MAX_WIFI_STR];
char wifi_password[MAX_WIFI_STR];
const char *WIFI_CONFIG_FILE = "/wifi.txt";

// UDP Communication
WiFiUDP udp_socket;
unsigned int udp_port = 4224;
static bool udp_active = false;
static bool udp_connected = false;

#define UDP_MAX_PACKET_SIZE 512
uint8_t udp_in_buffer[UDP_MAX_PACKET_SIZE];
uint8_t udp_out_buffer[UDP_MAX_PACKET_SIZE];

// External robot and communication instances
extern robot_t robot;
extern gchannels_t udp_commands;
extern gchannels_t serial_commands;
extern commands_list_t pars_list;
//extern PID_pars_t wheel_PID_pars;
extern const char *pars_fname;
extern bool load_pars_requested;

// Pi4 Communication Protocol (Text-based for ROS)
#define PI4_COMM_TIMEOUT_MS 1000
#define PI4_COMM_BUFFER_SIZE 256

// Pi4 Communication state
static bool pi4_connected = false;
static uint32_t last_pi4_comm_time = 0;

// Function declarations
void initializeWiFi();
void initializeUDP();
void initializeComRobot();
void processPi4Communication();
void processPi4Command(const String& command);
void sendPi4Response();

/**
 * Initialize communication systems
 * Sets up WiFi, UDP, and ComRobot debug communication
 */
void communication_init() {
    Serial.println("Initializing communication systems...");
    
#ifdef COMROBOT_DEBUG
    initializeWiFi();
    initializeUDP();
    initializeComRobot();
#endif
    
    Serial.println("Communication systems initialized.");
}

/**
 * Initialize WiFi connection
 */
void initializeWiFi() {
    Serial.println("Connecting to WiFi...");
    
    // Configure WiFi credentials
    //strcpy(wifi_ssid, "raspberrypi");
    //strcpy(wifi_password, "UJr2016#");
    strcpy(wifi_ssid, "5DPO-NETWORK");
    strcpy(wifi_password, "5dpo5dpo");
    
    // Reset WiFi state
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
    
    // Attempt connection
    WiFi.begin(wifi_ssid, wifi_password);
    
    unsigned long start_time = millis();
    const unsigned long timeout = 25000;
    
    while (WiFi.status() != WL_CONNECTED && millis() - start_time < timeout) {
        delay(500);
        Serial.print(".");
        yield();
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connection failed. Running offline.");
    }
}

/**
 * Initialize UDP communication
 */
void initializeUDP() {
    if (WiFi.status() == WL_CONNECTED) {
        delay(500);
        udp_connected = (udp_socket.begin(udp_port) == 1);
        if (udp_connected) {
            Serial.printf("UDP listening at IP %s, port %d\n", 
                         WiFi.localIP().toString().c_str(), udp_port);
        }
    }
}


#ifdef COMROBOT_DEBUG
/**
 * Initialize ComRobot debug communication
 */
void initializeComRobot() {
    analogReadResolution(10);
    
    // Register PID parameters
    pars_list.register_command("kf", &(wheel_PID_pars.Kf));
    pars_list.register_command("kc", &(wheel_PID_pars.Kc));
    pars_list.register_command("ki", &(wheel_PID_pars.Ki));
    pars_list.register_command("kd", &(wheel_PID_pars.Kd));
    pars_list.register_command("kfd", &(wheel_PID_pars.Kfd));
    pars_list.register_command("dz", &(wheel_PID_pars.dead_zone));
    pars_list.register_command("kfp", &(wheel_PID_pars.Kf_p));
    pars_list.register_command("kcp", &(wheel_PID_pars.Kc_p));
    pars_list.register_command("kip", &(wheel_PID_pars.Ki_p));
    pars_list.register_command("kdp", &(wheel_PID_pars.Kd_p));
    pars_list.register_command("kfdp", &(wheel_PID_pars.Kfd_p));
    
    // Initialize communication channels
    udp_commands.init(process_command, serial_write);
    serial_commands.init(process_command, serial_write);
    robot.pchannels = &serial_commands;
    
    // Initialize file system
    LittleFS.begin();

    
    load_commands(pars_fname, serial_commands);
}
#endif

/**
 * Check if WiFi is connected
 */
bool communication_is_connected() {
    return WiFi.status() == WL_CONNECTED;
}

/**
 * Get current IP address
 */
String communication_get_ip() {
    return WiFi.localIP().toString();
}

#ifdef COMROBOT_DEBUG
/**
 * Update communication systems
 * Handles UDP packet processing and maintains connections
 */
void communication_update() {
    // Initialize UDP if connected but not active
    if (communication_is_connected() && !udp_connected) {
        initializeUDP();
    }
    
    // Process UDP packets
    if (udp_connected) {
        int packet_size = udp_socket.parsePacket();
        if (packet_size > 0) {
            int bytes_read = udp_socket.read(udp_in_buffer, 
                                           min(packet_size, UDP_MAX_PACKET_SIZE - 1));
            if (bytes_read > 0) {
                udp_in_buffer[bytes_read] = 0;
                for (int i = 0; i < bytes_read; i++) {
                    udp_commands.process_char(udp_in_buffer[i]);
                }
                udp_traffic_active = 1;
            }
        }
    }
}
#endif

/**
 * Write data to serial and UDP
 */
void serial_write(const char* buffer, size_t size) {
    Serial.write(buffer, size);
    
    if (!udp_connected) return;
    
    uint16_t remote_port = udp_socket.remotePort();
    IPAddress remote_ip = udp_socket.remoteIP();
    
    if (remote_port == 0 || remote_ip == IPAddress(0,0,0,0)) {
        return; // No valid remote address
    }
    
    udp_socket.beginPacket(remote_ip, remote_port);
    udp_socket.write(buffer, size);
    udp_socket.endPacket();
}

/**
 * Process Pi4 communication on Core 1
 * Text-based protocol: Pi4 sends "CMD:v,w,pick" and Pico responds with "POS: x,y,theta, TOF: obstacle"
 */
void processPi4Communication() {
    static String input_buffer = "";
    
    // Read available serial data
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            // Process complete line
            if (input_buffer.length() > 0) {
                processPi4Command(input_buffer);
                input_buffer = "";
            }
        } else {
            input_buffer += c;
            
            // Prevent buffer overflow
            if (input_buffer.length() > PI4_COMM_BUFFER_SIZE) {
                input_buffer = "";
            }
        }
    }
    
    // Check for communication timeout
    if (pi4_connected && (millis() - last_pi4_comm_time > PI4_COMM_TIMEOUT_MS)) {
        pi4_connected = false;
        // Stop robot if communication is lost (thread-safe)
        robot.motors.driveMotor(0, 0);
        //setRobotIman(0);
    }
}

/**
 * Process Pi4 command in text format
 * Expected format: "CMD:v_d,w_d,pick_box"
 */
void processPi4Command(const String& command) {
    if (command.startsWith("CMD:")) {
        String data = command.substring(4); // Remove "CMD:" prefix
        
        // Parse comma-separated values
        int first_comma = data.indexOf(',');
        int second_comma = data.indexOf(',', first_comma + 1);
        
        if (first_comma > 0 && second_comma > 0) {
            // Extract values
            float v_d = data.substring(0, first_comma).toFloat();
            float w_d = data.substring(first_comma + 1, second_comma).toFloat();
            bool pick_box = (data.substring(second_comma + 1).toInt() != 0);
            
            // Update robot state (thread-safe)
            robot.setRobotVW(v_d,w_d);
            //setRobotVelocities(v_d, w_d);
            //setRobotIman(pick_box ? 1 : 0);
            
            // Update communication state
            last_pi4_comm_time = millis();
            pi4_connected = true;
            
            // Send response immediately
            sendPi4Response();
        }
    }
}

/**
 * Send response to Pi4 in text format
 * Format: "POS: x, y, theta, V: v, W: w, TOF: obstacle_detected"
 */
void sendPi4Response() {
    // Read robot state (thread-safe)
    float x, y, theta, v, w, tof_dist;
    //getRobotOdometry(x, y, theta, v, w);
    //getRobotTOF(tof_dist);
    
    // Check for obstacle detection (TOF distance < threshold)
    const float OBSTACLE_THRESHOLD = 0.05f; // 5cm threshold
    bool obstacle_detected = (tof_dist < OBSTACLE_THRESHOLD);
    
    // Format response message
    String response = "POS: ";
    response += String(x, 3);
    response += ", ";
    response += String(y, 3);
    response += ", ";
    response += String(theta, 3);
    response += ", V: ";
    response += String(v, 3);
    response += ", W: ";
    response += String(w, 3);
    response += ", TOF: ";
    response += obstacle_detected ? "1" : "0";
    response += "\n";
    
    // Send response
    Serial.print(response);
}

/**
 * Send debug data to ComRobot simulator (if enabled)
 */
void sendComRobotDebug(uint32_t loop_micros) {
#ifdef COMROBOT_DEBUG
    // Control mode and voltages
    serial_commands.send_command("mo", robot.control_mode);
    serial_commands.send_command("u1", robot.u1);
    serial_commands.send_command("u2", robot.u2);
    
    // Encoder data
    serial_commands.send_command("e1", robot.enc1);
    serial_commands.send_command("e2", robot.enc2);
    serial_commands.send_command("s1", robot.Senc1);
    serial_commands.send_command("s2", robot.Senc2);

    serial_commands.send_command("aux", robot.aux);
    serial_commands.send_command("b",robot.wheel_dist);
    
    // Battery voltage
    serial_commands.send_command("Vbat", pico4drive.battery_voltage);
    
    // Robot velocities
    serial_commands.send_command("vm", robot.v_m);
    serial_commands.send_command("wm", robot.w_m);
    serial_commands.send_command("vr", robot.v_ref);
    serial_commands.send_command("wr", robot.w_ref);
    
    // Wheel velocities
    serial_commands.send_command("wme", robot.w_m_esq);
    serial_commands.send_command("wmd", robot.w_m_dir);
    serial_commands.send_command("w1r", robot.w1ref);
    serial_commands.send_command("w2r", robot.w2ref);
    serial_commands.send_command("vme", robot.v_m_esq);
    serial_commands.send_command("vmd", robot.v_m_dir);
    
    // Motor PWM values
    serial_commands.send_command("m1", robot.pwm_esq);
    serial_commands.send_command("m2", robot.pwm_dir);
    
    // Electromagnet state
    serial_commands.send_command("im", robot.iman);

    // Robot position
    serial_commands.send_command("x", robot.x_sti);
    serial_commands.send_command("y", robot.y_sti);
    serial_commands.send_command("th", robot.theta_sti);
    
    // PID parameters
    serial_commands.send_command("kc", wheel_PID_pars.Kc);
    serial_commands.send_command("ki", wheel_PID_pars.Ki);
    serial_commands.send_command("kd", wheel_PID_pars.Kd);
    serial_commands.send_command("kf", wheel_PID_pars.Kf);
    
    // Network info
    serial_commands.send_command("IP", WiFi.localIP().toString().c_str());
    
    // TOF sensor
    serial_commands.send_command("tof", robot.tof_dist);
    
    // Send parameter updates
    pars_list.send_sparse_commands(serial_commands);
    
    // Debug info
    Serial.print(" cmd: ");
    Serial.print(serial_commands.frame.command);
    Serial.print("; ");
    
    debug = serial_commands.out_count;
    serial_commands.send_command("dbg", 5);
    serial_commands.send_command("loop", micros() - loop_micros);
    
    serial_commands.flush();
    Serial.println();
    
    // Handle OTA updates
    http_ota.handle();
#endif
}
/**
 * Process incoming commands
 */
void process_command(command_frame_t frame) {
    pars_list.process_read_command(frame);
    
    if (frame.command_is("mo")) {
        robot.control_mode = (control_mode_t)frame.value;
    }
    else if (frame.command_is("u1")) {
        robot.u1_req = frame.value;
    }
    else if (frame.command_is("u2")) {
        robot.u2_req = frame.value;
    }
    else if (frame.command_is("w1r")) {
        robot.w1_req = frame.value;
    }
    else if (frame.command_is("w2r")) {
        robot.w2_req = frame.value;
    }
    else if (frame.command_is("vr")) {
        robot.v_req = frame.value;
    }
    else if (frame.command_is("wr")) {
        robot.w_req = frame.value;
    }
    else if (frame.command_is("m1")) {
        //robot.pwm_esq = frame.value;
    }
    else if (frame.command_is("m2")) {
        //robot.pwm_dir = frame.value;
    }
    else if (frame.command_is("im")) {
        //robot.iman = (frame.value == 1) ? 1 : 0;
    }
    else if (frame.command_is("s1")) {
        robot.Senc1 = frame.value;
    }
    else if (frame.command_is("s2")) {
        robot.Senc2 = frame.value;
    }
    else if (frame.command_is("x")) {
        robot.xe = frame.value;
    }
    else if (frame.command_is("y")) {
        robot.ye = frame.value;
    }
    else if (frame.command_is("th")) {
        robot.thetae = frame.value;
    }else if (frame.command_is("aux")) {
        //robot.aux = frame.value;
    }else if (frame.command_is("b")){
        robot.wheel_dist = frame.value;
    }
}

/**
 * Send file over communication channel
 */
void send_file(const char *filename, int log_high) {
    File file = LittleFS.open(filename, "r");
    if (!file) {
        serial_commands.send_command("err", filename);
        return;
    }
    
    serial_commands.flush();
    Serial.flush();
    
    uint8_t byte_data;
    uint8_t mask = log_high ? 0x80 : 0;
    
    while (file.read(&byte_data, 1) == 1) {
        serial_commands.send_char(byte_data | mask);
    }
    
    file.close();
    serial_commands.flush();
    Serial.flush();
}


