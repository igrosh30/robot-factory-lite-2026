/* 
 * Communication module for Robot Factory Lite 2026
 * 
 * Handles:
 * - WiFi connection and UDP communication for ComRobot
 * - Serial communication with Raspberry Pi 4 (ROS)
 * - Debug data streaming to ComRobot simulator
 * - Command processing and parameter management
 * 
 * Author: DPO
 */
// communication.h
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <LittleFS.h>

// Communication configuration
#define LOOPMICROS 40000UL
#define UDP_PORT 4224
#define UDP_MAX_PACKET_SIZE 512
#define PI4_COMM_TIMEOUT_MS 1000
#define PI4_COMM_BUFFER_SIZE 256

// External dependencies
#include "gchannels.h"
#include "robot.h"
#include "pico4drive.h"

// Communication state
extern bool udp_connected;
extern bool pi4_connected;
extern int debug;

// Communication instances
extern gchannels_t udp_commands;
extern gchannels_t serial_commands;
extern commands_list_t pars_list;

// Initialization
void communication_init();
void initializeWiFi();
void initializeUDP();
void initializeComRobot();

// Main update functions
void communication_update();
void processPi4Communication();

// ComRobot debug communication
void sendComRobotDebug(uint32_t loop_micros);
void process_command(command_frame_t frame);
void serial_write(const char* buffer, size_t size);

// Pi4 communication
void processPi4Command(const String& command);
void sendPi4Response();

// Utility functions
bool communication_is_connected();
String communication_get_ip();
void send_file(const char *filename, int log_high);

#endif