#ifndef CONFIG_H              
#define CONFIG_H
#include <vector>
#include <Arduino.h>

//robot1 - IP: 10.227.21.112
//robot2 - IP: 10.227.21.116

#define ROBOT_MASTER
//#define ROBOT_SLAVE
// Proteção de compilação
#if !defined(ROBOT_MASTER) && !defined(ROBOT_SLAVE)
    #error "ERRO: Tens de definir se é o MASTER ou o SLAVE no config.h!"
#endif
#if defined(ROBOT_MASTER) && defined(ROBOT_SLAVE)
    #error "ERRO: Não podes definir MASTER e SLAVE ao mesmo tempo!"
#endif

// ================================================================
//                          PINOS
// ================================================================

//left&right looking to front of the robot
#define SWITCHL_PIN 27 
#define SWITCHR_PIN 2
//left&right looking to the back of the robot
#define BACK_L_SWITCH_PIN 4
#define BACK_R_SWITCH_PIN 5

#define ENC1_PIN_A 8 
#define ENC1_PIN_B 9

#define ENC2_PIN_A 6
#define ENC2_PIN_B 7

//pins do DRV1 referentes na pico(motor)
#define MOTOR1B_PIN 10
#define MOTOR1A_PIN 11

//pins do DRV2 referentes na pico(motor)
#define MOTOR2B_PIN 12
#define MOTOR2A_PIN 13


//DR3 referente na pico(solenoide)
#define FRONT_SOLENOID_PIN_A 15
#define FRONT_SOLENOID_PIN_B 14

#define BACK_SOLENOID_PIN_A 17
#define BACK_SOLENOID_PIN_B 16

// ================================================================
//                          Constantes
// ================================================================
#define MAX_VOLTAGE_USAGE 5.5
#define SENSOR_TARGET_NOR 0.33 //0*500 + 4*1000 / 1500 = 2.666/8 - 0.33
#define SENSOR_TARGET     500.0
#define CALIBRATION_MODE false

//Communications with ComRobot
#define GCHANNELS_BUF_IN_SIZE 500U
#define GCHANNELS_BUF_OUT_SIZE 500U
//#define COMMAND_LIST_SIZE 32

// Communication modes
#define DEBUG_LEVEL 1  // 0=minimal, 1=normal, 2=verbose


// (Paste your 'savedMin' array here)
const uint16_t HARDCODED_FRONT_MIN[] = { 114, 99, 91, 77, 80 }; 
const uint16_t HARDCODED_BACK_MIN[] = { 470, 460, 360, 400, 310 }; 

// (Paste your 'savedMax' array here)
const uint16_t HARDCODED_FRONT_MAX[] = { 962, 959, 951, 954, 850 };
const uint16_t HARDCODED_BACK_MAX[] = { 47, 40, 45, 27, 33 };

// ================================================================
// 1. Structs e tipos personalizados
// ================================================================
typedef struct
{
    float u1;
    float u2;
}MotorVoltages;

struct Node {
    int id;
    float x;
    float y;
    int tipo; 
    // 0 = intersecao, 1 = pick, 2 = drop, 3 = line, 5 = init, 404 = lixo
};

enum class Side {
    FRONT,
    BACK
};
enum class Side2Follow{
    LEFT,
    RIGHT
};
enum class EdgeDetection{
    UP,//when we detect white to back! 
    DOWN//detect black to white! 
};

// ================================================================
// 2. Estados da máquina de estados
// ================================================================
typedef enum {
    // ==========================================
    // --- SYSTEM & STARTUP (0 - 99) ---
    // ==========================================
    SYS_IDLE               = 0, 
    SYS_SET_CALIBRATION    = 1, 
    SYS_CALIBRATION        = 2, 
    SYS_START              = 3, 
    SYS_LEAVE_START        = 4, 
    SYS_APPROACH_WAREHOUSE = 5, 

    // ==========================================
    // --- BOX 1 SEQUENCE (100 - 199) ---
    // ==========================================
    B1_PICK                = 100, 
    B1_PICK_BACKUP         = 101, 
    B1_NAV_TO_DROP         = 120,
    B1_TURN_ON_NAV_TO_DROP = 121, 
    B1_ALIGN_DROP          = 123, 
    B1_APPROACH_DROP       = 104, 
    B1_DROP                = 105, 
    B1_DROP_BACKUP         = 106, 
    B1_LEAVE_DROPZONE      = 107,
    B1_LEAVE_DROPZONE_TURN = 117, 
    B1_NAV_NEXT_LAP        = 108, 

    // ==========================================
    // --- BOX 2 SEQUENCE (200 - 299) ---
    // ==========================================
    B2_NAV_PICK            = 200,
    B2_ALIGN_PICK          = 210,
    B2_PICK                = 211, 
    B2_PICK_BACKUP         = 202, 
    B2_NAV_TO_DROP         = 203, 
    B2_ALIGN_DROP          = 210,
    B2_ALIGN_DROP1         = 211,
    B2_APPROACH_DROP       = 205,//facing the wearhouse - move forward and drop! 
    B2_DROP                = 206, 
    B2_DROP_BACKUP         = 207, 
    B2_LEAVE_DROPZONE      = 208, 
    B2_NAV_NEXT_LAP        = 209, 

    // ==========================================
    // --- BOX 3 SEQUENCE (300 - 399) ---
    // ==========================================
    B3_NAV_PICK            = 300,
    B3_ALIGN_PICK          = 310,
    B3_PICK                = 311,
    B3_PICK_BACKUP         = 302,
    B3_NAV_TO_DROP         = 303,
    B3_ALIGN_DROP          = 304,
    B3_APPROACH_DROP       = 305,
    B3_DROP                = 306,
    B3_DROP_BACKUP         = 307,
    B3_LEAVE_DROPZONE      = 308,
    B3_NAV_NEXT_LAP        = 309,

    // ==========================================
    // --- BOX 4 SEQUENCE (400 - 499) ---
    // ==========================================
    B4_NAV_PICK            = 400,
    B4_ALIGN_PICK          = 410,
    B4_PICK                = 411,
    B4_PICK_BACKUP         = 402,
    B4_NAV_TO_DROP         = 403,
    B4_ALIGN_DROP          = 404,
    B4_APPROACH_DROP       = 405,
    B4_DROP                = 406,
    B4_DROP_BACKUP         = 407,
    B4_LEAVE_DROPZONE      = 408,
    B4_NAV_NEXT_LAP        = 409,

    NAV_TO_WEARHOUSE       = 799,
    NAV_LEAVING_WEARHOUSE  = 798,
    NAV_TO_WEARHOUSE_PICK  = 700,
    NAV_TO_WEARHOUSE_DROP  = 701,
    NAV_LEAVING_WEARHOUSE_P= 710,
    NAV_LEAVING_WEARHOUSE_D= 711,

    // ==========================================
    // --- GENERIC MANEUVERS
    // ==========================================
    GEN_TURN_90            = 900,
    GEN_MOVE_X             = 800,
    
    GEN_PICK_ZONE          = 500,
    GEN_PICK_COUNT         = 510,
    GEN_PICK_ALIGN         = 520,
    GEN_PICK_BOX           = 530,
    GEN_PICK_TURN_OUT      = 540,
    EXITING_PICK_ZONE      = 550,

    GEN_DROP_BOX           = 600,
    GEN_DROP_COUNT         = 610,
    GEN_DROP_ALIGN         = 620,
    GEN_DROP_TURN_OUT      = 630,
    EXITING_DROP_ZONE      = 640,
    

} fsm_state;

//this indexes should match in p4d.cpp aray declaration!
typedef enum { 
  p4d_drv1 = 0,
  p4d_drv2 = 1,
  p4d_drvSolenoid_front = 2,
  p4d_drvSolenoid_back =3,
} driver_num_t;


// ================================================================
// 3. Nós do plano 
// ================================================================
const std::vector<Node> mappingNodes1 = {
    // ID |   X    |   Y    | Type, 0-intersection / 1-pick / 2- drop
    {  0, -0.705,  0.480,  1 },
    {  1, -0.555,  0.480,  1 },
    {  2, -0.405,  0.480,  1 },
    {  3, -0.255,  0.480,  1 },
    {  4, -0.705,  0.360,  0 },
    {  5, -0.555,  0.360,  0 },
    {  6, -0.405,  0.360,  0 },
    {  7, -0.255,  0.360,  0 },
    {  8,  0.000,  0.360,  0 },
    {  9,  0.700,  0.360,  5 },
    { 10,  0.000,  0.150,  0 },
    { 11,  0.226,  0.150,  2 },
    { 12,  0.470,  0.150,  1 },
    { 13,  0.700,  0.150,  0 },
    { 14, -0.705,  0.000,  0 },
    { 15, -0.475,  0.000,  2 },
    { 16, -0.232,  0.000,  1 },
    { 17,  0.000,  0.000,  0 },
    { 18,  0.226,  0.000,  2 },
    { 19,  0.470,  0.000,  1 },
    { 20,  0.700,  0.000,  0 },
    { 21, -0.705, -0.150,  0 },
    { 22, -0.475, -0.150,  2 },
    { 23, -0.232, -0.150,  1 },
    { 24,  0.000, -0.150,  0 },
    { 25, -0.705, -0.360,  5 },
    { 26,  0.000, -0.360,  0 },
    { 27,  0.245, -0.360,  0 },
    { 28,  0.395, -0.360,  0 },
    { 29,  0.000,  0.000,  0 },
    { 30,  0.700, -0.360,  0 },
    { 31,  0.245, -0.480,  3 },
    { 32,  0.395, -0.480,  2 },
    { 33,  0.550, -0.480,  2 },
    { 34,  0.700, -0.480,  2 }
};

#endif