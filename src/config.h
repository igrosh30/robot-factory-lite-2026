#ifndef CONFIG_H              
#define CONFIG_H
#include <vector>
#include <Arduino.h>


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
#define CALIBRATION_MODE true

//Communications with ComRobot
#define GCHANNELS_BUF_IN_SIZE 500U
#define GCHANNELS_BUF_OUT_SIZE 500U
//#define COMMAND_LIST_SIZE 32

// Communication modes
#define DEBUG_LEVEL 1  // 0=minimal, 1=normal, 2=verbose


// (Paste your 'savedMin' array here)
const uint16_t HARDCODED_FRONT_MIN[] = { 470, 460, 360, 400, 310 }; 
const uint16_t HARDCODED_BACK_MIN[] = { 470, 460, 360, 400, 310 }; 

// (Paste your 'savedMax' array here)
const uint16_t HARDCODED_FRONT_MAX[] = { 47, 40, 45, 27, 33 };
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

// ================================================================
// 2. Estados da máquina de estados
// ================================================================

typedef enum {
    Idle                    ,   // 0
    Start                   ,   // 1
    Set_Calibration         ,   // 2
    Calibration             ,   // 3
    Move_F                  ,   // 4
    PickBox                 ,   // 5
    PickBox_Back            ,   // 6   
    
    Box1GO2DropZone             ,   // 7
    Box1GO2DropZone1            ,   // 8
    Box1GO2DropZone2            ,   // 9

    B1_LDZ                      , // 10
    LAP_2                       , //11
    DropBox_Back                ,   // 18 

    PickBox2                    ,   // 19
    PickBox_Back1               ,   // 20
    B2_DZ2                      ,   // 21
    B2_DZ2_1                    ,   // 22
    B3_PZ2                      ,   // 23 

    FL_frontL               

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