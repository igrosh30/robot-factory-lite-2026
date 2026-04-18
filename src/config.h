#ifndef CONFIG_H              
#define CONFIG_H
#include <vector>
#include <Arduino.h>

//Master - IP: 10.227.21.112
//Slave00- IP: 10.227.21.115 com7 
//SLAVE01- IP: 10.227.21.110 com8

//#define ROUND_1
#define ROUND_2


//#define ROBOT_MASTER
//#define ROBOT_SLAVE_00
#define ROBOT_SLAVE_01

// Proteção de compilação
#if !defined(ROBOT_MASTER) && !defined(ROBOT_SLAVE_00) && !defined(ROBOT_SLAVE_01)
    #error "ERRO: Tens de definir se é o MASTER ou o SLAVE no config.h!"
#endif

#if (defined(ROBOT_MASTER) && defined(ROBOT_SLAVE_00)) || \
    (defined(ROBOT_MASTER) && defined(ROBOT_SLAVE_01)) || \
    (defined(ROBOT_SLAVE_00) && defined(ROBOT_SLAVE_01))
    #error "ERRO: Múltiplos robots definidos ao mesmo tempo! Define apenas UM."
#endif

// ================================================================
//                          PINOS
// ================================================================

//left&right looking to front of the robot
#define SWITCHL_PIN 27 
#define SWITCHR_PIN 26
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
const uint8_t INVALID_SLOT = 255;

//Communications with ComRobot
#define GCHANNELS_BUF_IN_SIZE 500U
#define GCHANNELS_BUF_OUT_SIZE 500U
//#define COMMAND_LIST_SIZE 32

// Communication modes
#define DEBUG_LEVEL 0 // 0=minimal, 1=normal, 2=verbose, 3= COM
//#define CONFIG_H12_CANNELS

// (Paste your 'savedMin' array here)
#ifdef ROBOT_MASTER
const uint16_t HARDCODED_FRONT_MIN[] = { 114, 99, 91, 77, 80 }; 
const uint16_t HARDCODED_FRONT_MAX[] = { 962, 959, 951, 954, 850 };
#endif

#ifdef ROBOT_SLAVE_00
const uint16_t HARDCODED_FRONT_MIN[] = { 181, 186, 186, 112, 126 }; 
const uint16_t HARDCODED_FRONT_MAX[] = { 981, 980, 981, 975, 975 };
#endif

#ifdef ROBOT_SLAVE_01
const uint16_t HARDCODED_FRONT_MIN[] = { 136, 125, 132, 139, 127 }; 
const uint16_t HARDCODED_FRONT_MAX[] = { 972, 967, 972, 974, 972 };
#endif

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
    SYS_CALIBRATION        = 1, 
    SYS_WAIT_IR            = 2,
    SYS_LEAVE_START        = 4, 
    SYS_APPROACH_WAREHOUSE = 5, 

    NAV_TO_WEARHOUSE       = 799,
    NAV_LEAVING_WEARHOUSE  = 798,
    M_NAV_FROM_MACHINE     = 797,

    // ==========================================
    // --- COMMUNICATION STATES:
    // ==========================================
    COM_INIT               = 40, 
    COM_BOXES_SLAVE_00     = 41,
    COM_BOXES_SLAVE_01     = 42,

    // ==========================================
    // --- GENERIC MANEUVERS- from Start Point
    // ==========================================
    GEN_TURN_90            = 900,
    GEN_MOVE_X             = 800,
    
    GEN_PICK_ZONE          = 500,
    GEN_PICK_COUNT_NAV_FROM_START   = 510,
    GEN_PICK_COUNT_FROM_MACHINE = 511,
    GEN_PICK_ALIGN         = 520,
    GEN_PICK_BOX           = 530,
    GEN_PICK_TURN_OUT      = 540,
    EXITING_PICK_ZONE      = 550,
    EXITING_PICK_ZONE_RED  = 555,

    GEN_DROP_BOX           = 600,
    GEN_DROP_COUNT         = 610,
    GEN_DROP_ALIGN         = 620,
    GEN_DROP_TURN_OUT      = 630,
    EXITING_DROP_ZONE      = 640,
    

    // ==========================================
    // ---          MASTER STATES:          ////201 -- 202 -- 204
    // ==========================================
    
    // --- MASTER init states
    M_SYS_START             = 100,
    M_SYS_LEAVE_START       = 101, // SEND SLAVE- pickSlots blue box!  

    // --- Green Box
    M_NAV_PROCESS_GREEN_BOX         = 200,
    M_GEN_DROP_ALIGN                = 201,
    M_GEN_DROP_TURN_OUT             = 202,
    M_EXT_PROC_MACH_GREEN           = 203,
    M_EXT_PROC_MACH_BLUE            = 204,      
    
    //-----Red Box
    M_NAV_DROP_RED                  = 210,
    M_DROP_ALIGN_RED                = 211,
    M_DROP_TURN_OUT_RED             = 212,
    M_NAV_PICK_FROM_RED             = 213,



    // ==========================================
    // ---          SLAVE STATES:
    // ==========================================
    S_WAIT_CMD_START                = 300, 
    S_WAIT_PERMISSION               = 301,
    S_WAIT_BOX_INFO                 = 302,   
    S_NAV_MACHINE_OUT               = 310,
    S_NAV_EXIT_DROP_ZONE_2_MACHINE  = 305,
    S_NAV_TO_MACHINE_FROM_WHOUSE    = 320,
    S_WAIT_PICK_CMD                 = 399,
    S_MACHINE_ALIGN_PICK            = 331,
    S_MACHINE_PICK_BOX              = 330,
    S_MACHINE_TURN_OUT              = 340,
    S_NAV_MACHINE_TO_DROP           = 350,
    
    //SLAVE 01 STATES:
    S1_NAV_MACHINE_A                 = 400,
    S1_NAV_MACHINE_A_ALIGN           = 401,
    S1_ALIGN_PICK_A                  = 410,
    S1_PICK_BOX_A                    = 411,
    S1_NAV_DROP_B                    = 420,
    S1_ALIGN_DROP_B                  = 430,
    S1_NAV_MACHINE_A_FROM_B          = 440,
    



    NAV_DOCKING_STATION           = 9999,
    

} fsm_state;



//this indexes should match in p4d.cpp aray declaration!
typedef enum { 
  p4d_drv1 = 0,
  p4d_drv2 = 1,
  p4d_drvSolenoid_front = 2,
  p4d_drvSolenoid_back =3,
} driver_num_t;



#endif