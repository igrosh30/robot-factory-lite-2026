#ifndef CONFIG_H              
#define CONFIG_H
// ================================================================
//                          PINOS
// ================================================================
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

//#define SOLENOID_PIN_A 12
//#define SOLENOID_PIN_B 13


// ================================================================
// 1. Structs e tipos personalizados
// ================================================================
typedef struct
{
    float u1;
    float u2;
}MotorVoltages;

// ================================================================
// 2. Estados da máquina de estados
// ================================================================
typedef enum {
    STATE_STOP          = 0,
    STATE_FORWARD       = 101,
    STATE_STARTING      = 100,
    STATE_FOLLOW_LINE   = 201,
    STATE_TURN_90_LEFT  = 202,
    STATE_TURN_90_RIGHT = 203,
    STATE_PICKUP_BOX    = 301,
    STATE_DROP_BOX      = 302,
    STATE_FINISHED      = 999,
    STATE_OUTPUT        = 1
} RobotState;

#endif