#ifndef ROBOT_H
#define ROBOT_H
#endif

#include <Arduino.h>
#include <math.h>
#include "MotorController.h"
#include "Sensors.h"
#include "Actuators.h"
#include "config.h"
#include "gchannels.h"
#include "AppLayer.h"

#ifndef NUM_WHEELS
#define NUM_WHEELS 2

typedef enum
{
  cm_voltage,
  cm_pid,
  cm_kinematics,
  cm_pos
} control_mode_t;

enum ComState : uint8_t
{
    COM_IDLE,
    COM_START,
    COM_WAIT_PONG,
    COM_WAIT_SEND,
    COM_WAIT_ACK,
    COM_ERROR,
    COM_LISTEN     
};



class robot_t
{
public:
  MotorController motors;
  AppLayer appLayer;
  //Sensor frontSensor;
  //Sensor backSensor;
  RobotSide front;
  RobotSide back;

  //COM state:
  ComState currentComState;
  uint8_t sendTries;
  uint32_t comTimer;
  int cmdID_send_debug = 0;
  int send_debug = 0; // 0 not send , 1 send

  Node targetNode;
  Node currentNode;
  //MotorController motor1;
  //MotorController motor2;
  
  NodeId robot_id;
  int enc1, enc2;
  int Senc1, Senc2;
  float w1e, w2e;
  float v1e, v2e;
  float p1e, p2e;
  float ve, we;
  float ds, dtheta;
  float rel_s, rel_theta;
  float xe, ye, thetae;

  float dt;
  float v, w;
  float v_req, w_req;
  float dv_max, dw_max;

  float wheel_radius, wheel_dist;

  float v1ref, v2ref;
  float w1ref, w2ref;
  float p1ref;
  float u1, u2;
  float u1_req, u2_req;
  float i_sense, u_sense;
  float i_lambda;
  int PWM_1, PWM_2;
  bool stoped;
  // int PWM_1_req, PWM_2_req;
  float w1_req, w2_req;
  float p1_req;
  control_mode_t control_mode;
  float follow_v, follow_k;

  int solenoid_PWM;
  int led;

  
  float tof_dist, prev_tof_dist;

  int LastTouchSwitch, TouchSwitch;

  gchannels_t* pchannels;
  //state_machine_t* fsm;
  
  //COM methods:
  bool init_COM(Stream* comPort);
  void updateComState();
  void send_command(uint8_t cmdId);

  bool hasPendingCommand = false;
  uint8_t pendingCommandId = 0;


  robot_t(pico4drive_t& driver);

  void odometry(void);
  void setRobotVW(float Vnom, float Wnom);
  float getAngleDiff(float target_angle, float current_angle);//map to π and -π

  void accelerationLimit(void);
  void calcMotorsVoltage(void);

  void followLineRight(float Vnom,Side side);
  void followLineLeft(float Vnom,Side side);

  void followLine(float Vnom, RobotSide& sideRef, Side2Follow direction, EdgeDetection edge);

  
  void grabBox(RobotSide& sideRef);
  void dropBox(RobotSide& sideREf);
  bool has2Boxes();
  bool hasDroped2Boxes();

  //LOCALISATION METHODS:
  bool atPikZone();
  bool atDropZone();
};

extern robot_t robot;

#endif // ROBOT_H
