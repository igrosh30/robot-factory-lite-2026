#include <Arduino.h>
#include "robot.h"
#include <math.h>
#include "pico4drive.h"
#include "MotorController.h"

//robot_t robot;

template <typename T>
int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

robot_t::robot_t(pico4drive_t& driver)
  : front(driver,Side::FRONT),
    back (driver,Side::BACK)
{
  #ifdef ROBOT_MASTER
  robot_id = MASTER;
  #endif
  #ifdef ROBOT_SLAVE_00
  robot_id= SLAVE_00;
  #endif
  #ifdef ROBOT_SLAVE_01
  robot_id = SLAVE_01;
  #endif
  

  stoped = false;
  wheel_dist = 0.075;
  wheel_radius = 0.0689 / 2;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;
  //ini
  
  v_req = 0.0f;
  w_req = 0.0f;
  v = 0.0f;
  w = 0.0f;
  p1e = 0;
  p2e = 0;

  follow_k = -0.15;
  follow_v = 0.20;

  i_lambda = 0;
  led = 0;

  pchannels = NULL;
  
}

//I will do Serial2.setTx and setRx, then I will pass the Serial object in comPort!
bool robot_t::init_COM(Stream* comPort)
{
  if(comPort == nullptr) return false;
  currentComState = ComState::COM_IDLE; //This will be changed inside the StateMachine! 
  COM_SLAVE00_ok = true;// true if PING PONG WORKS! 
  COM_SLAVE01_ok = true;
  sendTries = 0;
  comTimer = 0;

  this->appLayer.init(comPort,robot_id);
  return true;

}
void robot_t::updateComState()
{
    #ifdef ROBOT_MASTER
    // ====================== MASTER ======================
    switch(currentComState)
    {
        case ComState::COM_START:
          //---------SERIAL PRINTS---------  
          Serial.println("[MASTER] sending PING to Slave 00...");
          Serial.print("Tries:"); Serial.print(sendTries);
          Serial.println("\n============================================");
          
          appLayer.sendPing(SLAVE_00); 
          comTimer = millis();
          currentComState = ComState::COM_WAIT_SLAVE00_PONG;
          break;

        case ComState::COM_WAIT_SLAVE00_PONG:
          appLayer.update();
          if(sendTries > 20) 
          {
              Serial.printf("exceded tries... Ping to SLAVE_00 failded");
              COM_SLAVE00_ok = false;
              //Send Ping to SLAVE_01...
              appLayer.sendPing(SLAVE_01); 
              currentComState = ComState::COM_WAIT_SLAVE01_PONG;
              comTimer = millis();
              sendTries = 0;
              //send a ping to SLAVE01:
              //currentComState = ComState::COM_ERROR;

              //break;
          }
          if(appLayer.hasReceivedPong())
          {
              COM_SLAVE00_ok = true;
              //send now to SLAVE_01
              Serial.println("[MASTER] sending PING to Slave 01...");
              appLayer.sendPing(SLAVE_01); 
              currentComState = ComState::COM_WAIT_SLAVE01_PONG;
              comTimer = millis();
              sendTries = 0;
          }
          else if(millis() - comTimer > 2000)
          {
              sendTries++;
              comTimer = millis();
              currentComState = ComState::COM_START;
          }
          break;

        case ComState::COM_WAIT_SLAVE01_PONG:
          appLayer.update();
          if(sendTries > 20)//ERROR in PING 
          {
              COM_SLAVE01_ok = false;
              sendTries = 0;
              currentComState = ComState::COM_WAIT_SEND; 
          }
          if(appLayer.hasReceivedPong())
          {
            Serial.println("[MASTER]: Received Pong from SLAVE_01...");
              COM_SLAVE01_ok = true;
              currentComState = ComState::COM_WAIT_SEND;
              sendTries = 0;
          }
          else if(millis() - comTimer > 3500)
          {
              sendTries++;
              comTimer = millis();
              //just need to resend if failed!
              Serial.println("[MASTER] TIMEOUT, recending PING frame to SlAVE_01");
              appLayer.sendPing(SLAVE_01);
          }
          break;        

        case ComState::COM_WAIT_SEND:
          if(pending_id_dest == SLAVE_00)
          {
            if(hasPendingCommandSend)
            {
                if(pendingParamLen > 0)
                {
                  appLayer.sendCommandWithData(NodeId::SLAVE_00, pendingCommandId, pendingParams, pendingParamLen);
                }else 
                {
                  appLayer.sendCommand(NodeId::SLAVE_00, pendingCommandId);    
                }
                comTimer = millis();
                currentComState = ComState::COM_WAIT_ACK;
            }
          }
          else if(pending_id_dest == SLAVE_01)
          {
            if(hasPendingCommandSend)
            {
              if(pendingParamLen > 0)
              {
                appLayer.sendCommandWithData(NodeId::SLAVE_01, pendingCommandId, pendingParams, pendingParamLen);
              }else 
              {
                appLayer.sendCommand(NodeId::SLAVE_01, pendingCommandId);    
              }
              comTimer = millis();
              currentComState = ComState::COM_WAIT_ACK;
            }
          }
          break;
        
        case ComState::COM_WAIT_ACK:
          appLayer.update();
          if(sendTries > 20 ){ //CHECK HOW MANY TRIES!
            hasPendingCommandSend = false; // Give up and clear flags
            pendingParamLen = 0;
            currentComState = ComState::COM_ERROR;
          } 
          else if(appLayer.hasReceivedAck(pendingCommandId))
          {
            Serial.println("[MASTER] ACK Received!");
            sendTries = 0;
            hasPendingCommandSend = false;
            pendingParamLen = 0;
            currentComState = ComState::COM_WAIT_SEND;
          }
          else if(millis() - comTimer > 1000) // This is a timeout! I should resend the frame if we enter here! 
          {
            //resend the frame!!!!!!!
            comTimer = millis();
            sendTries++;
            Serial.printf("[MASTER] Missed ACK. Resending CMD: %d (Try: %d)\n", pendingCommandId, sendTries);
            if(pendingParamLen > 0) appLayer.sendCommandWithData(pending_id_dest, pendingCommandId, pendingParams, pendingParamLen);
            else  appLayer.sendCommand(pending_id_dest, pendingCommandId);
          }
          break;
      
        case ComState::COM_ERROR:
          COM_SLAVE00_ok = false;
          break;
    }
    #endif

    // ====================== SLAVE ======================
    
    #ifdef ROBOT_SLAVE_00 
    switch(currentComState)
    {
        case ComState::COM_START:
            currentComState = ComState::COM_LISTEN_PING;
            break;

        case ComState::COM_LISTEN_PING:
          appLayer.update();
          if(appLayer.hasReceivedPing()){
            appLayer.clearPingFlag();
            currentComState = ComState::COM_LISTEN;
            Serial.println("Received PING...");
          } 
          break;

          case ComState::COM_LISTEN:
            //Serial.println("Listeting FRAMES");
            appLayer.update();                 
            
            /*if(appLayer.hasNewCommand())
            {
               This is checked in the fsm machine! }*/
            // If the FSM previously asked us to send a STATUS back to Master

            if(hasPendingCommandSend)
            {
                appLayer.sendCommand(MASTER, pendingCommandId);
                hasPendingCommandSend = false;
            }
            break;

        case ComState::COM_ERROR:
            currentComState = ComState::COM_LISTEN;   // auto-recover
            break;
    }
    #endif
    #ifdef ROBOT_SLAVE_01
    switch(currentComState)
    {
        case ComState::COM_START:
            currentComState = ComState::COM_LISTEN_PING;
            break;

        case ComState::COM_LISTEN_PING:
          appLayer.update();
          if(appLayer.hasReceivedPing())
          {
            appLayer.clearPingFlag();
            currentComState = ComState::COM_LISTEN;
            Serial.println("Received PING...");
          } 
          break;

          case ComState::COM_LISTEN:
            //Serial.println("Listeting FRAMES");
            appLayer.update();                 
            if(appLayer.hasNewCommand())
            {
                Serial.println("[SLAVE_01] Received PICK_RED. Waiting for FSM to trigger send...");
                currentComState = ComState::COM_WAIT_SEND; 
            }
            break;
          case ComState::COM_WAIT_SEND:
            if(hasPendingCommandSend)
            {
                Serial.printf("[SLAVE_01] FSM triggered send! Sending CMD %d to Node %d...\n", pendingCommandId, pending_id_dest);
                
                if(pendingParamLen > 0) appLayer.sendCommandWithData(pending_id_dest, pendingCommandId, pendingParams, pendingParamLen);
                else appLayer.sendCommand(pending_id_dest, pendingCommandId);
              
                comTimer = millis();
                sendTries = 0;
                currentComState = ComState::COM_WAIT_ACK;
            }
            break;
          
          case ComState::COM_WAIT_ACK:
            appLayer.update(); 
            
            if(sendTries > 20) 
            {
                Serial.println("[SLAVE_01] ERROR: SLAVE_00 not responding. Giving up.");
                hasPendingCommandSend = false; 
                pendingParamLen = 0;
                currentComState = ComState::COM_LISTEN; // Reset to listening
            } 
            else if(appLayer.hasReceivedAck(pendingCommandId))
            {
                Serial.println("[SLAVE_01] ACK Received from SLAVE_00! Mission accomplished.");
                hasPendingCommandSend = false;
                pendingParamLen = 0;
                currentComState = ComState::COM_LISTEN; // Success! Reset to listening
            }
            else if(millis() - comTimer > 2000) 
            {
                // Timeout! Resend the frame.
                comTimer = millis();
                sendTries++;
                Serial.printf("[SLAVE_01] Missed ACK. Resending CMD: %d (Try: %d)\n", pendingCommandId, sendTries);
                
                if(pendingParamLen > 0) appLayer.sendCommandWithData(pending_id_dest, pendingCommandId, pendingParams, pendingParamLen);
                else appLayer.sendCommand(pending_id_dest, pendingCommandId);    
            }
            break;
          case ComState::COM_ERROR:
            currentComState = ComState::COM_LISTEN;   // auto-recover
            break;    
    }
    #endif
}

void robot_t::send_command(uint8_t dst,uint8_t cmdId)
{
  if(!hasPendingCommandSend)//robot_id == MASTER && 
  {
    pending_id_dest = dst;
    pendingCommandId = cmdId;
    pendingParamLen = 0;
    hasPendingCommandSend = true;
  }
}
void robot_t::send_command_param(uint8_t dst, uint8_t cmdId, uint8_t param1)
{
  if( !hasPendingCommandSend)//robot_id == MASTER &&
  {
    pending_id_dest = dst;
    pendingCommandId = cmdId;
    pendingParams[0] = param1;
    pendingParamLen = 1;
    hasPendingCommandSend = true;
  }
}
void robot_t::send_command_param(uint8_t dst, uint8_t cmdId, uint8_t param1, uint8_t param2)
{
  if( !hasPendingCommandSend)//robot_id == MASTER &&
  {
    pending_id_dest = dst;
    pendingCommandId = cmdId;
    pendingParams[0] = param1;
    pendingParams[1] = param2;
    pendingParamLen = 2;
    hasPendingCommandSend = true;
  }
}
#ifdef ROBOT_SLAVE_01
void robot_t::slave01_dropGreenBox() // trigers that the slave01 dropped a green box!
{
  if(robot.front.actuators.isMagnetOn) this->drop_greenBox = false;
  else  this->drop_greenBox = true;
}
#endif


void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc1 * (TWO_PI / (64.0 * 2.0 * 1920.0));
  w2e = enc2 * (TWO_PI / (64.0 * 2.0 * 1920.0));

  v1e = w1e * wheel_radius;//wheel_radius is a paramether that we'll need to estimate so we have a good aproximation
  v2e = w2e * wheel_radius;

  p1e += v1e * dt;
  p2e += v2e * dt;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;

  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta / 2);
  ye += ds * sin(thetae + dtheta / 2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}

void robot_t::setRobotVW(float Vnom, float Wnom)
{
  //Sets que v_requested and w_requested for the robot
  v_req = Vnom;
  w_req = Wnom;
}

float robot_t::getAngleDiff(float target_angle, float current_angle) {
    float diff = target_angle - current_angle;
    
    // Force the difference into the [-pi, pi] range
    while (diff > M_PI) diff -= TWO_PI;
    while (diff < -M_PI) diff += TWO_PI;
    
    return diff;
}

//functions to handle robot actions on the box

void robot_t::grabBox(RobotSide& sideRef){
  sideRef.actuators.magnetOn();
}
void robot_t::dropBox(RobotSide& sideRef){
  sideRef.actuators.magnetOff();
}

bool robot_t::has2Boxes(){
  return front.actuators.isMagnetOn && back.actuators.isMagnetOn;
}


bool robot_t::hasDroped2Boxes(){
  return (!front.actuators.isMagnetOn && !back.actuators.isMagnetOn);
}



void robot_t::followLine_v2(float Vnom, RobotSide& sideRef, Side2Follow direction, EdgeDetection edge)
{
  sideRef.sensor.getLineError_v1(direction,edge);

  float w = sideRef.sensor.erro * sideRef.sensor.kl;

  this->setRobotVW(Vnom,w);
}

void robot_t::followLine(float Vnom, RobotSide& sideRef, Side2Follow direction, EdgeDetection edge)
{
  sideRef.sensor.getLineError(direction,edge);

  float w = sideRef.sensor.erro * sideRef.sensor.kl;

  this->setRobotVW(Vnom,w);
}

void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}

void robot_t::calcMotorsVoltage(void)
{
  if (control_mode == cm_voltage)
  {
    u1 = u1_req;
    u2 = u2_req;

    return;
  }
  else if (control_mode == cm_pid)
  {
    w1ref = w1_req;
    w2ref = w2_req;
  }
  else if (control_mode == cm_kinematics)
  {
    v1ref = v + w * wheel_dist / 2;
    v2ref = v - w * wheel_dist / 2;

    w1ref = v1ref / wheel_radius;
    w2ref = v2ref / wheel_radius;
  }
  else if (control_mode == cm_pos)
  {
    /*
    p1ref = p1_req;
    PID[0].last_pe = PID[0].pos_error;
    PID[0].pos_error = p1ref - p1e;
    w1ref = PID[0].ppars->Kc_p * PID[0].pos_error + PID[0].ppars->Kd_p * (PID[0].pos_error - PID[0].last_pe) / PID[0].ppars->dt;
    u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) * PID[0].ppars->dead_zone;
    */
  }

 
  // PWM_2 = u2 / battery_voltage * 255;
  // PWM_1 = u1 / battery_voltage * 255;
}

