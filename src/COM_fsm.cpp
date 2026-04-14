#include "COM_fsm.h"
#include "config.h"

fsm_COM::fsm_COM(robot_t& r) : robot(r)
{
    force_state(SYS_IDLE);
    sendingTries = 0;
}

void fsm_COM::next_state_rules()
{
    // ==========================================================
    //                    SYSTEM & STARTUP
    // ==========================================================
    auto& intersections = robot.front.sensor.intersections;
    
    if(state == SYS_IDLE && robot.front.actuators.isSwitch_left_On)
    {
        Serial.println("[FSM] Switch pressed. Moving to SYS_CALIBRATION.");
        robot.currentComState = ComState::COM_START; //does PING/PONG
        set_next_state(SYS_CALIBRATION);
    }
    else if(state == SYS_CALIBRATION)
    {
        Serial.println("[FSM] PING/PONG process...");
        
        #ifdef ROBOT_MASTER
        if(robot.currentComState == ComState::COM_WAIT_SEND)
        {
            set_next_state(S_WAIT_CMD_START);
            Serial.println("\n============================================");
            Serial.println("[FSM_MASTER] PING/PONG SUCESS!");
            Serial.println("============================================\n\n");
        } 
        else if(robot.currentComState == ComState::COM_WAIT_PONG)
        {
            Serial.println("[FSM_MASTER] Waiting PONG...");
        }
        else if (robot.currentComState == ComState::COM_ERROR){
            Serial.println("[FSM_MASTER] ERROR in PING/PONG!\n");
            Serial.println("[FSM_MASTER] RUNING WITHOUT COM...\n");
        }
        #endif 
        #ifdef ROBOT_SLAVE
        if(robot.currentComState == ComState::COM_LISTEN){
            set_next_state(SYS_START);
            Serial.println("[FSM_SLAVE] PING received! \n");
        }
        #endif 
    }
    else if(state == S_WAIT_CMD_START && tis > 2)
    {
        #ifdef ROBOT_SLAVE
        // WAIT for the master to tell us to go!
        if(robot.appLayer.hasNewCommand())
        {
            uint8_t cmd = robot.appLayer.getReceivedCmdId();
            
            if(cmd == CMD_ID::CMD_SLAVE_START)
            {
                Serial.println("\n============================================");
                Serial.printf("[FSM_SLAVE] SUCCESS! Received CMD: %d\n", cmd);
                Serial.println("============================================");
                
                robot.appLayer.clearNewCommand(); 
                set_next_state(SYS_LEAVE_START);
            }
        }
        #endif
        
        #ifdef ROBOT_MASTER
        set_next_state(SYS_LEAVE_START);
        #endif
    }
    else if(state == SYS_LEAVE_START)
    {
        #ifdef ROBOT_MASTER 

        // 1. Queue the command at exactly 5 seconds
        if(tis > 5) 
        {
            Serial.println("\n[FSM_MASTER] 5 seconds passed. Sending CMD_SLAVE_START to comms layer!");
            robot.send_command(CMD_ID::CMD_SLAVE_START);
        }
        
        // 2. Wait for the ACK! (The background updateComState will make hasPendingCommandSend false when ACK arrives)
        if(robot.appLayer.hasReceivedAck(CMD_ID::CMD_SLAVE_START))
        {
            Serial.println("\n============================================");
            Serial.println("[FSM_MASTER] SUCCESS! ACK was received!");
            Serial.println("============================================");
            set_next_state(SYS_IDLE);
        }
        
        #endif

        #ifdef ROBOT_SLAVE
        set_next_state(SYS_IDLE);
        #endif
    }
    else if(state == GEN_MOVE_X)
    {
           
    }
}

void fsm_COM::enter_state_actions_rules()
{
}

void fsm_COM::state_actions_rules()
{
}

// ====================================================================
// WARNING: DO NOT ADD void control(robot_t& robot) HERE!
// It will cause the "multiple definition" linker error. 
// Leave it in your main.cpp or fsm_round1.cpp!
// ====================================================================