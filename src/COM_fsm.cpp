#include "COM_fsm.h"
#include "config.h"
#include "fsm_round3.h"

fsm_COM::fsm_COM(robot_t& r) : robot(r)
{
    force_state(SYS_IDLE);
    sendingTries = 0;
}

// ==========================================================
//   ENTRY ACTIONS: Runs EXACTLY ONCE when entering a state
// ==========================================================
void fsm_COM::enter_state_actions_rules()
{
    #ifdef ROBOT_MASTER
    if(state == COM_BOXES_SLAVE_00)
    {
        uint8_t t_box = 3;
        Serial.printf("[MASTER] Sending to SLAVE_00 process %d boxes from Machine B \n", t_box);
        robot.send_command_param(NodeId::SLAVE_00, CMD_ID::INFO_BOX_MACHINE_B, t_box);
    }
    else if(state == COM_BOXES_SLAVE_01_A)
    {
        Serial.printf("[MASTER] Sending to SLAVE_01 process %d boxes from Machine A \n", 1);
        robot.send_command_param(NodeId::SLAVE_01, CMD_ID::INFO_BOX_MACHINE_A, 1);        
    }
    #endif

    #if defined(ROBOT_SLAVE_00) || defined(ROBOT_SLAVE_01)
    if(state == S_WAIT_CMD_START)
    {
        // This will only print ONCE now!
        Serial.println("[SLAVE] Waiting for command from MASTER...");
    }
    #endif
}

// ==========================================================
//   NEXT STATE RULES: Runs Continuously to check conditions
// ==========================================================
void fsm_COM::next_state_rules()
{
    if(state == SYS_IDLE && robot.front.actuators.isSwitch_left_On)
    {
        Serial.println("[FSM] Switch pressed. Moving to INIT (PING/PONG).");
        
        #ifdef ROBOT_MASTER
        robot.currentComState = ComState::COM_START;
        #endif
        
        #if defined(ROBOT_SLAVE_01) || defined(ROBOT_SLAVE_00)
        robot.currentComState = ComState::COM_START;
        #endif
        
        set_next_state(SYS_CALIBRATION); 
    }
    else if(state == SYS_CALIBRATION)
    {
        #ifdef ROBOT_MASTER
        if(robot.currentComState == ComState::COM_WAIT_SEND)
        {
            Serial.println("\n============================================");
            Serial.println("[FSM_MASTER] PING/PONG SUCCESS!");
            Serial.println("============================================\n");
            set_next_state(COM_BOXES_SLAVE_00);
        } 
        else if (robot.currentComState == ComState::COM_ERROR){
            Serial.println("[FSM_MASTER] ERROR in PING/PONG! RUNING WITHOUT COM...\n");
        }
        #endif 

        #if defined(ROBOT_SLAVE_00) || defined( ROBOT_SLAVE_01)
        if(robot.currentComState == ComState::COM_LISTEN){
            Serial.println("[FSM_SLAVE] PING received! \n");
            set_next_state(S_WAIT_CMD_START);
        }
        #endif 
    }
   
    // --- MASTER LOGIC ---
    #ifdef ROBOT_MASTER
    else if(state == COM_BOXES_SLAVE_00)
    {
        // Because the command was sent in enter_state, we ONLY check for the ACK here!
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_B))
        {
            Serial.println("[MASTER] ACK received from SLAVE_00!");
            set_next_state(COM_BOXES_SLAVE_01_A);
        }
    }
    else if(state == COM_BOXES_SLAVE_01_A)
    {
        // Same here, just wait for the ACK!
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_A))
        {
            Serial.println("[MASTER] ACK received from SLAVE_01!");
            set_next_state(M_SYS_LEAVE_START);
        }
    }
    else if(state == M_SYS_LEAVE_START)
    {
        Serial.println("[MASTER] all COM ok, leaving START area!");
        // Add your logic to physically move the Master here
    }
    #endif

    // --- SLAVE LOGIC ---
    #if defined(ROBOT_SLAVE_00) || defined(ROBOT_SLAVE_01)
    else if(state == S_WAIT_CMD_START)
    {
        if(robot.appLayer.hasNewCommand())
        {
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            Serial.print("Received a Command: ");
            Serial.println(cmdId);

            #ifdef ROBOT_SLAVE_00
            if (cmdId == CMD_ID::INFO_BOX_MACHINE_B) 
            {
                uint8_t val = robot.appLayer.getReceivedParams()[0];
                Serial.printf("[SLAVE_00] received: %d\n", val);
                set_next_state(S_NAV_MACHINE_OUT);     
            }
            #endif

            #ifdef ROBOT_SLAVE_01
            if (cmdId == CMD_ID::INFO_BOX_MACHINE_A) 
            {
                uint8_t val = robot.appLayer.getReceivedParams()[0];
                Serial.printf("[SLAVE_01] received: %d\n", val);
                set_next_state(S_NAV_MACHINE_OUT);     
            }
            #endif

            // ALWAYS CLEAR INSIDE THE IF STATEMENT AFTER PROCESSING!
            robot.appLayer.clearNewCommand(); 
        }
    }
    #endif
}

// ==========================================================
//   STATE ACTIONS: Runs Continuously while inside the state
// ==========================================================
void fsm_COM::state_actions_rules()
{
    // Leave this completely empty! 
    // Continuous logic (like line following) goes here later.
}