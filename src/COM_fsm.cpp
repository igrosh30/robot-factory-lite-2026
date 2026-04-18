#include "COM_fsm.h"
#include "config.h"
#include "fsm_round3.h"

fsm_COM::fsm_COM(robot_t& r) : robot(r)
{
    force_state(SYS_IDLE);
    sendingTries = 0;
}

void fsm_COM::next_state_rules()
{
    //
    // ==========================================================
    //                    SYSTEM & STARTUP
    // ==========================================================
    if(state == SYS_IDLE && robot.front.actuators.isSwitch_left_On)
    {
        Serial.println("[FSM] Switch pressed. Moving to INIT (PING/PONG).");
        
        #ifdef ROBOT_MASTER
        robot.currentComState = ComState::COM_START; // Starts the Ping sequence
        #endif
        
        #if defined(ROBOT_SLAVE_01) || defined(ROBOT_SLAVE_00)
        robot.currentComState = ComState::COM_START; // Starts listening for Pings
        #endif
        
        set_next_state(SYS_CALIBRATION); // Go to the handshake state!
    }
    else if(state == SYS_CALIBRATION)
    {
        
        #ifdef ROBOT_MASTER
        if(robot.currentComState == ComState::COM_WAIT_SEND)
        {
            set_next_state(COM_BOXES_SLAVE_00);
            Serial.println("\n============================================");
            Serial.println("[FSM_MASTER] PING/PONG SUCESS!");
            Serial.println("============================================\n\n");
            Serial.println("[FSM_MASTER] Waiting for IR....");
        } 
        else if(robot.currentComState == ComState::COM_WAIT_SLAVE00_PONG)
        {
            Serial.println("[FSM_MASTER] Waiting PONG from SLAVE_00...");
        }
        else if(robot.currentComState == ComState::COM_WAIT_SLAVE01_PONG)
        {
            //Serial.println("[FSM_MASTER] Waiting PONG from SLAVE_01...");
        }
        else if (robot.currentComState == ComState::COM_ERROR){
            Serial.println("[FSM_MASTER] ERROR in PING/PONG!\n");
            Serial.println("[FSM_MASTER] RUNING WITHOUT COM...\n");
        }
        #endif 
        #if defined(ROBOT_SLAVE_00) || defined( ROBOT_SLAVE_01)
        if(robot.currentComState == ComState::COM_LISTEN){
            set_next_state(S_WAIT_CMD_START);
            Serial.println("[FSM_SLAVE] PING received! \n");
        }
        #endif 
        
    }
   
    // ==========================================================
    //      MASTER envia no inicio as caixas a processar! 
    // ==========================================================
    #ifdef ROBOT_MASTER

    if(state == COM_BOXES_SLAVE_00)
    {
        // ONLY send if the COM layer is ready AND we haven't already finished the job
        if(!robot.hasPendingCommandSend && !robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_B))
        {
            uint8_t t_box = 3;
            Serial.printf("[MASTER] Sending to SLAVE_00 process %d boxes from Machine B \n", t_box);
            robot.send_command_param(NodeId::SLAVE_00, CMD_ID::INFO_BOX_MACHINE_B, t_box);
        }

        // We wait for an ack of the INFO_BOX_MACHINE_B
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_B))
        {
            Serial.printf("[MASTER] ack received from SLAVE_00!\n");
            set_next_state(COM_BOXES_SLAVE_01);
        }
    }
    
    else if(state == COM_BOXES_SLAVE_01)
    {
        // ONLY send if the COM layer is ready AND we haven't already finished the job
        if(!robot.hasPendingCommandSend && !robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_A))
        {
            Serial.printf("[MASTER] Sending to SLAVE_01 process %d boxes from Machine A \n", 1);
            robot.send_command_param(NodeId::SLAVE_01, CMD_ID::INFO_BOX_MACHINE_A, 1);        
        }
          
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_A))
        {
            Serial.printf("[MASTER] ack received from SLAVE_01!\n");
            set_next_state(M_SYS_LEAVE_START);
        }
    }
    #endif


    else if(state == S_WAIT_CMD_START)
    {
        #ifdef ROBOT_SLAVE_00
        // WAIT for the master to tell us to go!
        if(robot.appLayer.hasNewCommand())
        {
            Serial.print("Received a Command: ");
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            Serial.println(cmdId);

            if (cmdId == INFO_BOX_MACHINE_B ) 
            {
                if (robot.appLayer.getReceivedParamLen() > 0) 
                {
                    const uint8_t* params = robot.appLayer.getReceivedParams();
                    uint8_t val = params[0]; // This is the 1 byte you sent from Master
                    Serial.print("[SLAVE] received:");
                    Serial.println(val);
                    //processBox_MachineB = val;//This is what we need to perform 
                    //build_blueBoxPick(total_greens,NodeId::SLAVE); only called in the master side! 
                    set_next_state(S_NAV_MACHINE_OUT);//for now we will move the slave to machine OUTPUT!     
                }
            }
            robot.appLayer.clearNewCommand();
        }
        #endif
        
        #ifdef ROBOT_MASTER
        set_next_state(SYS_LEAVE_START);
        #endif
        
        #ifdef ROBOT_SLAVE_01
        
        // WAIT for the master to tell us to go!
        if(robot.appLayer.hasNewCommand())
        {
            Serial.print("Received a Command: ");
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            Serial.println(cmdId);

            if (cmdId == INFO_BOX_MACHINE_A) 
            {
                if (robot.appLayer.getReceivedParamLen() > 0) 
                {
                    const uint8_t* params = robot.appLayer.getReceivedParams();
                    uint8_t val = params[0]; // This is the 1 byte you sent from Master
                    Serial.print("[SLAVE] received:");
                    Serial.println(val);
                    //processBox_MachineB = val;//This is what we need to perform 
                    //build_blueBoxPick(total_greens,NodeId::SLAVE); only called in the master side! 
                    set_next_state(S_NAV_MACHINE_OUT);//for now we will move the slave to machine OUTPUT!     
                }
            }
        }
        robot.appLayer.clearNewCommand();
        #endif
    }
    else if(state == M_SYS_LEAVE_START)
    {
        Serial.println("[MASTER] all COM ok, leaving START area!");
    }
    /*
    else if(state == SYS_LEAVE_START)
    {
        #ifdef ROBOT_MASTER 
        if(tis > 5) 
        {
            Serial.println("\n[FSM_MASTER] 5 seconds passed. Sending CMD_SLAVE_START to comms layer!");
            robot.send_command(NodeId::SLAVE_00 ,CMD_ID::CMD_SLAVE_START);
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

        #ifdef ROBOT_SLAVE_00
        set_next_state(SYS_IDLE);
        #endif
    }
    else if(state == GEN_MOVE_X)
    {
           
    }*/
}

void fsm_COM::enter_state_actions_rules()
{
}

void fsm_COM::state_actions_rules()
{
    if(state == S_WAIT_CMD_START) Serial.println("[SLAVE] Waiting command");;
}

