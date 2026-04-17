#include "fsm_round3.h"

fsm_round3:: fsm_round3(robot_t& r) : robot(r)
{
    force_state(SYS_IDLE);

    current_box_index = 0;
    drop_sequence[0] = 3;
    drop_sequence[1] = 2;
    drop_sequence[2] = 1;
    drop_sequence[3] = 0;

    #ifdef ROBOT_MASTER
    this->MASTER_blueBox = 0;
    #endif

    #ifdef ROBOT_SLAVE_01
    this->SLAVE_blueBox = 0;
    //Slave flag to blue Pick Slots
    this->hasBlueBoxesInfo = false;
    #endif
    

}


void fsm_round3::next_state_rules()
{
    auto& intersections = robot.front.sensor.intersections;
    // ==========================================================
    //                    SYSTEM & STARTUP (Ambos os Robôs)
    // ==========================================================
    if(state == SYS_IDLE && robot.front.actuators.isSwitch_left_On)
    {
        set_next_state(SYS_CALIBRATION);
    }
    else if(state == SYS_CALIBRATION && (robot.thetae > 6.2 || !CALIBRATION_MODE ))
    {
        for(int i = 0; i<5; i++) {
            robot.front.sensor.minValues[i] = robot.front.sensor.minValues[i] - 5;
            robot.front.sensor.maxValues[i] = robot.front.sensor.maxValues[i] + 10;
        }
        set_next_state(COM_INIT);
    }
    else if(state == COM_INIT)
    {
        #ifdef ROBOT_MASTER
        //Serial.printf("[MASTER in COM_INIT]");
        if(robot.currentComState == ComState::COM_WAIT_SEND) set_next_state(SYS_WAIT_IR);
        else if (robot.currentComState == ComState::COM_ERROR)
        {
            Serial.printf("COM_INIT failed!");
        };
        #endif 
        
        #if defined(ROBOT_SLAVE_01) || defined(ROBOT_SLAVE_00)
        if(robot.currentComState == ComState::COM_LISTEN){
            Serial.println("COM INIT SUCCESS!");
            set_next_state(S_WAIT_BOX_INFO);
        }
        #endif 
    }
    else if(state == SYS_WAIT_IR && tis > 2) //Simulate 2 second waitting!
    {
        //if(IR_received) - process the message! 
        #ifdef ROBOT_MASTER
        //_________________________________//
        // INITIAL BOX LOGIC               //
        //_________________________________//
        build_sequence_from_IR("Wowou");//processBox_MachineA =total_reds &&  processBox_MachineB = total_reds+total_greens 
        build_blueBoxPick();
        set_next_state(COM_BOXES_SLAVE_00);
        //blue box Pick will be only for one slave to do (if we had 3 blue boxes? here will be good to send other slave!) 
        #endif
    }
   
    // ==========================================================
    //               LÓGICA EXCLUSIVA DO MASTER 
    // ==========================================================
    #ifdef ROBOT_MASTER
    if(state == COM_BOXES_SLAVE_00)
    {
        //TELL SLAVE_00 how many boxes he will proces@Machine B:
        uint8_t t_box = this->total_reds + this->total_greens;
        if(t_box > 0)
        {
            Serial.printf("[MASTER] Sending to SLAVE_0 process %d boxes from Machine B!\n", t_box);
            robot.send_command_param(NodeId::SLAVE_00, CMD_ID::INFO_BOX_MACHINE_B,t_box);
        } 
        else if(this->total_blues > 0) robot.send_command_param(NodeId::SLAVE_00, CMD_ID::INFO_BLUE_BOX, this->total_blues); //there are 4 blue boxes! 
        //In the case of 4 blue boxes - we should call the fsm_round1 !!! so this never appens! 

        //we wait an ack of the INFO_BOX_MAHCHINE_B!!!!
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_B))
        {
            Serial.printf("[MASTER] ack received from SLAVE_00!\n");
            set_next_state(COM_BOXES_SLAVE_01);
        }
        
    }
    else if(state == COM_BOXES_SLAVE_01)
    {
        //TELL SLAVE_01 how many boxes he will proces@Machine B:
        if(this->total_reds > 0){
            Serial.printf("[MASTER] Sending to SLAVE_01 process %d boxes from Machine A \n",this->total_reds);
            robot.send_command_param(NodeId::SLAVE_01, CMD_ID::INFO_BOX_MACHINE_A, this->total_reds);
        }
        else if(this->total_blues > 0) robot.send_command_param(NodeId::SLAVE_01, CMD_ID::INFO_BLUE_BOX, this->total_blues); //there are 4 blue boxes! 
        //In the case of 4 blue boxes - we should call the fsm_round1 !!! so this never appens! 

        //we wait an ack of the INFO_BOX_MAHCHINE_B!!!!
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_A))
        {
            Serial.printf("[MASTER] ack received from SLAVE_01!\n");
            set_next_state(M_SYS_LEAVE_START);
        }
    }
    else if(state == M_SYS_LEAVE_START && intersections == 3)
    {
        //here send the slots to the robot slave when entering this state!
        //build_currentBox(this->currentBox);
        set_next_state(GEN_PICK_ZONE);
    }



    // ====================================
    //     MASTER process Red BOX!:
    // ====================================
    else if(state == M_NAV_DROP_RED)
    {
        if(intersections == 1 && tis == 0.20/robot.v_req ) intersections = 0; // reset if count wrong
        if(intersections == 1)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = M_DROP_ALIGN_RED;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == M_DROP_ALIGN_RED && tis > 3)
    {
        //SEND SLAVE_01 pick box!!!
        robot.send_command(NodeId::SLAVE_01,CMD_ID::CMD_EXECUTE_PICK_RED);
        set_next_state(M_DROP_TURN_OUT_RED);
    }
    else if(state == M_DROP_TURN_OUT_RED)
    {
        current_box_index++;//DROP A BOX! increment box Index!
        target_distance = d_retrive_process_box;
        move_direction = -1;
        turn_direction = 1;
        target_turn_angle = PI/2;
        state_after_maneuver = M_NAV_PICK_FROM_RED;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == M_NAV_PICK_FROM_RED) 
    {
        if(intersections == 1)
        {
            set_next_state(GEN_PICK_ZONE);
        }
    }

    // ====================================
    //     MASTER process GREEN BOX!:
    // ====================================
    else if(state == M_NAV_PROCESS_GREEN_BOX)
    {
        if(intersections == 2 ) //Let's do edge:UP! 
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = M_GEN_DROP_ALIGN;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == M_GEN_DROP_ALIGN && tis > 3) 
    {
        robot.send_command(NodeId::SLAVE_00, CMD_ID::CMD_EXECUTE_PICK_GREEN);
        set_next_state(M_GEN_DROP_TURN_OUT);
    }
    else if(state == M_GEN_DROP_TURN_OUT)
    {
        //MASTER DROPED BOX!
        current_box_index++;
        target_distance = d_retrive_process_box;
        move_direction = this->total_greens > 0 ? -1 : 1;
        turn_direction = 1;
        target_turn_angle = PI/2;
        state_after_maneuver = this->total_greens > 0 ? M_EXT_PROC_MACH_GREEN : M_EXT_PROC_MACH_BLUE;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == M_EXT_PROC_MACH_GREEN)
    {
        if (robot.front.sensor.intersections == 1 )
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = M_NAV_FROM_MACHINE;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == M_NAV_FROM_MACHINE )
    {
        if(intersections == 1 && tis == 0.20/robot.v_req ) intersections == 0; // reset if count wrong
        if(intersections == 1)
        {
            isFromMachine = true;
            set_next_state(GEN_PICK_ZONE);
        }
    }
    #endif


    // ==========================================================
    //               LÓGICA EXCLUSIVA DO SLAVE_00
    // ==========================================================
    #ifdef ROBOT_SLAVE_00 
    // 1. DORMIR À ESPERA DA CAIXA NA MÁQUINA
    if(state == S_WAIT_BOX_INFO)
    {
        if(robot.appLayer.hasNewCommand())
        {
            Serial.print("Received a Command: ");
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            Serial.println(cmdId);

            if (cmdId == INFO_BOX_MACHINE_B) 
            {
                if (robot.appLayer.getReceivedParamLen() > 0) 
                {
                    const uint8_t* params = robot.appLayer.getReceivedParams();
                    uint8_t val = params[0]; // This is the 1 byte you sent from Master
                    Serial.print("Value received:");
                    Serial.println(val);
                    processBox_MachineB = val;//This is what we need to perform 
                    //build_blueBoxPick(total_greens,NodeId::SLAVE); only called in the master side! 
                    Serial.println("GOING TO NAV_MACHINE OUT....");
                    set_next_state(S_NAV_MACHINE_OUT);//for now we will move the slave to machine OUTPUT!     
                }
            }
        }
        robot.appLayer.clearNewCommand();
    }
    
    else if(state == S_NAV_MACHINE_OUT)
    {
        if(intersections == 5)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/3;
            state_after_maneuver = S_NAV_TO_MACHINE_FROM_WHOUSE;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if (state == S_NAV_TO_MACHINE_FROM_WHOUSE )
    {
        if(intersections == 1 && tis < 0.1/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!
        if(intersections == 2)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = S_WAIT_PICK_CMD;
            set_next_state(GEN_MOVE_X);
        }
        
    }
    else if(state == S_WAIT_PICK_CMD)
    {
        if(robot.appLayer.hasNewCommand())
        {
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            if (cmdId == CMD_EXECUTE_PICK_GREEN) 
            {
                //SHOULD I RESET HERE?!!!!!!
                set_next_state(S_MACHINE_ALIGN_PICK);
            }
            //RESET THE COM FLAGS!
            robot.appLayer.clearNewCommand();
        }
        //else set_next_state(S_MACHINE_ALIGN_PICK);
    }
    else if(state == S_MACHINE_ALIGN_PICK && ((robot.front.actuators.isSwitch_left_On || robot.front.actuators.isSwitch_right_On)) )
    {
        set_next_state(S_MACHINE_PICK_BOX);
    }

    else if(state == S_MACHINE_PICK_BOX && tis > 0.7) set_next_state(S_MACHINE_TURN_OUT);
    
    else if(state == S_MACHINE_TURN_OUT)
    {
        target_distance = d_retrive_process_box;
        move_direction = -1;
        turn_direction = 1;
        target_turn_angle = PI/3;
        state_after_maneuver = S_NAV_MACHINE_TO_DROP;
        set_next_state(GEN_MOVE_X);
    }
    else if (state == S_NAV_MACHINE_TO_DROP && intersections == 2)
    {
        //Let's see the drop slots left! 
        //let's leave the slot 0 for a blue box! 
        //I will consider that the green boxes will be placed in higher positions first!!
        currentBox.drop_slot = drop_sequence[current_box_index++];
        set_next_state(GEN_DROP_BOX);
    }
    else if(state == S_NAV_EXIT_DROP_ZONE_2_MACHINE)//Follow LEFT
    {
        if(currentBox.drop_slot == 0)
        {
            set_next_state(S_NAV_TO_MACHINE_FROM_WHOUSE);
        }
        if(currentBox.drop_slot - intersections == 0)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/3;
            state_after_maneuver = S_NAV_TO_MACHINE_FROM_WHOUSE;
            set_next_state(GEN_MOVE_X);
        }
    }

    // 2. NAVEGAR PARA O OUTPUT DA MÁQUINA B
    
    #endif

    // ==========================================================
    //               LÓGICA EXCLUSIVA DO SLAVE_01
    // ==========================================================
    #ifdef ROBOT_SLAVE_01
    if(state == S_WAIT_BOX_INFO)
    {
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
                    Serial.print("Value received:");
                    Serial.println(val);
                    processBox_MachineA = val;//This is what we need to perform 
                    //build_blueBoxPick(total_greens,NodeId::SLAVE); only called in the master side! 
                    Serial.println("GOING TO NAV_MACHINE OUT....");
                    set_next_state(S1_NAV_MACHINE_A);//for now we will move the slave to machine OUTPUT!     
                }
            }
        }
        robot.appLayer.clearNewCommand();
    }
    else if(state == S1_NAV_MACHINE_A)
    {
        //ALWASY BE WATCHING FOR BLUE BOX INFO!! while going to process box output!   
        if(!hasBlueBoxesInfo)
        {   
            if(robot.appLayer.hasNewCommand())
            {
                Serial.printf("NEW COMAND RECEIVED: ");
                uint8_t cmdId = robot.appLayer.getReceivedCmdId();
                Serial.println(cmdId);
                if(cmdId == INFO_BLUE_PICK_SLOT && !hasBlueBoxesInfo)//should be value 5!
                {
                    Serial.println("STORING VALUES.......");
                    uint8_t len = robot.appLayer.getReceivedParamLen();
                    Serial.printf("Len of params received: %d\n",len); 
                    const uint8_t* params = robot.appLayer.getReceivedParams();
                    this->SLAVE_blueBox = len;

                    if (len >= 1) this->S_blue_PICK[0] = params[0]; 
                    if(len == 2) this->S_blue_PICK[1] = params[1];
                    hasBlueBoxesInfo = true;
                    Serial.printf("[SLAVE_01] blue pick slot:");
                    for(int i = 0; i< SLAVE_blueBox; i++) Serial.printf("%d-",S_blue_PICK[i]);
                }
            }
        }
        if(intersections == 3)
        {

        }
    }
    else if(state == S_WAIT_PICK_CMD) // SET ROBOT TO STOP!
    {
        if(robot.appLayer.hasNewCommand())
        {
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            if (cmdId == CMD_EXECUTE_PICK_RED) 
            {
                set_next_state(S1_ALIGN_PICK_A);
            }
            //RESET THE COM FLAGS!
            robot.appLayer.clearNewCommand();
        }
    }
    else if(state == S1_ALIGN_PICK_A && ((robot.front.actuators.isSwitch_left_On || robot.front.actuators.isSwitch_right_On)))
    {
        set_next_state(S1_PICK_BOX_A);//entering here, activate the magnet!!!!!
    }
    else if(state == S1_PICK_BOX_A && tis > 0.7)
    {
        target_distance = d_retrive_process_box;
        move_direction = -1;
        turn_direction = -1;
        target_turn_angle = PI/3;
        state_after_maneuver = S1_NAV_DROP_B;
        set_next_state(GEN_MOVE_X);
    }
    else if (state == S1_NAV_DROP_B && intersections == 1)
    {
        target_distance = d_mv_aft_intersection;
        move_direction = 1;
        turn_direction = -1;
        target_turn_angle = PI/3;
        state_after_maneuver = S1_ALIGN_DROP_B;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == S1_ALIGN_DROP_B && tis> 3)
    {
        robot.front.actuators.magnetOff();
        //Turn OUT!
        current_box_index++;
        processBox_MachineA--;
        target_distance = d_retrive_process_box;
        move_direction = -1;
        turn_direction = -1;
        target_turn_angle = PI/2;
        if(processBox_MachineA > 0 ) state_after_maneuver = S1_NAV_MACHINE_A_FROM_B;
        else if(processBox_MachineA == 0 && SLAVE_blueBox > 0 ) state_after_maneuver = M_EXT_PROC_MACH_BLUE;
        else state_after_maneuver = NAV_DOCKING_STATION;
        set_next_state(GEN_MOVE_X);
    }    

    #endif
    // ==========================================================
    //               GEN_MOVE & GEN_TURN
    // ==========================================================
    else if(state == GEN_MOVE_X)
    {
        float distance_moved = abs(robot.rel_s - ref_s);
        if(distance_moved >= target_distance)
        {
            set_next_state(GEN_TURN_90);
            ref_s = 0;
        }
    }
    else if(state == GEN_TURN_90)
    {
        float angle_moved = abs(robot.getAngleDiff(ref_theta,robot.rel_theta));
        if(angle_moved >= target_turn_angle)
        {
            set_next_state(state_after_maneuver);
            state_after_maneuver = 0;//reset
            turn_direction = 0;//reset
        }
    }
    //------EXIT FROM DROP@Input MACHINE B to pick blue box! 
    else if(state == M_EXT_PROC_MACH_BLUE && intersections == 4)
    {
        target_distance = d_mv_aft_intersection;
        state_after_maneuver = NAV_TO_WEARHOUSE;
        turn_direction = -1;
        move_direction = 1;
        target_turn_angle = PI/2; //45
        set_next_state(GEN_MOVE_X);
    }
    // ==========================================================
    //                GENERIC PICK BOX & DROP BOX
    // ==========================================================
    else if(state == GEN_PICK_ZONE)
    {
        //SEE HOW MANY boxes there are to process - if we have green to process - send to pick Area!
        //Or I should send the total boxes and the robots agree at the begining who takes what?!
        if(!isFromMachine)
        {
            if(currentBox.pick_slot == 0) set_next_state(GEN_PICK_ALIGN);
            else
            {
                robot.setRobotVW(0,0);
                
                target_distance = d_mv_aft_intersection;
                move_direction = 1;
                turn_direction = -1;
                target_turn_angle = PI/2;
                state_after_maneuver = GEN_PICK_COUNT_NAV_FROM_START;
                set_next_state(GEN_MOVE_X);
            }
        }
        else
        {
            set_next_state(GEN_PICK_COUNT_FROM_MACHINE);    
        }
    }
    else if(state == GEN_PICK_COUNT_NAV_FROM_START)
    {
        if(currentBox.pick_slot == robot.front.sensor.intersections)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = GEN_PICK_ALIGN;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == GEN_PICK_COUNT_FROM_MACHINE)
    {
        if(3 - currentBox.pick_slot == intersections)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = -1;
            target_turn_angle = PI/2;
            state_after_maneuver = GEN_PICK_ALIGN;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == GEN_PICK_ALIGN && ((robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On)))
    {
        set_next_state(GEN_PICK_BOX);
    }
    else if(state == GEN_PICK_BOX && tis > 0.7)
    {
        set_next_state(GEN_PICK_TURN_OUT);
    }
    else if(state == GEN_PICK_TURN_OUT)
    {
        target_distance = d_retrive_from_wearhouse;
        move_direction = -1;

        if(currentBox.color == 'r')
        {
            //need to Process the RED BOX!!!
            turn_direction = 1;
            target_turn_angle = currentBox.pick_slot == 0 ? PI : PI/2;
            state_after_maneuver = EXITING_PICK_ZONE_RED;
        }
        else
        {
            turn_direction = -1;
            target_turn_angle = PI/2;
            state_after_maneuver = EXITING_PICK_ZONE;
        } 
        set_next_state(GEN_MOVE_X);
    }
    else if(state == EXITING_PICK_ZONE)
    {
        if( 3 - robot.front.sensor.intersections == currentBox.pick_slot)
        {
            isFromMachine = false;
            if(currentBox.color == 'g') set_next_state(M_NAV_PROCESS_GREEN_BOX);
            else if(currentBox.color == 'b') set_next_state(NAV_LEAVING_WEARHOUSE);
        }
    }
    else if(state == EXITING_PICK_ZONE_RED)
    {
        if(currentBox.pick_slot - intersections == 0) set_next_state(M_NAV_DROP_RED);
    }

    else if(state == GEN_DROP_BOX)
    {
        //GREEN BOX- SLAVE CALCULATES IN S_NAV_MACHINE_TO_DROP
        if(currentBox.drop_slot == 0)
        {
            set_next_state(GEN_DROP_ALIGN);
        }
        else
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = -1;
            target_turn_angle = PI/2;
            state_after_maneuver = GEN_DROP_COUNT;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == GEN_DROP_COUNT)//inside here I do follow Right!
    {
        if(currentBox.drop_slot == robot.front.sensor.intersections)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = GEN_DROP_ALIGN;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == GEN_DROP_ALIGN && tis > 2)
    {
        #ifdef ROBOT_SLAVE_00 // is the only one that doesn't care! his mission is to drop from M_B! 
        processBox_MachineB--;
        #endif

        if(currentBox.color == 'b')
        {
            #ifdef ROBOT_MASTER
            MASTER_blueBox--;
            #endif
            #ifdef ROBOT_SLAVE_01
            SLAVE_blueBox--;
            #endif
        } 
        set_next_state(GEN_DROP_TURN_OUT);//when entering Turn Off the Magnet! 
    }
    else if(state == GEN_DROP_TURN_OUT)//@OutGoing WearHouse! 
    {
        target_distance = d_retrive_from_wearhouse;
        move_direction = -1;
        target_turn_angle = PI/2;

        #ifdef ROBOT_SLAVE_00
        if(processBox_MachineB == 0)//finished MISSION! 
        { 
            turn_direction = -1;
            state_after_maneuver = EXITING_DROP_ZONE;
        }
        else
        {
            state_after_maneuver = S_NAV_EXIT_DROP_ZONE_2_MACHINE;
            turn_direction = 1; 
            target_turn_angle = (currentBox.drop_slot == 0)? PI: PI/2;
        }
        #endif
        
        #if defined(ROBOT_MASTER) || defined( ROBOT_SLAVE_01)
        turn_direction = -1;
        state_after_maneuver = EXITING_DROP_ZONE;
        #endif

        set_next_state(GEN_MOVE_X);

        
    }
    else if(state == EXITING_DROP_ZONE)
    {

        if( 3 - robot.front.sensor.intersections == currentBox.drop_slot)
        {    
            #ifdef ROBOT_MASTER
            if(MASTER_blueBox > 0) set_next_state(NAV_LEAVING_WEARHOUSE);
            else set_next_state (NAV_DOCKING_STATION);
            #endif
            #ifdef ROBOT_SLAVE_01
            if(SLAVE_blueBox > 0) set_next_state(NAV_LEAVING_WEARHOUSE);
            else set_next_state (NAV_DOCKING_STATION);
            #endif
            
            #ifdef ROBOT_SLAVE_00
            set_next_state(NAV_DOCKING_STATION); 
            #endif
        
        
        }
    }
    else if(state == NAV_LEAVING_WEARHOUSE )
    {
        if(intersections == 1 && tis < 0.1/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!
        if(intersections == 2)
        {
            target_distance = d_mv_aft_intersection;
            state_after_maneuver = NAV_TO_WEARHOUSE;
            turn_direction = -1;
            move_direction = 1;
            target_turn_angle = PI/2; //45
            set_next_state(GEN_MOVE_X);
        }
        
    }
    else if(state == NAV_TO_WEARHOUSE)
    {
        if(intersections == 1 && tis < 0.1/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!

        if(intersections == 3)
        {
            if(robot.front.actuators.isMagnetOn)
            {
                currentBox.drop_slot = drop_sequence[current_box_index++];
                set_next_state(GEN_DROP_BOX);
            }
            else
            {
                set_next_state(GEN_PICK_ZONE);
            }
        }
    }
    else if(state == NAV_DOCKING_STATION)
    {
        robot.setRobotVW(0,0);
    }
}




void fsm_round3:: enter_state_actions_rules()
{
    // ==========================================================
    //                    SYSTEM & STARTUP
    // ==========================================================
    if(state == SYS_IDLE)
    {
        robot.front.actuators.magnetOff();
        robot.setRobotVW(0,0);
        robot.thetae = PI*0.5;
        //Check this!
        robot.xe = -69.5;
        robot.ye = -35.5;
        robot.dropBox(robot.front);
        robot.front.sensor.intersections = 0;
        robot.front.sensor.readValues();
    }
    else if(state == COM_INIT)
    {
        robot.currentComState = ComState::COM_START;//transit to start PING/PONG
    }
    else if(state == M_SYS_LEAVE_START)
    {
        robot.front.sensor.intersections = 0;
    }

    // ==========================================================
    //             GENERIC NAV to PROCESS BOX Green/Red
    // ==========================================================
    else if(state == M_NAV_PROCESS_GREEN_BOX || state == M_EXT_PROC_MACH_GREEN || 
            state == M_NAV_FROM_MACHINE        || state == M_EXT_PROC_MACH_BLUE  ||
            state == EXITING_PICK_ZONE_RED   || state == M_NAV_DROP_RED || 
            state == M_NAV_PICK_FROM_RED )
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == M_GEN_DROP_TURN_OUT || state == M_DROP_TURN_OUT_RED)
    {
        robot.front.actuators.magnetOff();
    }
    

    
    else if(state == GEN_MOVE_X)
    {
        ref_s = robot.rel_s;
    }
    else if(state == GEN_TURN_90)
    {
        ref_theta = robot.rel_theta;
    }

    // ==========================================================
    //                 GENERIC PICK BOX
    // ==========================================================
    else if(state == GEN_PICK_ZONE)
    {
        //BUILD CURRENT BOX TO PICK when entering GEN_PICK_ZONE!!!!!
        build_currentBox(currentBox);
    }
    else if(state == GEN_PICK_COUNT_NAV_FROM_START || state == EXITING_PICK_ZONE || state == GEN_PICK_COUNT_FROM_MACHINE)
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == GEN_PICK_BOX)
    {
        robot.thetae = PI*0.5;
        robot.front.actuators.magnetOn();//Only need to tell the hardware once to turn on the magnet ON
    }

    // ==========================================================
    //                 GENERIC DROP BOX
    // ==========================================================
    else if(state == GEN_DROP_BOX || state == GEN_DROP_COUNT || 
            state == GEN_DROP_ALIGN)
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == GEN_DROP_TURN_OUT)
    {
        robot.front.actuators.magnetOff();
    }

    // ==========================================================
    //                 GENERIC NAV_TO_WEARHOUSE 
    // ==========================================================
    else if(state == NAV_TO_WEARHOUSE || state == NAV_LEAVING_WEARHOUSE)
    {
        
        if(robot.front.actuators.isMagnetOn)
        {
            //update pose:
            //robot.xe = 
            //robot.ye =
            robot.thetae = -PI/2;
        }
        else 
        {
            //update pose:
            //robot.xe = 
            //robot.ye =
            robot.thetae = PI/2;
        }
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    else if(state == EXITING_DROP_ZONE)
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }

    // ==========================================================
    //                 SLAVE ROBOT:
    // ==========================================================
    else if(state == S_NAV_MACHINE_OUT || state == S_NAV_TO_MACHINE_FROM_WHOUSE ||
            state == S_NAV_EXIT_DROP_ZONE_2_MACHINE|| state == S_NAV_MACHINE_TO_DROP)
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    else if(state == S_MACHINE_PICK_BOX)
    {
        robot.front.actuators.magnetOn();
    }
    

}

void fsm_round3::state_actions_rules()
{
    auto& intersections = robot.front.sensor.intersections;   
    
    // Run every loop while in current state
    float v_nom, w_nom = 0.0f;
    
    // ==========================================================
    //                    SYSTEM & STARTUP
    // ==========================================================
    if(state == SYS_CALIBRATION)
    {
        if(CALIBRATION_MODE)
        {
            robot.setRobotVW(0, 0.6);
            robot.front.sensor.calibrate();  
        }
        else{
            robot.front.sensor.setCalibration(HARDCODED_FRONT_MIN,HARDCODED_FRONT_MAX);
        }
    }
    else if(state == SYS_LEAVE_START)
    {
        v_nom = 0.08;
        robot.followLine(v_nom, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
    }
    else if(state == SYS_APPROACH_WAREHOUSE)
    {
        robot.setRobotVW(0.05, 0);
    }

    // ==========================================================
    //             GENERIC NAV to PROCESS BOX
    // ==========================================================
    #ifdef MASTER
    else if(state == M_SYS_START) robot.setRobotVW(0 ,0); // wait ACK from SLAVE
    else if(state == M_SYS_LEAVE_START)
    {
        //always sending! the slave will only store once! 
        if (this->SLAVE_blueBox == 1) 
        {
            robot.send_command_param( NodeId::SLAVE_01, INFO_BLUE_PICK_SLOT, S_blue_PICK[0]);
        }
        else if (this->SLAVE_blueBox == 2) 
        {
            robot.send_command_param(NodeId::SLAVE_01,INFO_BLUE_PICK_SLOT, S_blue_PICK[0], S_blue_PICK[1]);
        }
        robot.followLine(0.12, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if(state == M_NAV_PROCESS_GREEN_BOX || state == M_GEN_DROP_ALIGN)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection:: DOWN);
    }
    else if(state == M_EXT_PROC_MACH_GREEN)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection:: UP);
        
    }
    else if(state == M_NAV_FROM_MACHINE)
    {
        robot.followLine(0.08, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
    }
    else if(M_EXT_PROC_MACH_BLUE)
    {

        if(intersections == 0)
        {
            robot.front.sensor.getLineError(Side2Follow::LEFT, EdgeDetection:: DOWN);            
            robot.setRobotVW(0.1,0);
        }
        else if(intersections == 1 )
        {
            robot.followLine(0.08, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
        }
        else if(intersections == 2 )
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection:: DOWN);
        }
        else if(intersections == 3)
        {
            robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
        }else robot.setRobotVW(0,0);//Error I want to stop the robot! 
    }
    //RED Box:
    else if(state == M_NAV_DROP_RED || state == M_DROP_ALIGN_RED)
    {
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection:: DOWN);
    }
    else if(state == M_NAV_PICK_FROM_RED)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    #endif
    




    // ==========================================================
    //                 PICK & DROP BOX/ MANUEVERS 
    // ==========================================================
    
    else if(state == GEN_MOVE_X)
    {
        robot.setRobotVW(move_direction* 0.08, 0);
    }
    else if(state == GEN_TURN_90)
    {
        robot.setRobotVW(0.0, turn_direction);//SEE this w velocity! 
    }

    // ==========================================================
    //                 GENERIC PICK BOX
    // ==========================================================
    else if(state == GEN_PICK_COUNT_NAV_FROM_START || state == GEN_PICK_ALIGN || state == EXITING_PICK_ZONE)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
    }
    else if(state == GEN_PICK_COUNT_FROM_MACHINE)
    {
        robot.followLine(0.08, robot.front, Side2Follow::LEFT, EdgeDetection::UP);
    }
    else if(state == GEN_PICK_BOX)
    {
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.04, 0.4);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.04, -0.4);
        }
        else{
            robot.setRobotVW(0.04, 0);
        }
    }
    
    // ==========================================================
    //                 GENERIC DROP BOX
    // ==========================================================
    else if(state == GEN_DROP_COUNT || state == GEN_DROP_ALIGN || state == EXITING_DROP_ZONE)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
    }

    // ==========================================================
    //                       GENERIC NAV_TO_WEARHOUSE
    // ==========================================================

    else if(state == NAV_LEAVING_WEARHOUSE || state == NAV_TO_WEARHOUSE)
    {
        robot.followLine(0.12, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }


    // ==========================================================
    //                       SLAVE ACTIONS:
    // ==========================================================
    #ifdef ROBOT_SLAVE_00

    if(state == S_WAIT_BOX_INFO || state == S_WAIT_PICK_CMD)
    {
        robot.setRobotVW(0,0);
    }
    else if(state == S_NAV_MACHINE_OUT)//get out at fifth intersection! 
    {
        if(intersections == 0) robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
        else robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::UP);
    }
    else if(state == S_NAV_TO_MACHINE_FROM_WHOUSE )
    {
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection::UP);
    }
    else if(state == S_MACHINE_ALIGN_PICK)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
        //robot.setRobotVW(0.08,0);// or follow the line?!
    }
    else if(state == S_MACHINE_PICK_BOX)
    {
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.04, 0.4);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.04, -0.4);
        }
        else{
            robot.setRobotVW(0.04, 0);
        }
    }
    else if(state == S_NAV_MACHINE_TO_DROP | state == S_NAV_EXIT_DROP_ZONE_2_MACHINE)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    
    
    #endif
}

#ifdef ROBOT_MASTER
void fsm_round3:: build_sequence_from_IR(String ir_data) 
{
    // RESET às variáveis para garantir que está limpo
    this->total_greens = 0;
    this->total_blues = 0;
    this->total_reds = 0;
    int box_index = 0;
    // u o o u

    for(int i = 0; i < ir_data.length(); i++)
    {
        if(box_index > 3) return;         

        if(ir_data[i] == 'w')   this->red_pick_slots[this->total_reds++] = box_index++;
        else if(ir_data[i] == 'u')  this->green_pick_slots[this->total_greens++] = box_index++;
        else if(ir_data[i] == 'o')  this->blue_pick_slots[this->total_blues++] = box_index++;
    }
}

void fsm_round3::build_blueBoxPick()
{
    if(this->total_blues == 0) {
        this->MASTER_blueBox = 0;
        this->SLAVE_blueBox = 0;
        return;
    }
    //if even number:
    if(this->total_blues % 2 == 0)
    {
        this->MASTER_blueBox =  this->total_blues / 2;
        this->SLAVE_blueBox = this->MASTER_blueBox;
    } 
    else{
        this->MASTER_blueBox =  (uint8_t)(float(this->total_blues)/ 2.0) + 0.5f;
        
        if(this->total_blues - this->MASTER_blueBox >= 0)
            this->SLAVE_blueBox = this->total_blues - this->MASTER_blueBox;                   
    } 
    
    uint8_t currIndex = 0;
    for (int i = 0; i< this->MASTER_blueBox; i++)
    {
        this->M_blue_PICK[i] = this->blue_pick_slots[i];//I want to start getting form the low slots! 
        currIndex++;
    }
    for(int i = 0; i< this->SLAVE_blueBox ; i++)
    {
        this->S_blue_PICK[i] = this->blue_pick_slots[currIndex++]; 
    }   
}
#endif


void fsm_round3::build_currentBox(BoxRound2 &box)//WHAT ABOUT PICK SLOT????
{
   
    #ifdef ROBOT_MASTER
    if(this->total_reds > 0 && this->current_red_index < 4 )
    {
        box.color = 'r';
        box.pick_slot = this->red_pick_slots[this->current_red_index++];
        this->total_reds--;
        //SHOULD I DECREASE THE TOT_REDS STRAITHG!? 
    }
    else if(this->total_greens > 0 && current_green_index < 4) // when we store a green box we decrement?!
    {
        box.color = 'g';
        box.pick_slot = this->green_pick_slots[this->current_green_index++];
        this->total_greens--;
        //what should be the dropSlot?!
    }
    else if(this->MASTER_blueBox > 0 && current_blue_index < 2)
    {
        box.color = 'b';
        box.pick_slot = this->S_blue_PICK[this->current_blue_index++];
        box.drop_slot = drop_sequence[this->current_box_index]; 
        this->total_blues--;
    }

    #endif

    #ifdef ROBOT_SLAVE_01
    if(this->SLAVE_blueBox > 0 && current_blue_index < 2)
    {
        box.color = 'b';
        box.pick_slot = this->S_blue_PICK[this->current_blue_index++];
        this->SLAVE_blueBox--;
    }
    #endif   
    //current_box_index++; Do we add to the next box!????- better to do when a robot drop box!
}