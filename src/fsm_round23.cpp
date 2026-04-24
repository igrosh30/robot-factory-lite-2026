#include "fsm_round23.h"

fsm_round23:: fsm_round23(robot_t& r) : robot(r)
{
    force_state(SYS_IDLE);

    this->v_req_nav = 0.2;
    this -> v_req_leaving_pickZ = 0.15;

    current_box_index = 0;
    
    #ifdef ROBOT_SLAVE_00
    drop_sequence[0] = 0;
    drop_sequence[1] = 3;
    #endif
    #ifdef ROBOT_MASTER
    this->MASTER_blueBox = 0;
    drop_sequence[0] = 3;
    drop_sequence[1] = 2;
    #endif

    #ifdef ROBOT_SLAVE_01
    this->hasBlueBoxesInfo = false;
    this->SLAVE_blueBox = 0;
    drop_sequence[0] = 1;
    drop_sequence[1] = 1;
    #endif
}


void fsm_round23::next_state_rules()
{
    auto& intersections = robot.front.sensor.intersections;
    // ==========================================================
    //                    SYSTEM & STARTUP (Ambos os Robôs)
    // ==========================================================
    if(state == SYS_IDLE && robot.front.actuators.isSwitch_left_On)
    {
        /*
        Serial.println("In state Idle");
        Serial.println("Transitting to WAIT....");
        target_time = 5;
        state_after_timeout = SYS_CALIBRATION;
        set_next_state(GEN_WAIT_Y);*/
        set_next_state(SYS_CALIBRATION);
    }
    else if(state == SYS_CALIBRATION && (robot.thetae > 6.2 || !CALIBRATION_MODE ))
    {
        Serial.print("In CALIBRATE STATE!");
        for(int i = 0; i<5; i++) {
            robot.front.sensor.minValues[i] = robot.front.sensor.minValues[i] - 5;
            robot.front.sensor.maxValues[i] = robot.front.sensor.maxValues[i] + 10;
        }
        /*
        #ifdef RUN_WITOUTH_COM
            #ifdef ROBOT_SLAVE_01
            this->SLAVE_greenBox = 1;
            this->SLAVE_blueBox = 1;
            this->S_green_PICK[0] = 3;
            this->S_blue_PICK[0] = 0;
            set_next_state(S1_LEAVE_START);
            #endif
            #ifdef ROBOT_SLAVE_00
            this->processBox_MachineB = 2;
            set_next_state(SYS_LEAVE_DOCKING);
            #endif
            #ifdef ROBOT_MASTER
            this->MASTER_greenBox = 1;
            this->M_green_PICK[0] = 2;
            this->current_green_index = 0;
            
            this->current_blue_index = 0;
            this->M_blue_PICK[0] = 1;
            this->MASTER_blueBox = 1;
            
            set_next_state(NAV_LEAVING_CENTER);
            #endif
        #endif
        TEST WITHOUT COM! 
        For SLAVE_00:
        #ifdef ROBOT_MASTER
        build_sequence_from_IR("Wwwou");//processBox_MachineA =total_reds &&  processBox_MachineB = total_reds+total_greens 
        build_blueBoxPick();
        set_next_state(M_SYS_LEAVE_START);
        #endif
        #ifdef ROBOT_SLAVE_01
        this->processBox_MachineA = 1;
        this->SLAVE_greenBox = 1;
        this->SLAVE_blueBox = 1;
        this->S_green_PICK[0] = 3;
        this->S_blue_PICK[0] = 1;
        set_next_state(S1_LEAVE_START);

        if(SLAVE_greenBox > 0) set_next_state(S1_LEAVE_START);
        //else set_next_state(S1_NAV_MACHINE_A);
        #endif
        #ifdef ROBOT_SLAVE_00
        this->processBox_MachineB = 3;
        set_next_state(S_NAV_MACHINE_OUT);
        #endif*/
    
        #ifdef ROBOT_MASTER
        //init_boxes = "Wuuoo";
        #endif
        set_next_state(COM_INIT);
    }
    else if(state == COM_INIT)//Enter state triggers state at COM 
    {
        #ifdef ROBOT_MASTER
        set_next_state(M_WAIT_IR);
        #endif
        #ifdef ROBOT_SLAVE_01
        set_next_state(COM_WAIT_BLUE_SLOT);
        #endif

        #ifdef ROBOT_SLAVE_00
        set_next_state(S_WAIT_BOX_INFO);
        #endif

    }
   
    // ==========================================================
    //               LÓGICA EXCLUSIVA DO MASTER 
    // ==========================================================
    #ifdef ROBOT_MASTER
    if(state == M_WAIT_IR && robot.robot_getIR(init_boxes)) //&& robot.robot_getIR(init_boxes) UNCOMENT FOR THE IR RECEPTION!
    {
        //_________________________________//
        // INITIAL BOX LOGIC               //
        //_________________________________//

        build_sequence_from_IR(init_boxes);//processBox_MachineA =total_reds &&  processBox_MachineB = total_reds+total_greens 
        build_slaveSlots();
        set_next_state(COM_BOXES_SLAVE_00);
    }
    else if(state == COM_BOXES_SLAVE_00)
    {
        //the send occurs when entering the state! 
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BOX_MACHINE_B))
        {
            Serial.printf("[MASTER] ack received from SLAVE_00!\n");
            set_next_state(COM_BLUE_SLOT_SLAVE_01);
        }
        
    }
    else if(state == COM_BLUE_SLOT_SLAVE_01)
    {
        //we wait an ack of the INFO_BOX_MAHCHINE_B!!!!
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BLUE_PICK_SLOT))
        {
            Serial.printf("[MASTER] BLUE PICK: ack received from SLAVE_01!\n");
            set_next_state(COM_BOXES_SLAVE_01_GREEN);
        }
    }
    else if(state == COM_BOXES_SLAVE_01_GREEN)
    {
        if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_GREEN_PICK_SLOT))
        {
            Serial.printf("[MASTER] GREEN: ack received from SLAVE_01!\n");
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = NAV_LEAVING_CENTER;
            set_next_state(GEN_TURN_90);
            //set_next_state(SYS_LEAVE_DOCKING);
        }
    }
    else if (state == NAV_LEAVING_CENTER )
    {
        if(intersections > 0 && tis < 0.1/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!
        if(intersections == 1)
        {
            target_distance = d_mv_aft_intersection;
            state_after_maneuver = NAV_TO_WEARHOUSE;
            turn_direction = -1;
            move_direction = 1;
            target_turn_angle = PI/2; //45
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == NAV_PROCESS_GREEN_BOX)
    {
        if(intersections > 0 && tis < 0.15 / robot.v_req) intersections = 0;
        if(intersections == 1)
        {

            //set_next_state(M_WAIT_SLAVE_DROP);
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = -1;
            target_turn_angle = PI/2;
            state_after_timeout  = NAV_PROCESS_GREEN_BOX_ALIGN;
            state_after_maneuver = GEN_WAIT_Y;
            target_time = 4;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == NAV_PROCESS_GREEN_BOX_ALIGN)
    {
        if(intersections > 0 && tis < 0.1 / robot.v_req) intersections = 0;
        if(intersections == 1)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = M_GEN_DROP_ALIGN;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == M_GEN_DROP_ALIGN && tis > 2)//this was a GREEN BOX RIGHT?! 
    {
        robot.send_command(NodeId::SLAVE_00, CMD_ID::CMD_EXECUTE_PICK_GREEN);
        //MASTER DROPED BOX!
        robot.front.actuators.magnetOff();
        current_box_index++;
        target_distance = d_retrive_process_box;
        move_direction = -1;
        target_turn_angle = PI/2;
        
        if(this->MASTER_greenBox > 0 || this->MASTER_blueBox)
        {
            turn_direction = 1;   
            state_after_maneuver = M_EXT_PROC_MACH_GREEN;
        }
        else
        {
            turn_direction = -1;
            state_after_maneuver = SYS_IDLE;
        }
        
        set_next_state(GEN_MOVE_X);
        
    }
    else if(state == M_EXT_PROC_MACH_GREEN)
    {
        if(intersections > 0 && tis < 0.1/robot.v_req) intersections = 0;
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
    else if(state == M_NAV_FROM_MACHINE)
    {
        if(intersections > 0 && tis == 0.20/robot.v_req ) intersections == 0; // reset if count wrong
        if(intersections == 1)
        {
            isFromMachine = true;
            set_next_state(GEN_PICK_ZONE);
        }
    }
    #endif

    else if(state == SYS_LEAVE_DOCKING)
    {
        #ifdef ROBOT_MASTER
        turn_direction = 1;
        state_after_maneuver = NAV_LEAVING_CENTER;
        #endif
        #ifdef ROBOT_SLAVE_00
        turn_direction = -1;
        state_after_maneuver = S_NAV_MACHINE_OUT;
        #endif

        target_distance = d_leave_docking;
        
        move_direction = 1;
        target_turn_angle = PI/2; 
        set_next_state(GEN_MOVE_X);
    }
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
                    //set_next_state(SYS_LEAVE_DOCKING);//for now we will move the slave to machine OUTPUT!     
                    set_next_state(S_NAV_MACHINE_OUT);//for now we will move the slave to machine OUTPUT!     
                }
            }
            robot.appLayer.clearNewCommand();
        }
        
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
        if(intersections > 0 && tis < 0.2/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!
        if(intersections == intersections_trigger)
        {
            target_distance = d_mv_aft_intersection_B;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = DEG_TO_RAD*80;
            state_after_maneuver = S_WAIT_PICK_CMD;
            /*
            if(intersections_trigger == 1) state_after_maneuver = S_WAIT_PICK_CMD_2;
            else state_after_maneuver = S_WAIT_PICK_CMD;*/
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == S_WAIT_PICK_CMD)
    {   
        if(current_box_index == 0)
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
            else if(tis > 12)
            {
                set_next_state(S_MACHINE_ALIGN_PICK);   
                robot.appLayer.clearNewCommand();
            }
        }
        else
        {
            set_next_state(S_MACHINE_ALIGN_PICK);
        }
    }
    else if(state == S_MACHINE_ALIGN_PICK && ((robot.front.actuators.isSwitch_left_On || robot.front.actuators.isSwitch_right_On)) )
    {
        set_next_state(S_MACHINE_PICK_BOX);
    }

    else if(state == S_MACHINE_PICK_BOX && tis > 0.4) set_next_state(S_MACHINE_TURN_OUT);
    
    else if(state == S_MACHINE_TURN_OUT)
    {
        target_distance = d_retrive_process_box;
        move_direction = -1;
        turn_direction = 1;
        target_turn_angle = PI/2;
        state_after_maneuver = S_NAV_MACHINE_TO_DROP;
        set_next_state(GEN_MOVE_X);
    }
    else if (state == S_NAV_MACHINE_TO_DROP)
    {
        if(intersections > 0 && tis < 0.05/robot.v_req) intersections = 0;
        if(current_box_index == 0)
        {
            if(intersections == 1) set_next_state(GEN_DROP_BOX);
        }
        else 
        {
            if(intersections == 2) set_next_state(GEN_DROP_BOX);
        }
        //Let's see the drop slots left! 
        //let's leave the slot 0 for a blue box! 
        //I will consider that the green boxes will be placed in higher positions first!!

    }
    else if(state == S_NAV_EXIT_DROP_ZONE_2_MACHINE)//Follow LEFT
    {
        if(currentBox.drop_slot == 0)
        {
            set_next_state(S_NAV_TO_MACHINE_FROM_WHOUSE);
        }
        else if(currentBox.drop_slot - intersections == 0)
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
    else if(state == COM_WAIT_BLUE_SLOT)
    {
        if(robot.appLayer.hasNewCommand())
        {
            Serial.print("Received a Command: ");
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            //Serial.println(cmdId);

            if (cmdId == CMD_ID::INFO_BLUE_PICK_SLOT) 
            {
                if (robot.appLayer.getReceivedParamLen() > 0) 
                {
                    uint8_t len = robot.appLayer.getReceivedParamLen();
                    Serial.printf("Len of params received: %d\n",len); 
                    const uint8_t* params = robot.appLayer.getReceivedParams();
                    this->SLAVE_blueBox = len;

                    if (len >= 1) this->S_blue_PICK[0] = params[0]; 
                    if(len == 2) this->S_blue_PICK[1] = params[1];
            
                    set_next_state(S1_WAIT_GREEN_SLOT);//for now we will move the slave to machine OUTPUT!     
                }
            }
        }
        robot.appLayer.clearNewCommand();
    }
    else if(state == S1_WAIT_GREEN_SLOT)
    {
        if(robot.appLayer.hasNewCommand())
        {
            Serial.print("Received a Command: ");
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            //Serial.println(cmdId);

            if (cmdId == CMD_ID::INFO_GREEN_PICK_SLOT) 
            {
                if (robot.appLayer.getReceivedParamLen() > 0) 
                {
                    uint8_t len = robot.appLayer.getReceivedParamLen();
                    Serial.printf("Len of params received: %d\n",len); 
                    const uint8_t* params = robot.appLayer.getReceivedParams();
                    
                    this->SLAVE_greenBox = len;

                    if (len >= 1) this->S_green_PICK[0] = params[0]; 
                    if(len == 2) this->S_green_PICK[1] = params[1];
            
                    if(SLAVE_greenBox > 0) set_next_state(S1_LEAVE_START);
                }
            }
        }
        robot.appLayer.clearNewCommand();
    }
    else if(state == S1_LEAVE_START && intersections == 3)
    {
        set_next_state(GEN_PICK_ZONE);
    }

    else if (state == S1_NAV_DROP_B && intersections == 1)
    {
        target_distance = d_mv_aft_intersection;
        move_direction = 1;
        turn_direction = -1;
        target_turn_angle = PI/2;
        state_after_maneuver = S1_ALIGN_DROP_B;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == NAV_PROCESS_GREEN_BOX)
    {
        if(intersections > 0 && tis < 0.15 / robot.v_req) intersections = 0;
        if(intersections == 1)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = -1;
            target_turn_angle = PI/2;
            state_after_maneuver = NAV_PROCESS_GREEN_BOX_ALIGN;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == NAV_PROCESS_GREEN_BOX_ALIGN)
    {
        if(intersections > 0 && tis < 0.1 / robot.v_req) intersections = 0;
        if(intersections == 2)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2; 
            state_after_maneuver = S1_ALIGN_DROP_B;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == S1_ALIGN_DROP_B && tis> 1.9)
    {
        robot.send_command(NodeId::SLAVE_00,CMD_ID::CMD_EXECUTE_PICK_GREEN);
        robot.front.actuators.magnetOff();
        current_box_index++;
        target_distance = d_aft_dropRed;
        move_direction = -1;
        turn_direction = -1;
        target_turn_angle = DEG_TO_RAD*80;
        if(SLAVE_blueBox > 0) state_after_maneuver = S1_LEAVE_CENTER;
        
        set_next_state(GEN_MOVE_X);
    }
    else if(state == S1_LEAVE_CENTER && intersections == 2)
    {
        target_distance = d_mv_aft_intersection;
        state_after_maneuver = NAV_LEAVING_CENTER;
        turn_direction = -1;
        move_direction = 1;
        target_turn_angle = PI/2; //45
        set_next_state(GEN_MOVE_X);
    }
    else if(state == NAV_LEAVING_CENTER )
    {
        if(intersections > 0 && tis < 0.1/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!
        if(intersections == 1)
        {
            target_distance = d_mv_aft_intersection;
            state_after_maneuver = NAV_TO_WEARHOUSE;
            turn_direction = -1;
            move_direction = 1;
            target_turn_angle = PI/2; //45
            set_next_state(GEN_MOVE_X);
        }
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
    else if(state == GEN_WAIT_Y)
    {
        if( tis > target_time)
        {
            set_next_state(state_after_timeout);
        }
    }
    //------EXIT FROM DROP@Input MACHINE B to pick blue box! 
    else if(state == M_EXT_PROC_MACH_BLUE )
    {
        if(intersections > 0 && tis < 0.08/robot.v_req) intersections = 0;
        if(intersections == 3)
        {
            target_distance = d_mv_aft_intersection;
            state_after_maneuver = NAV_LEAVING_CENTER;
            turn_direction = -1;
            move_direction = 1;
            target_turn_angle = PI/2; //45
            set_next_state(GEN_MOVE_X);
        }
    }
    // ==========================================================
    //                GENERIC PICK BOX & DROP BOX                   
    // ==========================================================
    else if(state == GEN_PICK_ZONE)
    {
        #ifdef ROBOT_MASTER
        if(!isFromMachine)
        {
            if(currentBox.pick_slot == 0)
            {
                target_time = 3;
                state_after_timeout = GEN_PICK_ALIGN;
                set_next_state(GEN_WAIT_Y);
            } 
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
            robot.front.sensor.intersections = 0;
            set_next_state(GEN_PICK_COUNT_FROM_MACHINE);    
        }
        #endif
        #ifdef ROBOT_SLAVE_01
        if(!isFromMachine)
        {
            if(currentBox.pick_slot == 0 && current_box_index != 0)
            {
                target_time = 3;
                state_after_timeout = GEN_PICK_ALIGN;
                set_next_state(GEN_WAIT_Y);
            } 
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
            robot.front.sensor.intersections = 0;
            set_next_state(GEN_PICK_COUNT_FROM_MACHINE);    
        }
        #endif
        
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
    else if(state == GEN_PICK_BOX && tis > 0.4)
    {
        if(current_box_index != 0)
        {
            /*
            target_time = 3;
            state_after_timeout = GEN_PICK_TURN_OUT;
            set_next_state(GEN_WAIT_Y);*/
            set_next_state(GEN_PICK_TURN_OUT);
        }
        else{
            set_next_state(GEN_PICK_TURN_OUT);
        }
    }
    else if(state == GEN_PICK_TURN_OUT)
    {
        target_distance = d_retrive_from_wearhouse;
        move_direction = -1;
        turn_direction = -1;
        target_turn_angle =  DEG_TO_RAD*80;
        state_after_maneuver = EXITING_PICK_ZONE;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == EXITING_PICK_ZONE)
    {
        if( 3 - robot.front.sensor.intersections == currentBox.pick_slot)
        {
            //isFromMachine = false;
            if(currentBox.color == 'g') set_next_state(NAV_PROCESS_GREEN_BOX);
            else if(currentBox.color == 'b') set_next_state(NAV_LEAVING_WEARHOUSE);
        }
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
            target_distance = d_mv_aft_intersection_drop;
            move_direction = 1;
            turn_direction = -1;
            target_turn_angle = DEG_TO_RAD*80;
            state_after_maneuver = GEN_DROP_COUNT;
            set_next_state(GEN_MOVE_X);
        }
    }

    else if(state == GEN_DROP_COUNT)//inside here I do follow Right!
    {
        if(currentBox.drop_slot == robot.front.sensor.intersections)
        {
            target_distance = d_mv_aft_intersection_drop;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = DEG_TO_RAD*80;
            state_after_maneuver = GEN_DROP_ALIGN;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == GEN_DROP_ALIGN && tis > 1.7)
    {
        #ifdef ROBOT_SLAVE_00 // is the only one that doesn't care! his mission is to drop from M_B! 
        processBox_MachineB--;
        #endif

        if(currentBox.color == 'b')
        {
            /* THIS WAS DONE WHEN BUILDING THE BOX TO PICK!
            #ifdef ROBOT_MASTER
            MASTER_blueBox--;//This was done 
            #endif
            #ifdef ROBOT_SLAVE_01
            SLAVE_blueBox--;
            #endif*/
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
            target_turn_angle = (currentBox.drop_slot == 0)? DEG_TO_RAD*175: DEG_TO_RAD*80;
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
            else set_next_state (NAV_END_ROUND);
            #endif
            #ifdef ROBOT_SLAVE_01
            if(SLAVE_blueBox > 0) set_next_state(NAV_LEAVING_WEARHOUSE);
            else set_next_state (NAV_END_ROUND);
            #endif
            
            #ifdef ROBOT_SLAVE_00
            set_next_state(NAV_END_ROUND); 
            #endif
        
        
        }
    }
    
    else if(state == NAV_LEAVING_WEARHOUSE )
    {
        if(intersections == 1 && tis < 0.1/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!
        if(intersections == 2)
        {
            state_after_maneuver = NAV_TO_WEARHOUSE;
            turn_direction = -1;
            target_turn_angle = DEG_TO_RAD*70; 
            set_next_state(GEN_TURN_90);
        }
        
    }
    else if(state == NAV_TO_WEARHOUSE)
    {
        if(intersections == 1 && tis < 0.1/robot.v_req) intersections = 0; // reset if after the turn count 1 one int more!
        if(intersections == 3)
        {
            if(robot.front.actuators.isMagnetOn)
            {
                set_next_state(GEN_DROP_BOX);
            }
            else
            {
                #ifdef ROBOT_SLAVE_01                    
                set_next_state(COM_SLV_01_WAIT);
                #endif 
                #ifdef ROBOT_MASTER
                set_next_state(GEN_PICK_ZONE);
                #endif
            }
        }
        /*
        #ifdef ROBOT_SLAVE_01
        if(!robot.front.actuators.isMagnetOn)
        {
            if(intersections == 2) set_next_state(COM_SLV_01_WAIT);
        }
        #endif*/
    }
    else if(state == COM_SLV_01_WAIT && tis > 5)
    {
        set_next_state(GEN_PICK_ZONE);
    }
    else if(state == NAV_DOCKING_STATION)
    {
        robot.setRobotVW(0,0);
    }
    else if(state == NAV_END_ROUND && intersections == 1)
    {
        #ifdef ROBOT_MASTER
        target_distance = d_mv_aft_intersection+0.1;
        #endif
        #ifdef ROBOT_SLAVE_00
        target_distance = d_mv_aft_intersection+0.2;
        #endif
        #ifdef ROBOT_SLAVE_01
        target_distance = d_mv_aft_intersection+0.4;
        #endif
        
        state_after_maneuver = END_ROUND;
        turn_direction = 1;
        move_direction = 1;
        target_turn_angle = PI/2; //45
        set_next_state(GEN_MOVE_X);
    }
    else if(state == END_ROUND)
    {
        float distance_moved = abs(robot.rel_s - ref_s);
        if(distance_moved >= d_leave_docking)
        {
            set_next_state(SYS_IDLE);
            ref_s = 0;
        }
    }
}




void fsm_round23:: enter_state_actions_rules()
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
        robot.setRobotVW(0,0);
        robot.currentComState = ComState::COM_START;
    }
    #ifdef ROBOT_MASTER
    if(state == COM_BOXES_SLAVE_00)
    {
       //TELL SLAVE_00 how many boxes he will proces@Machine B:
        uint8_t t_box = this->total_greens;
        if(t_box > 0)
        {
            Serial.printf("[MASTER] Sending to SLAVE_0 process %d boxes from Machine B!\n", t_box);
            robot.send_command_param(NodeId::SLAVE_00, CMD_ID::INFO_BOX_MACHINE_B,t_box);
        } 
        else if(this->total_blues > 0) robot.send_command_param(NodeId::SLAVE_00, CMD_ID::INFO_BOX_MACHINE_B, this->total_blues);
    }
    else if(state == COM_BLUE_SLOT_SLAVE_01)
    {
        robot.setRobotVW(0,0);
        if (this->SLAVE_blueBox == 1) 
        {
            robot.send_command_param( NodeId::SLAVE_01, INFO_BLUE_PICK_SLOT, S_blue_PICK[0]);
            Serial.print("Sending...");Serial.print(S_blue_PICK[0]);
        }
        else if (this->SLAVE_blueBox == 2) 
        {
            robot.send_command_param(NodeId::SLAVE_01,INFO_BLUE_PICK_SLOT, S_blue_PICK[0], S_blue_PICK[1]);
        }
    }
    else if(state == COM_BOXES_SLAVE_01_GREEN)
    {
        robot.setRobotVW(0,0);
        if (this->SLAVE_greenBox == 1) 
        {
            robot.send_command_param( NodeId::SLAVE_01, INFO_GREEN_PICK_SLOT, S_green_PICK[0]);
            Serial.print("Sending...");Serial.print(S_green_PICK[0]);
        }
        else if (this->SLAVE_greenBox == 2) 
        {
            robot.send_command_param(NodeId::SLAVE_01,INFO_GREEN_PICK_SLOT, S_green_PICK[0], S_green_PICK[1]);
        }
    }
    else if(state== M_WAIT_SLAVE_DROP) robot.setRobotVW(0,0);
    #endif

    

    // ==========================================================
    //             GENERIC NAV to PROCESS BOX Green/Red
    // ==========================================================
    #if defined(ROBOT_SLAVE_01) || defined(ROBOT_MASTER)
    if(state == M_EXT_PROC_MACH_GREEN    || state == M_EXT_PROC_MACH_BLUE ||
        state == EXITING_PICK_ZONE_RED   || state == M_NAV_DROP_RED      || state == M_NAV_PICK_FROM_RED )
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == M_NAV_FROM_MACHINE)
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == M_DROP_TURN_OUT_RED)
    {
        robot.front.actuators.magnetOff();
    }
    else if(state == NAV_PROCESS_GREEN_BOX || state == NAV_PROCESS_GREEN_BOX_ALIGN)
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;

        #ifdef ROBOT_MASTER
        if(this->MASTER_greenBox == 0 )
        {
            robot.send_command(SLAVE_01,CMD_CAN_ENTER_PICK);
        }
        #endif
    }
    #endif
    

    
    else if(state == GEN_MOVE_X || state == END_ROUND)
    {
        robot.thetae = 0;
        ref_s = robot.rel_s;
    }
    else if(state == GEN_TURN_90)
    {
        ref_theta = robot.rel_theta;
    }
    else if(state == GEN_WAIT_Y)
    {
        
        robot.setRobotVW(0,0);
    }

    // ==========================================================
    //                 GENERIC PICK BOX
    // ==========================================================
    else if(state == GEN_PICK_ZONE)
    {
        //BUILD CURRENT BOX TO PICK when entering GEN_PICK_ZONE!!!!!
        build_currentBox(this->currentBox);
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
    else if(state == GEN_DROP_BOX)
    {
        currentBox.drop_slot = drop_sequence[current_box_index++];
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == GEN_DROP_COUNT || state == GEN_DROP_ALIGN)
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
    else if(state == NAV_TO_WEARHOUSE || state == NAV_LEAVING_WEARHOUSE || state == NAV_LEAVING_CENTER)
    {
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
    else if(state == S_NAV_TO_MACHINE_FROM_WHOUSE)
    {
        if(current_box_index == 0 ) intersections_trigger = 1;
        else intersections_trigger = 2;
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;

    }
    else if(state == S_NAV_MACHINE_OUT|| state == S_NAV_EXIT_DROP_ZONE_2_MACHINE
        || state == S_NAV_MACHINE_TO_DROP || state == SYS_LEAVE_DOCKING)
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    else if(state == S_MACHINE_PICK_BOX)
    {
        robot.front.actuators.magnetOn();
    }

    #ifdef ROBOT_SLAVE_01
    if(state ==S1_NAV_MACHINE_A || state == S1_NAV_DROP_B || state == S1_NAV_MACHINE_A_FROM_B )
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    else if(state == S1_PICK_BOX_A)
    {
        robot.front.actuators.magnetOn();
    } 
    else if(state == COM_WAIT_BLUE_SLOT || state == COM_SLV_01_WAIT ||
            state == S1_WAIT_GREEN_SLOT) robot.setRobotVW(0,0);
    
    else if(state == S1_LEAVE_CENTER )
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    #endif
    else if(state == NAV_END_ROUND)
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    

}

void fsm_round23::state_actions_rules()
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
        robot.followLine(v_req_nav, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
    }
    else if(state == SYS_APPROACH_WAREHOUSE)
    {
        robot.setRobotVW(0.05, 0);
    }
    else if(state == M_EXT_PROC_MACH_BLUE)
    {
        robot.followLine_v2(0.1, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
    }

    // ==========================================================
    //             GENERIC NAV to PROCESS BOX
    // ==========================================================
    else if(state == NAV_PROCESS_GREEN_BOX)
    {
        robot.followLine(v_req_leaving_pickZ, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
    }
    
    #ifdef ROBOT_MASTER
    if(state == NAV_PROCESS_GREEN_BOX_ALIGN)
    {
        robot.followLine(v_req_leaving_pickZ, robot.front, Side2Follow::RIGHT, EdgeDetection:: DOWN);
    }
    else if(state == M_SYS_START) robot.setRobotVW(0 ,0); // wait ACK from SLAVE
    else if(state == M_SYS_LEAVE_START)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if(state == EXITING_PICK_ZONE_RED)
    {
        if(currentBox.pick_slot - intersections -1  <= 0)
        {
            robot.setRobotVW(0.08,0);
            robot.front.sensor.getLineError(Side2Follow::LEFT, EdgeDetection:: DOWN);
        } 
        else
        {
            robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN );
        }
    }
    else if(state == M_GEN_DROP_ALIGN)
    {
        robot.followLine(0.13, robot.front, Side2Follow::RIGHT, EdgeDetection:: DOWN);
    }
    else if(state == M_EXT_PROC_MACH_GREEN)
    {
        robot.followLine_v2(v_req_leaving_pickZ, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
        
    }
    else if(state == M_NAV_FROM_MACHINE)
    {
        robot.followLine(v_req_leaving_pickZ, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
    }
    #endif
    




    // ==========================================================
    //                 PICK & DROP BOX/ MANUEVERS 
    // ==========================================================
    
    else if(state == GEN_MOVE_X)
    {
        robot.setRobotVW(move_direction*0.1, -robot.thetae*robot.k_thetae);
    }
    else if(state == GEN_TURN_90)
    {
        robot.setRobotVW(0.0, turn_direction*2);//SEE this w velocity! 
    }
    else if(state == END_ROUND)
    {
        robot.setRobotVW(move_direction*0.1, -robot.thetae*robot.k_thetae); 
    }

    // ==========================================================
    //                 GENERIC PICK BOX
    // ==========================================================
    else if(state == GEN_PICK_COUNT_NAV_FROM_START || state == GEN_PICK_ALIGN || state == EXITING_PICK_ZONE)
    {
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
    }
    else if(state == GEN_PICK_COUNT_FROM_MACHINE)
    {
        robot.followLine_v2(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if(state == GEN_PICK_BOX)
    {
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.08, 1);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.08, -1);
        }
        else{
            robot.setRobotVW(0.08, 0);
        }
    }
    
    // ==========================================================
    //                 GENERIC DROP BOX
    // ==========================================================
    else if(state == GEN_DROP_COUNT || state == EXITING_DROP_ZONE)
    {
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
    }
    else if(state == GEN_DROP_ALIGN )
    {
        if(currentBox.drop_slot == 0) robot.setRobotVW(0.08,0);
        else robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
    }

    // ==========================================================
    //                       GENERIC NAV_TO_WEARHOUSE
    // ==========================================================

    else if(state == NAV_LEAVING_WEARHOUSE || state == NAV_LEAVING_CENTER)
    {
        robot.followLine(v_req_leaving_pickZ, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if(state == NAV_TO_WEARHOUSE)
    {
        if(intersections == 2) robot.followLine(0.16, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
        else robot.followLine(v_req_leaving_pickZ, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }


    // ==========================================================
    //                       SLAVE ACTIONS:
    // ==========================================================
    #ifdef ROBOT_SLAVE_00
    if(state == S_WAIT_BOX_INFO || state == S_WAIT_PICK_CMD || state == S_WAIT_PICK_CMD_2 )
    {
        robot.setRobotVW(0,0);
    }
    else if(state == S_NAV_MACHINE_OUT)//get out at fifth intersection! 
    {
        if(intersections == 0) robot.followLine(0.15, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
        else robot.followLine(0.15, robot.front, Side2Follow::LEFT, EdgeDetection::UP);
    }
    else if(state == S_NAV_TO_MACHINE_FROM_WHOUSE)
    {
        if(this->current_box_index == 0) robot.followLine(0.15, robot.front, Side2Follow::RIGHT, EdgeDetection::UP);
        else
        {
            robot.followLine(0.13, robot.front, Side2Follow::RIGHT, EdgeDetection::UP);
        }
    }
    else if(state == S_MACHINE_ALIGN_PICK)
    {
        robot.followLine(0.18, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
        //robot.setRobotVW(0.08,0);// or follow the line?!
    }
    else if(state == S_MACHINE_PICK_BOX)
    {
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.04, 0.8);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.04, -0.8);
        }
        else{
            robot.setRobotVW(0.08, 0);
        }
    }
    else if(state == S_NAV_MACHINE_TO_DROP)
    {
        robot.followLine(0.16, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if( state == S_NAV_EXIT_DROP_ZONE_2_MACHINE)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    
    
    
    #endif

    #ifdef ROBOT_SLAVE_01
    if(state == S_WAIT_BOX_INFO || state == S_WAIT_PICK_CMD)
    {
        robot.setRobotVW(0,0);
    }

    //---------SLAVE 01 to PICK FIRST BOX ------------------
    else if(state == S1_LEAVE_START)
    {
        robot.followLine(v_req_nav, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    if(state == NAV_PROCESS_GREEN_BOX_ALIGN)
    {
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection:: DOWN);
    }
    else if( state == S1_DROP_B2)
    {
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
    }
    else if(state == S1_NAV_DROP_B)
    {
        robot.followLine(v_req_leaving_pickZ, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if(state == S1_ALIGN_DROP_B)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if(state == S1_LEAVE_CENTER)
    {
        robot.followLine_v2(v_req_leaving_pickZ,robot.front,Side2Follow::LEFT,EdgeDetection::UP);
    }
    #endif
    else if(state == NAV_END_ROUND)
    {
        robot.followLine(this->v_req_nav, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }

}

#ifdef ROBOT_MASTER
void fsm_round23:: build_sequence_from_IR(String ir_data) 
{
    // RESET às variáveis para garantir que está limpo
    this->total_greens = 0;
    this->total_blues = 0;
    int box_index = 0;
    // u o o u

    for(int i = 0; i < ir_data.length(); i++)
    {
        if(box_index > 3) return; 
        if(ir_data[i] == 'u')  this->green_pick_slots[this->total_greens++] = box_index++;
        else if(ir_data[i] == 'o')  this->blue_pick_slots[this->total_blues++] = box_index++;
    }
}

void fsm_round23::build_slaveSlots()
{
    if(this->total_blues == 0) 
    {
        this->MASTER_blueBox = 0;
        this->SLAVE_blueBox = 0;
        return;
    }
    //GREEN BOX:
    //We assume that there is at least one RED BOX! - 3rd round!  
    if(this->total_greens > 0)
    {
        this->SLAVE_greenBox = 1;
        this->MASTER_greenBox = total_greens - SLAVE_greenBox;
        
        int indx = 0;
        for(int i = 0; i < MASTER_greenBox; i++)
        {
            this->M_green_PICK[i] = green_pick_slots[indx++];
        }
        S_green_PICK[0] = green_pick_slots[indx];//last slot! 
    }
    
    //Blue BOX:
    if(this->total_blues % 2 == 0)
    {
        this->MASTER_blueBox =  this->total_blues / 2;
        this->SLAVE_blueBox  = this->MASTER_blueBox;
    } 
    else
    {
        this->SLAVE_blueBox = (this->total_blues +1) / 2;
        this->MASTER_blueBox = this->total_blues - this->SLAVE_blueBox;
        if(this->MASTER_blueBox <= 0) this->MASTER_blueBox = 0;
    } 
    
    uint8_t currIndex = 0;
    for (int i = 0; i< this->SLAVE_blueBox; i++)
    {
        this->S_blue_PICK[i] = this->blue_pick_slots[i];//I want SLAVE_01 to start getting form the low slots! 
        currIndex++;
    }
    for(int i = 0; i< this->MASTER_blueBox ; i++)
    {
        this->M_blue_PICK[i] = this->blue_pick_slots[currIndex++]; 
    }   
}
#endif


void fsm_round23::build_currentBox(BoxRound2 &box)//WHAT ABOUT PICK SLOT????
{
   
    #ifdef ROBOT_MASTER
    if(this->MASTER_greenBox > 0 && current_green_index < 4) // when we store a green box we decrement?!
    {
        box.color = 'g';
        box.pick_slot = this->M_green_PICK[this->current_green_index++];
        this->MASTER_greenBox--;
        //what should be the dropSlot?!
    }
    else if(this->MASTER_blueBox > 0 && current_blue_index < 2)
    {
        box.color = 'b';
        box.pick_slot = this->M_blue_PICK[this->current_blue_index++];
        //box.drop_slot = drop_sequence[this->current_box_index]; 
        this->MASTER_blueBox --;
        //this->total_blues--;
    }

    #endif

    #ifdef ROBOT_SLAVE_01
    if(this->SLAVE_greenBox > 0 && this->current_green_index < 2)
    {
        box.color = 'g';
        box.pick_slot= this->S_green_PICK[this->current_green_index++];
        this->SLAVE_greenBox--;
    }
    else if(this->SLAVE_blueBox > 0 && current_blue_index < 2)
    {
        box.color = 'b';
        box.pick_slot = this->S_blue_PICK[this->current_blue_index++];
        this->SLAVE_blueBox--;
    }
    #endif   
    //current_box_index++; Do we add to the next box!????- better to do when a robot drop box!
}