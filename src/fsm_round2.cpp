#include "fsm_round2.h"

fsm_round2::fsm_round2(robot_t& r) : robot(r)
{
    force_state(SYS_IDLE);

    current_box_index = 0;
    drop_sequence[0] = 3;
    drop_sequence[1] = 2;
    drop_sequence[2] = 1;
    drop_sequence[3] = 0;
    for(int i = 0; i<2 ; i++)
    {
        S_blue_PICK[i] = INVALID_SLOT;
        S_blue_PICK[i] = INVALID_SLOT;
    }
    this->SLAVE_blueBox = 0;
    this->MASTER_blueBox = 0;
}

void fsm_round2::next_state_rules()
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
        //if(IR received successfully)
        
        //LET's ADD here the receiving of the IR to translate STATE! 
        #ifdef ROBOT_MASTER
        //_________________________________//
        // INITIAL BOX LOGIC //
        //_________________________________//
        build_sequence_from_IR("Wouou");//WE HAVE THE NUM OF BOXES AT WHAT PLACE! 
        build_blueBoxPick(this->total_greens);
        /*
        
        */
        #endif
        set_next_state(COM_INIT);
    }
    else if(state == COM_INIT)
    {
        #ifdef ROBOT_MASTER
        //Serial.printf("[MASTER in COM_INIT]");
        if(robot.currentComState == ComState::COM_WAIT_SEND) set_next_state(M_SYS_START);
        else if (robot.currentComState == ComState::COM_ERROR)
        {

        };
        #endif 
        #ifdef ROBOT_SLAVE
        if(robot.currentComState == ComState::COM_LISTEN){
            set_next_state(S_WAIT_CMD_START);
        }
        #endif 
    }
   


    // ==========================================================
    //               LÓGICA EXCLUSIVA DO MASTER - caixa Verde
    // ==========================================================
    #ifdef ROBOT_MASTER
    
    if(state == M_SYS_START)
    {
        //SEND SLAVE what to pick! - if COM ok we can send to slave!!!
        if(total_greens > 0) robot.send_command_param(CMD_ID::INFO_GREEN_BOX, this->total_greens);
        else if(total_blues > 0) robot.send_command_param(CMD_ID::INFO_BLUE_BOX, this->total_blues);//should be 4 blues! 
        if(total_greens > 0 )
        {
            if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_GREEN_BOX)) set_next_state(M_SYS_LEAVE_START);
        }
        else if(total_blues > 0)
        {
            if(robot.appLayer.hasReceivedAck(CMD_ID::INFO_BLUE_BOX)) set_next_state(M_SYS_LEAVE_START);   
        }

    }
    else if(state == M_SYS_LEAVE_START && intersections == 3)
    {
        build_currentBox(this->currentBox, NodeId::MASTER);
        set_next_state(GEN_PICK_ZONE);
    }

    // ==========================================================
    //               NAV SEQUENCES:
    // ==========================================================
    else if(state == M_NAV_PROCESS_BOX)
    {
        if(path_strategy == FOLLOW_LINE)
        {
            //follow right 
        }
        else if(path_strategy == FL_AND_TURNS)
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
    }
    else if(state == M_GEN_DROP_ALIGN && tis > 3) 
    {
        robot.send_command(CMD_ID::CMD_EXECUTE_PICK_GREEN);
        set_next_state(M_GEN_DROP_TURN_OUT);
    }
    else if(state == M_GEN_DROP_TURN_OUT)
    {
        current_box_index++;
        total_greens--;
        target_distance = d_retrive_process_box;
        move_direction = -1;
        turn_direction = 1;
        target_turn_angle = PI/2;
        state_after_maneuver = M_GEN_EXITING_PROCESS_MACHINE;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == M_GEN_EXITING_PROCESS_MACHINE)
    {
        if (robot.front.sensor.intersections == 1 )
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = NAV_FROM_MACHINE;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == NAV_FROM_MACHINE )
    {
        if(intersections == 1 && tis == 0.20/robot.v_req ) intersections == 0; // reset if count wrong
        if(intersections == 1)
        {
            isFromMachine = true;
            build_currentBox(this->currentBox,MASTER);//get the next slot!
            set_next_state(GEN_PICK_ZONE);
        }
    }
    #endif


    // ==========================================================
    //               LÓGICA EXCLUSIVA DO SLAVE
    // ==========================================================
    #ifdef ROBOT_SLAVE
    
    // 1. DORMIR À ESPERA DA CAIXA NA MÁQUINA
    if(state == S_WAIT_BOX_INFO)
    {
        if(robot.appLayer.hasNewCommand())
        {
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            if (cmdId == INFO_GREEN_BOX || cmdId == INFO_BLUE_BOX) 
            {
                if (robot.appLayer.getReceivedParamLen() > 0) 
                {
                    const uint8_t* params = robot.appLayer.getReceivedParams();
                    uint8_t total = params[0]; // This is the 1 byte you sent from Master
                    
                    if(cmdId == INFO_GREEN_BOX)
                    {
                        total_greens = total;//This is what we need to perform 
                        //build_blueBoxPick(total_greens,NodeId::SLAVE); only called in the master side! 
                        set_next_state(S_NAV_MACHINE_OUT);//for now we will move the slave to machine OUTPUT!
                    }
                    else 
                    {
                        this->currentBox.color = 'b';
                        total_greens = 0;
                        total_blues = 4;
                        //Set the state to rotate and go pick blue boxes! 
                    }      
                }
            }
        }
        robot.appLayer.clearNewCommand();
    }
    
    else if(state == S_NAV_MACHINE_OUT)
    {
        if(robot.appLayer.hasNewCommand())
        {
            uint8_t cmdId = robot.appLayer.getReceivedCmdId();
            if(cmdId == INFO_BLUE_PICK_SLOT)
            {
                uint8_t len = robot.appLayer.getReceivedParamLen();
                const uint8_t* params = robot.appLayer.getReceivedParams();
                this->num_slave_blue_boxes = len;

                if (len >= 1) this->slave_blue_pick_slots[0] = params[0]; 
                if(len == 2) this->slave_blue_pick_slots[1] = params[1];
            }
        }
        if(intersections == 5)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
            target_turn_angle = PI/2;
            state_after_maneuver = S_NAV_TO_MACHINE_FROM_WHOUSE;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if (state == S_NAV_TO_MACHINE_FROM_WHOUSE && intersections == 2)
    {
        target_distance = d_mv_aft_intersection;
        move_direction = 1;
        turn_direction = 1;
        target_turn_angle = PI/2;
        state_after_maneuver = S_WAIT_PICK_CMD;
        set_next_state(GEN_MOVE_X);
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
    else if(state == S_MACHINE_ALIGN_PICK && ((robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On)) )
    {
        set_next_state(S_MACHINE_PICK_BOX);
    }

    else if(state == S_MACHINE_PICK_BOX && tis > 0.7) set_next_state(S_MACHINE_TURN_OUT);
    
    else if(state == S_MACHINE_TURN_OUT)
    {
        target_distance = d_retrive_process_box;
        move_direction = -1;
        turn_direction = 1;
        target_turn_angle = PI/2;
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
            target_turn_angle = PI/2;
            state_after_maneuver = S_NAV_TO_MACHINE_FROM_WHOUSE;
            set_next_state(GEN_MOVE_X);
        }
    }

    // 2. NAVEGAR PARA O OUTPUT DA MÁQUINA B
    
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
                state_after_maneuver = GEN_PICK_COUNT_START;
                set_next_state(GEN_MOVE_X);
            }
        }
        else
        {
            set_next_state(GEN_PICK_COUNT_MACHINE);    
        }
        
            
    }
    else if(state == GEN_PICK_COUNT_START)
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
    else if(state == GEN_PICK_COUNT_MACHINE)
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
        turn_direction = -1;
        target_turn_angle = PI/2;
        state_after_maneuver = EXITING_PICK_ZONE;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == EXITING_PICK_ZONE)
    {
        if( 3 - robot.front.sensor.intersections == currentBox.pick_slot)
        {
            isFromMachine = false;
            if(currentBox.color == 'g') set_next_state(M_NAV_PROCESS_BOX);
            else if(currentBox.color == 'b') set_next_state(NAV_LEAVING_WEARHOUSE);
        }
    }

    else if(state == GEN_DROP_BOX)
    {

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
        if(currentBox.color == 'b')
        {
            #ifdef ROBOT_MASTER
            MASTER_blueBox--;
            #endif
            #ifdef ROBOT_SLAVE
            num_slave_blue_boxes--;
            #endif
        } 
        else if(currentBox.color == 'g') total_greens--;
        set_next_state(GEN_DROP_TURN_OUT);//when entering Turn Off the Magnet! 
    }
    else if(state == GEN_DROP_TURN_OUT)
    {
        target_distance = d_retrive_from_wearhouse;
        move_direction = -1;
        turn_direction = 1;
        target_turn_angle = (currentBox.drop_slot == 0)? PI: PI/2;
        if(total_greens == 0)// let's go pick a blue box!!!!! if there is one left!
        { 
            turn_direction = -1;
            target_turn_angle = PI/2;
            state_after_maneuver = EXITING_DROP_ZONE;
        }
        else state_after_maneuver = S_NAV_EXIT_DROP_ZONE_2_MACHINE;
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
            #ifdef ROBOT_SLAVE
            if(num_slave_blue_boxes > 0) set_next_state(NAV_LEAVING_WEARHOUSE);
            else set_next_state(NAV_DOCKING_STATION);
            
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
            currentBox.drop_slot = drop_sequence[current_box_index++];
            set_next_state(GEN_DROP_BOX);
        }
    }
    else if(state == NAV_DOCKING_STATION)
    {
        robot.setRobotVW(0,0);
    }
}




void fsm_round2::enter_state_actions_rules()
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
    else if(state == M_SYS_START)
    {
        
    }
    else if(state == M_SYS_LEAVE_START)
    {
        robot.front.sensor.intersections = 0;
        if (this->SLAVE_blueBox == 1) 
        {
            robot.send_command_param(INFO_BLUE_PICK_SLOT, S_blue_PICK[0]);
        }
        else if (this->SLAVE_blueBox == 2) 
        {
            // Call the 2-parameter overload
            robot.send_command_param(INFO_BLUE_PICK_SLOT, S_blue_PICK[0], S_blue_PICK[1]);
        }
    }

    // ==========================================================
    //             GENERIC NAV to PROCESS BOX
    // ==========================================================
    else if(state == M_NAV_PROCESS_BOX || state == M_GEN_EXITING_PROCESS_MACHINE || 
            state == NAV_FROM_MACHINE)
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == M_GEN_DROP_TURN_OUT)
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
    else if(state == GEN_PICK_COUNT_START || state == EXITING_PICK_ZONE || state == GEN_PICK_COUNT_MACHINE)
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

void fsm_round2::state_actions_rules()
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
    else if(state == M_SYS_START) robot.setRobotVW(0 ,0); // wait ACK from SLAVE
    else if(state == M_SYS_LEAVE_START)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    else if(state == M_NAV_PROCESS_BOX || state == M_GEN_DROP_ALIGN)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection:: DOWN);
    }
    else if(state == M_GEN_EXITING_PROCESS_MACHINE)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection:: UP);
        
    }
    else if(state == NAV_FROM_MACHINE)
    {
        robot.followLine(0.08, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
    }
    




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
    else if(state == GEN_PICK_COUNT_START || state == GEN_PICK_ALIGN || state == EXITING_PICK_ZONE)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
    }
    else if(state == GEN_PICK_COUNT_MACHINE)
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
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }


    // ==========================================================
    //                       SLAVE ACTIONS:
    // ==========================================================
    #ifdef ROBOT_SLAVE

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
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
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

void fsm_round2:: build_sequence_from_IR(String ir_data) 
{
    // RESET às variáveis para garantir que está limpo
    this->total_greens = 0;
    this->total_blues = 0;
    int box_index = 0;
    // u o o u

    for(int i = 0; i < ir_data.length(); i++)
    {
        if(box_index > 3) return;         
        if(ir_data[i] == 'u') {
            this->green_pick_slots[total_greens] = box_index;
            this->total_greens++;
            box_index ++;
        }
        else if(ir_data[i] == 'o') {
            this->blue_pick_slots[total_blues] = box_index;
            this->total_blues++;
            box_index++;
        }
    }

}


void fsm_round2::build_currentBox(BoxRound2 &box, NodeId robotID)//WHAT ABOUT PICK SLOT????
{
    if(robotID == NodeId::MASTER)// only master picks greenBoxes! 
    {
        if(this->total_greens > 0 && current_green_index < 4) // when we store a green box we decrement?!
        {
            box.color = 'g';
            box.pick_slot = this->green_pick_slots[this->current_green_index];
            current_green_index++;
            //what should be the dropSlot?!
        }
        else if(this->MASTER_blueBox > 0 && current_blue_index < 2)
        {
            box.color = 'b';
            box.pick_slot = this->S_blue_PICK[this->current_blue_index];
            //box.drop_slot = drop_sequence[this->current_blue_index]; 
            current_blue_index++;
        }
    }
    else if(robotID == NodeId::SLAVE)
    {
        if(this->SLAVE_blueBox > 0 && current_blue_index < 2)
        {
            box.color = 'b';
            box.pick_slot = this->S_blue_PICK[this->current_blue_index];
            current_blue_index++;
        }
    }
}

void fsm_round2::build_blueBoxPick(uint8_t num_green_box)
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



void fsm_round2::send_sync_to_slave() {


}

bool fsm_round2::check_sync_from_master() {
    return false;
}

