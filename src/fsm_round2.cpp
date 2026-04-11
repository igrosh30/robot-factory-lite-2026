#include "fsm_round2.h"

fsm_round2::fsm_round2(robot_t& r) : robot(r)
{
    force_state(SYS_IDLE);   
    drop_sequence[0] = 0;
    drop_sequence[1] = 1;
    drop_sequence[2] = 2;
    drop_sequence[3] = 3;
}

void fsm_round2::next_state_rules()
{
    //auto& intersections = robot.front.sensor.intersections;
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
        
        #ifdef ROBOT_MASTER
            set_next_state(M_SYS_START); // Master espera pelo sinal de START
        #endif
        #ifdef ROBOT_SLAVE
            set_next_state(S_SYS_START); // Slave vai logo dormir à espera do Master
        #endif
    }
   


    // ==========================================================
    //               LÓGICA EXCLUSIVA DO MASTER - caixa Verde
    // ==========================================================
    #ifdef ROBOT_MASTER
    
    if(state == M_SYS_START && tis > 2)//change to while received a correct IR! 
    {
        
        build_sequence_from_IR("Wouou");
        set_next_state(M_SYS_LEAVE_START); 
    }
    else if(state == M_SYS_LEAVE_START && robot.front.sensor.intersections == 3)
    {
        build_currentBox(this->currentBox);
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
            if(robot.front.sensor.intersections == 2 ) //Let's do edge:UP! 
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
        set_next_state(M_GEN_DROP_TURN_OUT);
    }
    else if(state == M_GEN_DROP_TURN_OUT)
    {
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
    else if(state == NAV_FROM_MACHINE && robot.front.sensor.intersections == 1)
    {
        isFromMachine = true;
        build_currentBox(this->currentBox);//get the next slot!
        set_next_state(GEN_PICK_ZONE);

    }

   
    #endif


    // ==========================================================
    //               LÓGICA EXCLUSIVA DO SLAVE
    // ==========================================================
    #ifdef ROBOT_SLAVE
    
    // 1. DORMIR À ESPERA DA CAIXA NA MÁQUINA
    if(state == S_SYS_LEAVE_START)
    {
        robot.setRobotVW(0.0, 0.0); 
        
        if(check_sync_from_master()) //first receive the frme then go  
        {
            
            
        }
    }

    // 2. NAVEGAR PARA O OUTPUT DA MÁQUINA B
    /*
    else if(state == )
    {
        // Lógica de aproximação à saída da máquina (ajustar ao mapa)
        if(robot.front.sensor.intersections == 1) {
            set_next_state(GEN_PICK_ALIGN); // Usa o teu pick genérico!
        }
    }

    // 3. APANHOU A CAIXA -> VAI PARA O ARMAZÉM DE SAÍDA
    else if(state == EXITING_PICK_ZONE && has_box)
    {
        drop_slot = sequence[completed_green_boxes].drop_slot;
        set_next_state(NAV_TO_WEARHOUSE_DROP); // Navegação genérica
    }

    // 4. LARGOU NO ARMAZÉM -> VOLTAR AO HOLDING NODE
    else if(state == EXITING_DROP_ZONE && !has_box)
    {
        completed_green_boxes++;
        set_next_state(SLAVE_RETURN_TO_HOLDING);
    }

    // 5. REGRESSA E ENTRA EM WAIT
    else if(state == SLAVE_RETURN_TO_HOLDING)
    {
        // Navega de volta ao cruzamento seguro
        if(robot.front.sensor.intersections == 2)
        { 
            if(completed_green_boxes < total_green_boxes)
            {
                set_next_state(S_WAITING_FOR_SYNC);
            } else
            {
                set_next_state(SYS_FINISH); // Acabou o trabalho das verdes!
            }
        }
    
    }*/
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
        if(angle_moved >= target_turn_angle){
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
        if(3 - currentBox.pick_slot == robot.front.sensor.intersections)
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
    else if(state == GEN_PICK_BOX && tis > 1)
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
        if(drop_slot == 0)
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
        if(currentBox.color == 'b') total_blues --;
        set_next_state(GEN_DROP_TURN_OUT);//when entering Turn Off the Magnet! 
    }
    else if(state == GEN_DROP_TURN_OUT)
    {
        target_distance = d_retrive_from_wearhouse;
        move_direction = -1;
        turn_direction = -1;
        target_turn_angle = PI/2;
        state_after_maneuver = EXITING_DROP_ZONE;
        set_next_state(GEN_MOVE_X);
    }
    else if(state == EXITING_DROP_ZONE)
    {
        if( 3 - robot.front.sensor.intersections == currentBox.drop_slot)
        {
        }
    }

}




void fsm_round2::enter_state_actions_rules()
{
    // ==========================================================
    //                    SYSTEM & STARTUP
    // ==========================================================
    if(state == SYS_IDLE)
    {
        robot.setRobotVW(0,0);
        robot.front.actuators.magnetOff();
    }
    else if(state == M_SYS_START)
    {
        //robot.currentComState = COM_START;
        robot.setRobotVW(0,0);
        robot.thetae = PI*0.5;
        //Check this!
        robot.xe = -69.5;
        robot.ye = -35.5;
        robot.dropBox(robot.front);
        robot.front.sensor.intersections = 0;
        robot.front.sensor.readValues();
    }
    else if(state == M_SYS_LEAVE_START)
    {
        robot.front.sensor.intersections = 0;
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
    else if(state == NAV_FROM_MACHINE) robot.setRobotVW(0,0);
    else if(state == NAV_TO_WEARHOUSE || state == NAV_LEAVING_WEARHOUSE)
    {
        robot.setRobotVW(0.0, 0.0);
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
    else if(state == M_SYS_START)
    {
        if(total_greens > 0 ) robot.send_command(CMD_ID::CMD_GO_PROCESS_MACHINE);
        else robot.send_command(CMD_ID::CMD_EXECUTE_PICK_BLUE);
    }
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
        if(intersections == 0 ) robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection:: UP);
        else if(intersections >= 1) robot.followLine(0.1, robot.front, Side2Follow::RIGHT, EdgeDetection:: UP);
    }
    else if(state == NAV_FROM_MACHINE)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection:: DOWN);
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
        robot.followLine(0.08, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
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


    


}

void fsm_round2:: build_sequence_from_IR(String ir_data) 
{
    // RESET às variáveis para garantir que está limpo
    total_greens = 0;
    total_blues = 0;
    int box_index = 0;
    // u o o u

    for(int i = 0; i < ir_data.length(); i++)
    {
        if(box_index > 3) return;         
        if(ir_data[i] == 'u') {
            green_pick_slots[total_greens] = box_index;
            total_greens++;
            box_index ++;
        }
        else if(ir_data[i] == 'o') {
            blue_pick_slots[total_blues] = box_index;
            total_blues++;
            box_index++;
        }
    }

}


void fsm_round2::build_currentBox(BoxRound2 &box)
{
    if(total_greens > 0 && current_green_index < 4) // when we store a green box we decrement?!
    {
        box.color = 'g';
        box.pick_slot = this->green_pick_slots[this->current_green_index];
        current_green_index++;
        //what should be the dropSlot?!
    }
    else if(total_blues > 0 && current_blue_index < 4)
    {
        box.color = 'b';
        box.pick_slot = this->blue_pick_slots[this->current_blue_index];
        current_blue_index++;
    }
}


// Mais tarde tens de ligar isto à UART1 ou UART2 (Serial1/Serial2)
void fsm_round2::send_sync_to_slave() {
    // Exemplo: Serial1.print("RDY\n");
    // (A implementar quando tratares do hardware IR)
}

bool fsm_round2::check_sync_from_master() {
    // Exemplo: Se pressionares um botão no Slave para simular a chegada da SMS
    // return robot.front.actuators.isSwitch_right_On; 
    
    // (A implementar quando tratares da leitura UART)
    return false; // Retorna true quando ler a frame certa
}

