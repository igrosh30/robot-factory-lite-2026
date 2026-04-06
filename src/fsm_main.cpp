#include "fsm_main.h"
#include "config.h"

fsm_main::fsm_main(robot_t& r) : robot(r)
{
    // You can set initial state here if you want
    force_state(SYS_IDLE);
    current_box_index = 0;
    #ifdef ROBOT_MASTER
        sequence[0].pick_slot = 3;
        sequence[0].drop_slot = 3;
        sequence[1].pick_slot = 2;
        sequence[1].drop_slot = 2;
    #endif
    #ifdef ROBOT_SLAVE
        sequence[0].pick_slot = 0;
        sequence[0].drop_slot = 0;
        sequence[1].pick_slot = 1;
        sequence[1].drop_slot = 1;
    #endif
}

void fsm_main::next_state_rules()
{
    // ==========================================================
    //                    SYSTEM & STARTUP
    // ==========================================================
    if(state == SYS_IDLE && robot.front.actuators.isSwitch_left_On)
    {
        set_next_state(SYS_CALIBRATION);
    }
    else if(state == SYS_CALIBRATION && (robot.thetae > 6.2 || !CALIBRATION_MODE ))//CHECK IF ITS A MINUS
    {
        for(int i = 0; i<5; i++)
        {
            robot.front.sensor.minValues[i] = robot.front.sensor.minValues[i] - 5;
            robot.front.sensor.maxValues[i] = robot.front.sensor.maxValues[i] + 10;
        }
        set_next_state(SYS_START);
    }
    else if(state == SYS_START && tis >1)
    {
        set_next_state(SYS_LEAVE_START);
    }
    else if(state==SYS_LEAVE_START && robot.front.sensor.intersections == 3)
    {
        pick_slot = sequence[current_box_index].pick_slot;
        set_next_state(GEN_PICK_ZONE);
    }
    

    // ==========================================================
    //                 GENERIC MANEUVER MEMORY
    // ==========================================================
    /*
    // LOGIC TO GET BACK: Catch all backwards movement to trigger generic turn
    else if(state == B1_PICK_BACKUP || state == B2_PICK_BACKUP ||
            state == B3_PICK_BACKUP || state == B4_PICK_BACKUP ||
            state == B1_DROP_BACKUP || state == B2_DROP_BACKUP ||
            state == B3_DROP_BACKUP || state == B4_DROP_BACKUP)
    {
        float distance_moved = abs(ref_s - robot.rel_s);
        if(distance_moved > 0.1)
        {
            if(state == B1_PICK_BACKUP) state_after_maneuver = B1_NAV_TO_DROP;
            else if(state == B2_PICK_BACKUP) state_after_maneuver = B2_NAV_TO_DROP;
            else if(state == B3_PICK_BACKUP) state_after_maneuver = B3_NAV_TO_DROP;
            else if(state == B4_PICK_BACKUP) state_after_maneuver = B4_NAV_TO_DROP;
            else if(state == B1_DROP_BACKUP) state_after_maneuver = B1_LEAVE_DROPZONE;
            else if(state == B2_DROP_BACKUP) state_after_maneuver = B2_LEAVE_DROPZONE;
            else if(state == B3_DROP_BACKUP) state_after_maneuver = B3_LEAVE_DROPZONE;
            else if(state == B4_DROP_BACKUP) state_after_maneuver = B4_LEAVE_DROPZONE;
            
            turn_direction = -1;
            set_next_state(GEN_TURN_90);
        }
    }*/ 
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
    //                       GENERIC PICK BOX
    // ==========================================================
    else if(state == GEN_PICK_ZONE )
    {
        if(pick_slot == 0)
        {
            set_next_state(GEN_PICK_ALIGN);
        }
        else
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = -1;
            target_turn_angle = PI/2;
            state_after_maneuver = GEN_PICK_COUNT;
            set_next_state(GEN_MOVE_X);
        }
    }
    else if(state == GEN_PICK_COUNT)
    {
        if(pick_slot == robot.front.sensor.intersections)
        {
            target_distance = d_mv_aft_intersection;
            move_direction = 1;
            turn_direction = 1;
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
        if( 3 - robot.front.sensor.intersections == pick_slot)
        {
            set_next_state(NAV_LEAVING_WEARHOUSE);
        }
    }


    // ==========================================================
    //                       GENERIC DROP BOX
    // ==========================================================

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
        if(drop_slot == robot.front.sensor.intersections)
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
        if( 3 - robot.front.sensor.intersections == drop_slot)
        {
            if(++current_box_index < 2)
            {
                // Ainda há caixas para apanhar! Prepara a próxima viagem e volta para a linha.
                //pick_slot = sequence[current_box_index].pick_slot; DO ONLY ONCE: state = NAV_TO_WEARHOUSE_PICK
                //drop_slot = sequence[current_box_index].drop_slot;
                set_next_state(NAV_LEAVING_WEARHOUSE);
            }
            else
            {
                set_next_state(SYS_IDLE);
            }

        }
    }

    // ==========================================================
    //                       GENERIC NAV_TO_WEARHOUSE
    // ==========================================================
    else if(state == NAV_TO_WEARHOUSE && robot.front.sensor.intersections == 3)//state == NAV_TO_WEARHOUSE_PICK, ver ==3! 
    {
        if(!robot.front.actuators.isMagnetOn)
        {
            pick_slot = sequence[current_box_index].pick_slot;
            set_next_state(GEN_PICK_ZONE);
        }
        else
        {
            drop_slot = sequence[current_box_index].drop_slot;
            set_next_state(GEN_DROP_BOX);
        }
        
    }
    else if(state == NAV_LEAVING_WEARHOUSE )
    {
        //Go x front then turn!
        if(path_strategy == TURN_AFTER_DETECTION)
        {
            if(robot.front.sensor.intersections == 2)   //detetado com UP
            {
                state_after_maneuver = NAV_TO_WEARHOUSE;
                turn_direction = -1;
                target_turn_angle = PI/4; //45
                set_next_state(GEN_TURN_90);
            }
        }
        else if(path_strategy == DISTANCE_TURN)
        {
            if(robot.front.sensor.intersections == 2)
            {//I can do to count the fith intersection in UP direction or not!                  
                
                //Move forward 4.0cm!
                target_distance = 0.040f;     
                move_direction = 1;

                target_turn_angle = PI/2;
                turn_direction = -1;
                state_after_maneuver = NAV_TO_WEARHOUSE;//estado final DEPOIS do turn
                set_next_state(GEN_MOVE_X);//after this we always go GEN_TURN_90! 
            }
        }
        else if(path_strategy == THETA_TURN)
        {
            if(robot.thetae < -1.4) set_next_state(NAV_TO_WEARHOUSE);            
        }
    }
    /*
    else if(state == NAV_LEAVING_WEARHOUSE_D && robot.front.sensor.intersections == 2)//I JUST NEED TO SEE PICK/DROP 
    {
        //Go x front then turn!
        if(path_strategy == TURN_AFTER_DETECTION)
        {
            if(robot.front.sensor.intersections == 5)   //detetado com UP
            {
                state_after_maneuver = NAV_TO_WEARHOUSE_DROP;
                turn_direction = -1;
                target_turn_angle = PI/4; //45
                set_next_state(GEN_TURN_90);
            }
        }
        else if(path_strategy == DISTANCE_TURN)
        {
            if(robot.front.sensor.intersections == 5)
            {//I can do to count the fith intersection in UP direction or not!                  
                
                //Move forward 4.0cm!
                target_distance = 0.040f;     
                move_direction = 1;

                target_turn_angle = PI/2;
                turn_direction = -1;
                state_after_maneuver = NAV_TO_WEARHOUSE_DROP;//estado final DEPOIS do turn
                set_next_state(GEN_MOVE_X);//after this we always go GEN_TURN_90! 
            }
        }
        else if(path_strategy == THETA_TURN)
        {
            if(robot.thetae < -1.4) set_next_state(NAV_TO_WEARHOUSE_DROP);            
        }
    }*/
    else if(state == NAV_TO_WEARHOUSE_DROP)
    {
        drop_slot = sequence[current_box_index].drop_slot;
        set_next_state(GEN_DROP_BOX);
    }

    // ==========================================================
    //                       BOX 1 SEQUENCE
    // ==========================================================
    else if(state == B1_PICK && tis > 1) 
    {
        set_next_state(B1_PICK_BACKUP);
    }
    else if(state == B1_NAV_TO_DROP)
    {
        
    }
    else if (state == B1_ALIGN_DROP && robot.front.sensor.intersections == 4) //STATE: 123 to test DropBox!
    {
        //drop_slot = 1;// Set the droping Slot! 
        //set_next_state(B1_APPROACH_DROP);
        set_next_state(GEN_DROP_BOX);
    }
    else if (state == B1_APPROACH_DROP && tis > 2)
    {
        set_next_state(B1_DROP_BACKUP);
    }
    else if(state == B1_LEAVE_DROPZONE)
    {
        if(path_strategy == TURN_AFTER_DETECTION)
        {
            if(robot.front.sensor.intersections == 5)   // detetado com UP
            {
                state_after_maneuver = B1_NAV_NEXT_LAP;
                turn_direction = -1;
                target_turn_angle = PI/3; //60º
                set_next_state(GEN_TURN_90);
            }
        }
        else if(path_strategy == DISTANCE_TURN)
        {
            if(robot.front.sensor.intersections == 5)
            {//I can do to count the fith intersection in UP direction or not!                  
                
                //Move forward 4.0cm!
                target_distance = 0.040f;     
                move_direction = 1;

                target_turn_angle = PI/2;
                turn_direction = -1;
                state_after_maneuver = B1_NAV_NEXT_LAP;//estado final DEPOIS do turn
                set_next_state(GEN_MOVE_X);//after this we always go GEN_TURN_90! 
            }
        }
        else if(path_strategy == THETA_TURN)
        {
            if(robot.thetae < -1.4) set_next_state(B1_NAV_NEXT_LAP);            
        }
    }

    // ==========================================================
    //                       BOX 3 SEQUENCE
    // ==========================================================
    else if(state == B2_NAV_NEXT_LAP && (robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On))
    {
        set_next_state(B3_PICK);
    }
    else if(state == B3_PICK && tis>1 )
    {
        set_next_state(B3_PICK_BACKUP);
    }
}

void fsm_main::enter_state_actions_rules()
{
    // ==========================================================
    //                    SYSTEM & STARTUP
    // ==========================================================
    if(state == SYS_IDLE)
    {
        robot.setRobotVW(0,0);
        robot.front.actuators.magnetOff();
    }
    else if(state == SYS_START)
    {
        robot.setRobotVW(0,0);
        robot.thetae = PI*0.5;
        //Check this!
        robot.xe = -69.5;
        robot.ye = -35.5;
        robot.dropBox(robot.front);
        robot.front.sensor.intersections = 0;
        robot.front.sensor.readValues();
    }
    else if(state == SYS_LEAVE_START)
    {
        robot.front.sensor.intersections = 0;
    }

    // ==========================================================
    //                 PICK & DROP BOX
    // ==========================================================
    else if(state == B1_PICK || state == B2_PICK || state == B3_PICK)
    {
        robot.thetae = PI*0.5;//maybe when we leave!? 
        robot.front.actuators.magnetOn();//Only need to tell the hardware once to turn on the magnet ON!        
    }
    else if(state == B1_PICK_BACKUP || state == B2_PICK_BACKUP || 
            state == B3_PICK_BACKUP || state == B4_PICK_BACKUP )
    {
        ref_s = robot.rel_s;
    }
    else if(state == B1_DROP_BACKUP || state == B2_DROP_BACKUP ||
            state == B3_DROP_BACKUP || state == B4_DROP_BACKUP )
    {
        ref_s = robot.rel_s;
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
    else if(state == GEN_PICK_COUNT || state == EXITING_PICK_ZONE)
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
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    /*
    else if(state == B1_TURN_ON_NAV_TO_DROP  || state ==  B1_LEAVE_DROPZONE_TURN)
    {
        ref_s = robot.rel_s;
    }*/

    // ==========================================================
    //                   NAV SEQUENCES
    // ==========================================================
    else if(state == B2_NAV_PICK || state == B3_NAV_PICK || state == B4_NAV_PICK)
    {
        robot.front.sensor.intersections = 0;
    }
    else if(state == B1_NAV_TO_DROP  || state == B1_ALIGN_DROP || 
        state == B1_APPROACH_DROP    || state == B1_LEAVE_DROPZONE )
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    }
    else if(state == B2_NAV_TO_DROP ||  state == B2_ALIGN_DROP ) 
    {
        robot.front.sensor.intersections = 0;
        robot.front.sensor.wasIntersection = false;
    }
    else if(state == B1_NAV_NEXT_LAP)
    {
        robot.front.sensor.wasIntersection = false;
        robot.front.sensor.intersections = 0;
    } 
    else if(state == B2_LEAVE_DROPZONE || state == B2_NAV_NEXT_LAP || state == B3_PICK || state == B3_NAV_TO_DROP )
    {
        robot.front.sensor.intersections = 0;
    }  
}

void fsm_main::state_actions_rules()
{
    // High-speed reference alias
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
    //                 PICK & DROP BOX/ MANUEVERS 
    // ==========================================================
    
    //-------WE ARE FACING THE WEAREHOUSE - JUST MOVE FORWARD! 
    else if(state == B2_ALIGN_PICK || state == B3_ALIGN_PICK || state == B4_ALIGN_PICK )
    {
        robot.setRobotVW(0.08,0);
    }
    // --- PICK THE BOX! 
    else if(state == B1_PICK || state == B2_PICK || state == B3_PICK)
    {
        // Ensure it picks the box! 
        
    }
    //START leaving the Whearhouse 
    else if(state == B1_PICK_BACKUP || state == B2_PICK_BACKUP||
            state == B3_PICK_BACKUP || state == B4_PICK_BACKUP||
            state == B1_DROP_BACKUP || state == B2_DROP_BACKUP||
            state == B3_DROP_BACKUP || state == B4_DROP_BACKUP)
    {
        robot.setRobotVW(-0.08,0);
    }
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
    else if(state == GEN_PICK_COUNT || state == GEN_PICK_ALIGN || state == EXITING_PICK_ZONE)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
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

    else if(state == NAV_TO_WEARHOUSE_PICK || state == NAV_LEAVING_WEARHOUSE)
    {
        robot.followLine(0.1, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
    }
    
    // ==========================================================
    //                       BOX 1 SEQUENCE
    // ==========================================================
    else if(state == B1_NAV_TO_DROP)
    {
        if(path_strategy == TURN_AFTER_DETECTION ||DISTANCE_TURN )
        {
            if(intersections < 3) robot.followLine(0.10, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
            else robot.followLine(0.10, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
        }
        else if(path_strategy == THETA_TURN)
        {
            if(intersections < 3) robot.followLine(0.10, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
            else if(intersections == 3 ) robot.followLine(0.10, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
            else if(intersections > 3) robot.followLine(0.10, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);

        }
    }
    else if(state == B1_ALIGN_DROP)
    {   
        robot.followLine(0.08, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
    }
    else if(state == B1_APPROACH_DROP)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT,EdgeDetection:: DOWN);
    }
    else if(state == B1_LEAVE_DROPZONE)
    {
        if(intersections < 3)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT,EdgeDetection:: DOWN);
        }
        else if(intersections >= 3)
        {
            robot.followLine(0.08, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
        }
    }

    // ==========================================================
    //                       BOX 2 SEQUENCE
    // ==========================================================
    else if(state == B1_NAV_NEXT_LAP )
    {
        if(intersections == 1){
            robot.thetae = PI * 0.5; // reset the theta! Use M_PI for standard math library compatibility
        }
        if(intersections <= 3) robot.followLine(0.08, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);  
    }
    else if(state == B2_NAV_PICK) 
    {
        robot.followLine(0.08, robot.front, Side2Follow::LEFT ,EdgeDetection:: DOWN);
    }
    else if(state == B2_NAV_TO_DROP)
    {
        if(path_strategy == TURN_AFTER_DETECTION ||DISTANCE_TURN )
        {
            if(intersections < 2) robot.followLine(0.10, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
            else robot.followLine(0.10, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
        }
        else if(path_strategy == THETA_TURN)
        {
            if(intersections < 3) robot.followLine(0.10, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);
            else if(intersections == 3 ) robot.followLine(0.10, robot.front, Side2Follow::LEFT, EdgeDetection::DOWN);
            else if(intersections > 3) robot.followLine(0.08, robot.front, Side2Follow::RIGHT, EdgeDetection::DOWN);

        }
    }
    else if(state == B2_ALIGN_DROP)//16
    {
        if(intersections <= 2) robot.followLine(0.08, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
        else if(intersections == 3)//passar 1: Esq -1: direita
        {
            
        } 
        else if(intersections >= 3) robot.followLine(0.04, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
    }
    else if(state == B2_DROP)
    {
        robot.setRobotVW(-0.08, 0);
        
    }
    else if(state == B2_LEAVE_DROPZONE)
    {
        if(intersections < 3)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT,EdgeDetection:: DOWN);
        }
        else if(intersections == 3)
        {
            robot.followLine(0.08, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
        }
        else if(intersections >= 4)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT,EdgeDetection:: DOWN);
        }
    }


}

void control(robot_t& robot)
{
    state_machines.step();
}