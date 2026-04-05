#include "fsm_main.h"
#include "config.h"

fsm_main::fsm_main(robot_t& r) : robot(r)
{
    // You can set initial state here if you want
    force_state(SYS_IDLE);
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
        set_next_state(SYS_APPROACH_WAREHOUSE);
    }
    else if(state== SYS_APPROACH_WAREHOUSE && (robot.front.actuators.isSwitch_left_On || robot.front.actuators.isSwitch_right_On) )
    {
        set_next_state(B1_PICK);
    }

    // ==========================================================
    //                 GENERIC MANEUVER MEMORY
    // ==========================================================
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
    } 
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
    //                       BOX 1 SEQUENCE
    // ==========================================================
    else if(state == B1_PICK && tis > 1) 
    {
        set_next_state(B1_PICK_BACKUP);
    }
    else if(state == B1_NAV_TO_DROP)
    {
        if(path_strategy == TURN_AFTER_DETECTION)
        {
            if(robot.front.sensor.intersections == 5)   // detetado com UP
            {
                state_after_maneuver = B1_ALIGN_DROP;
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
                state_after_maneuver = B1_ALIGN_DROP;//estado final DEPOIS do turn
                set_next_state(GEN_MOVE_X);//after this we always go GEN_TURN_90! 
            }
        }
        else if(path_strategy == THETA_TURN)
        {
            if(robot.thetae < -1.4) set_next_state(B1_ALIGN_DROP);            
        }
    }
    else if (state == B1_ALIGN_DROP && robot.front.sensor.intersections == 3)
    {
        set_next_state(B1_APPROACH_DROP);
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
    //                       BOX 2 SEQUENCE
    // ==========================================================
    else if(state == B1_NAV_NEXT_LAP && robot.front.sensor.intersections == 3)
    {
        target_distance = 0.04;
        move_direction = 1;

        turn_direction = -1;
        target_turn_angle = PI/2;

        state_after_maneuver = B2_NAV_PICK;
        set_next_state(GEN_MOVE_X);
        
    }
    else if(state == B2_NAV_PICK && ((robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On)))//why cant I just follow the line and then when the sensor activate I can pick the box
    {
        set_next_state(B2_PICK);
    }

    //else if(state == B2_ALIGN_PICK && ((robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On)))
    //{
        //set_next_state(B2_PICK);
    //}
    else if(state == B2_PICK && tis > 1) 
    {
        set_next_state(B2_PICK_BACKUP);
    }
    else if (state == B2_NAV_TO_DROP)
    {
        if(path_strategy == TURN_AFTER_DETECTION)
        {
            if(robot.front.sensor.intersections == 4)   // detetado com UP
            {
                state_after_maneuver = B2_ALIGN_DROP;
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
                state_after_maneuver = B2_ALIGN_DROP;//estado final DEPOIS do turn
                set_next_state(GEN_MOVE_X);//after this we always go GEN_TURN_90! 
            }
        }
        else if(path_strategy == THETA_TURN)
        {
            if(robot.thetae < -1.4) set_next_state(B2_ALIGN_DROP);            
        }   
    }
    else if(state == B2_ALIGN_DROP)
    {
        if(robot.front.sensor.intersections == 3 )
        {
            target_turn_angle = PI/2;
            turn_direction = -1;
            state_after_maneuver = B2_ALIGN_DROP1;//estado final DEPOIS do turn
            set_next_state(GEN_TURN_90);//after this we always go GEN_TURN_90! 
        }
    }
    else if(state == B2_ALIGN_DROP1)
    {
        if(robot.front.sensor.intersections == 1)
        {
            target_distance = 0.040f;     
            move_direction = 1;

            target_turn_angle = PI/2;
            turn_direction = -1;
            state_after_maneuver = B2_APPROACH_DROP;//estado final DEPOIS do turn
            set_next_state(GEN_MOVE_X);//after this we always go GEN_TURN_90! 
        }
    }
    else if(state == B2_APPROACH_DROP )
    {
        set_next_state(B2_LEAVE_DROPZONE);
    }
    else if(state == B2_LEAVE_DROPZONE && robot.thetae < -4.4)
    {
        set_next_state(B2_NAV_NEXT_LAP);
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
    //                 MAKING TURNS
    // ==========================================================

    else if(state == B1_TURN_ON_NAV_TO_DROP  || state ==  B1_LEAVE_DROPZONE_TURN)
    {
        ref_s = robot.rel_s;
    }

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
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.05, 0.4);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.05, -0.4);
        }
        else{
            robot.setRobotVW(0.05, 0);
        }
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

    else if(state == B1_TURN_ON_NAV_TO_DROP)
    {
        robot.setRobotVW(0.08,0);
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

    // ==========================================================
    //                       BOX 3 SEQUENCE
    // ==========================================================
    else if(state == B2_NAV_NEXT_LAP) // ENTER reset COUNT!
    {
        if(intersections < 2) robot.followLine(0.08, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
        else if(intersections >= 2 && intersections < 4) robot.followLine(0.05, robot.front, Side2Follow::RIGHT,EdgeDetection:: DOWN);
        else if(intersections >= 4) robot.followLine(0.04, robot.front, Side2Follow::LEFT,EdgeDetection:: DOWN);
        
        if(intersections == 1){
            robot.thetae = M_PI * 0.5; // reset the theta! 
        }
    }
    else if(state == B3_NAV_TO_DROP)
    {
        if(intersections == 0) {
            robot.followLine(-0.08, robot.front, Side2Follow::RIGHT,EdgeDetection:: DOWN);
        }
        else if(intersections == 1) {
            // TODO: Add your logic for when intersection count hits 1
        }
    }
}

void control(robot_t& robot)
{
    state_machines.step();
}