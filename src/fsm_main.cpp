#include "fsm_main.h"
#include "config.h"

fsm_main::fsm_main(robot_t& r) : robot(r)
{
    // You can set initial state here if you want
    //force_state(Idle);  
    force_state(Idle);
}

void fsm_main::next_state_rules()
{
    // Step 3: your transitions go here
    if(state == Idle && robot.front.actuators.isSwitch_left_On)
    {
        set_next_state(Calibration);
    }
    else if(state == Calibration && robot.thetae > 6.2 )//CHECK IF ITS A MINUS
    {
        for(int i = 0; i<5; i++)
        {
            robot.front.sensor.minValues[i] = robot.front.sensor.minValues[i] - 5;
            robot.front.sensor.maxValues[i] = robot.front.sensor.maxValues[i] + 10;
            //robot.back.sensor.minValues[i] = robot.back.sensor.minValues[i] - 5;
            //robot.back.sensor.maxValues[i] = robot.back.sensor.maxValues[i] + 10;
        }
        set_next_state(Start);
    }
    else if(state == Start )//&& robot.front.actuators.isSwitchOn
    {
        set_next_state(FL_frontL);
        //set_next_state(FollowLineFrontL);
    }
    else if(state==FL_frontL && robot.front.sensor.intersections == 3)
    {
        set_next_state(Move_F);
    }
    else if(state== Move_F && (robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On) )
    {
        set_next_state(PickBox1);
    }
    else if(state == PickBox1 && tis > 1) 
    {
        set_next_state(PickBox_Back1);
    }
    //LOGIC TO GET BACK
    else if(state == PickBox_Back1 || state == PickBox_Back2||
            state == PickBox_Back3 || state == PickBox_Back4)
    {
        float distance_moved = abs(ref_s - robot.rel_s);
        if(distance_moved > 0.1)
        {
            if(state == PickBox_Back1) state_after_maneuver = Box1GO2DropZone;
            else if(state == PickBox_Back2) state_after_maneuver = B2_DZ;
            else if(state == PickBox_Back3) state_after_maneuver = B3_DZ;
            turn_direction = -1;
            set_next_state(TURN_90);
        }
        
    }
    else if(state == TURN_90)
    {
        float angle_moved = abs(robot.getAngleDiff(ref_theta,robot.rel_theta));
        if(angle_moved >= PI*0.5){
            set_next_state(state_after_maneuver);
            state_after_maneuver = 0;//reset
            turn_direction = 0;//reset
        }
    }
    else if(state == Box1GO2DropZone && robot.thetae < -1.4)//think of better approach!
    {
        set_next_state(Box1GO2DropZone1);//foll Left
    }
    else if (state == Box1GO2DropZone1 && robot.front.sensor.intersections == 3 )
    {
        set_next_state(Box1GO2DropZone2);
    }
    else if (state == Box1GO2DropZone2 && tis > 2)
    {
        set_next_state(DropBox_Back);
    }
    else if(state == DropBox_Back && (robot.front.sensor.intersections==1 ))//DROP&GO back|| tis > 4.5
    {
        set_next_state(B1_LDZ);//Go to START point!
    }
    else if(B1_LDZ && robot.thetae < -4.4)
    {
        set_next_state(LAP_2); 
    }
    else if(state==LAP_2 && (robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On))
    {
        
        set_next_state(PickBox2);
    }
    else if(state == PickBox2 && tis > 1) 
    {
        set_next_state(B2_DZ);
        //set_next_state(PickBox_Back2);
    }/*
    else if(state == PickBox_Back2 && (robot.front.sensor.intersections == 1 ))///|| tis>4.5
    {
        set_next_state(B2_DZ2);
    }*/
    else if(state == B2_DZ && robot.thetae < -1.4)
    {
        set_next_state(B2_DZ2_1);
    }
    else if(state == B2_DZ2_1  && robot.front.sensor.intersections == 3)
    {
        set_next_state(TURN_90);
    }
    /*
    else if(state == B2_DZ2_1 && robot.front.sensor.intersections == 3 ) set_next_state(TURN_90)
    */
    
    else if(state == B2_DROP && (robot.front.sensor.intersections==1 || tis > 4.5) )
    {
        set_next_state(B2_LDZ);
    }
    else if(B2_LDZ && robot.thetae < -4.4)
    {
        set_next_state(LAP_3);
    }
    else if(state == LAP_3 && (robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On))
    {
        set_next_state(PickBox3);
    }
    else if(state == PickBox3 && tis>1 )
    {
        set_next_state(PickBox_Back3);
    }
    else if(state == PickBox_Back3)
    {
        //set_next_state();
    }


}

void fsm_main::enter_state_actions_rules()
{
    if(state == Idle)
    {
        robot.setRobotVW(0,0);
        robot.front.actuators.magnetOff();
        //robot.back.actuators.magnetOff();
    }
    else if(state == Start)
    {
        robot.setRobotVW(0,0);
        robot.thetae = PI*0.5;
        //Check this!
        robot.xe = -69.5;
        robot.ye = -35.5;
        robot.dropBox(robot.front);
        //robot.dropBox(robot.back);
        robot.front.sensor.intersections = 0;
        //robot.back.sensor.countIntersections = 0;
        //robot.back.sensor.readValues();
        robot.front.sensor.readValues();
    }
    else if(state == FL_frontL)
    {
        robot.front.sensor.intersections = 0;
    }
    else if(state == PickBox1 || state == PickBox2 || state == PickBox3)
    {
        robot.thetae = PI*0.5;//maybe when we leave!? 
        robot.front.actuators.magnetOn();//Only need to tell the hardware once to turn on the magnet ON!        
    }
    else if(state == PickBox_Back1 || state == PickBox_Back2 || 
            state == PickBox_Back3 || state == PickBox_Back4 )
    {
        ref_s = robot.rel_s;
    }
    else if(state == TURN_90)
    {
        ref_theta = robot.rel_theta;
    }
    else if(state == Box1GO2DropZone  || state == Box1GO2DropZone1 || state == Box1GO2DropZone2  )
    {
        robot.front.sensor.intersections = 0;
    }
    else if(state == DropBox_Back)
    {
        robot.front.actuators.magnetOff();
        robot.front.sensor.intersections = 0;
    }
    else if(state == B1_LDZ)
    {
        robot.front.sensor.intersections = 0;
    }
    else if(state == LAP_2)
    {
        robot.front.sensor.intersections = 0;
    } 
    else if(state == B2_DZ || state == B2_DZ2_1)
    {
        robot.front.sensor.intersections = 0;
    }
    else if(state == B2_DROP)
    {
        robot.front.sensor.intersections = 0;
        robot.front.actuators.magnetOff();
    }
    else if(state == B2_LDZ || state == LAP_3 || state == PickBox3 || B3_DZ )
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
    
    // Run once when entering a new state
    if(state == Calibration)
    {
        if(CALIBRATION_MODE)
        {
            robot.setRobotVW(0, 0.6);
            //robot.back.sensor.calibrate();
            robot.front.sensor.calibrate();  
        }
        else{
            robot.front.sensor.setCalibration(HARDCODED_FRONT_MIN,HARDCODED_FRONT_MAX);
            //robot.back.sensor.setCalibration(HARDCODED_BACK_MIN,HARDCODED_BACK_MAX);
        }
    }
    else if(state == FL_frontL)
    {
        v_nom = 0.08;
        robot.followLine(v_nom, robot.front, Side2Follow::LEFT);
    }
    else if(state == Move_F)
    {
        robot.setRobotVW(0.05, 0);
    }
    else if(state == PickBox1 || state == PickBox2 || state == PickBox3)
    {
        // Ensure it picks the box! 
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.03, 0.2);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.03, -0.2);
        }
        else{
            robot.setRobotVW(0.03, 0);
        }
    }
    else if(state == PickBox_Back1 || state == PickBox_Back2 ||
            state == PickBox_Back3 || state == PickBox_Back4)
    {
        robot.setRobotVW(-0.05,0);
        /*
        if(state == PickBox_Back2) robot.setRobotVW(-0.04, 0);
        else robot.setRobotVW(-0.08, 0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);*/
    }
    else if(state == TURN_90)
    {
        robot.setRobotVW(0.0, turn_direction*0.6);
    }
    else if(state == Box1GO2DropZone)
    {
        if(intersections < 4)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
        else if(intersections == 4)
        {
            robot.followLine(0.08, robot.front, Side2Follow::LEFT);
        }
        else if(intersections == 5)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
    }
    else if(state == Box1GO2DropZone1)
    {
        robot.followLine(0.08, robot.front, Side2Follow::LEFT);
    }
    else if(state == Box1GO2DropZone2)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
    }
    else if (state == DropBox_Back)
    {
        robot.setRobotVW(-0.08, 0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);
    }
    else if(state == B1_LDZ)
    {
        if(intersections < 4)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
        else if(intersections == 4)
        {
            robot.followLine(0.08, robot.front, Side2Follow::LEFT);
        }
        else if(intersections == 5)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
    }
    else if(state == LAP_2)
    {
        if(intersections < 2) robot.followLine(0.08, robot.front, Side2Follow::LEFT);
        else if(intersections == 2) robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        else if(intersections >= 3) robot.followLine(0.04, robot.front, Side2Follow::LEFT);
        
        if(intersections == 1){
            robot.thetae = M_PI * 0.5; // reset the theta! Use M_PI for standard math library compatibility
        }
    }
    else if(state == B2_DZ)
    {
        if(intersections == 0) robot.followLine(-0.04, robot.front, Side2Follow::LEFT);
        else if(intersections < 4)
        {
            if(intersections <2) robot.followLine(0.04, robot.front, Side2Follow::RIGHT);
            else robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
        else if(intersections == 4)
        {
            robot.followLine(0.08, robot.front, Side2Follow::LEFT);
        }
        else if(intersections == 5)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
    }
    else if(state == B2_DZ2_1)//16
    {
        if(intersections <= 2) robot.followLine(0.08, robot.front, Side2Follow::LEFT);
        else if(intersections == 3)//passar 1: Esq -1: direita
        {
            
        } 
        else if(intersections >= 3) robot.followLine(0.04, robot.front, Side2Follow::LEFT);
    }
    else if(state == B2_DROP)
    {
        robot.setRobotVW(-0.08, 0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);
    }
    else if(state == B2_LDZ)
    {
        if(intersections < 3)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
        else if(intersections == 3)
        {
            robot.followLine(0.08, robot.front, Side2Follow::LEFT);
        }
        else if(intersections >= 4)
        {
            robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
        }
    }
    else if(state == LAP_3) // ENTER reset COUNT!
    {
        if(intersections < 2) robot.followLine(0.08, robot.front, Side2Follow::LEFT);
        else if(intersections >= 2 && intersections < 4) robot.followLine(0.05, robot.front, Side2Follow::RIGHT);
        else if(intersections >= 4) robot.followLine(0.04, robot.front, Side2Follow::LEFT);
        
        if(intersections == 1){
            robot.thetae = M_PI * 0.5; // reset the theta! 
        }
    }
    else if(state == B3_DZ)
    {
        if(intersections == 0) {
            robot.followLine(-0.08, robot.front, Side2Follow::RIGHT);
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