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
    else if(state==FL_frontL && robot.front.sensor.countIntersections == 3)
    {
        set_next_state(Move_F);
    }
    else if(state== Move_F && (robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On) )
    {
        set_next_state(PickBox);
    }
    else if(state == PickBox && tis > 1.5) 
    {
        set_next_state(PickBox_Back);
    }
    //LOGIC TO GET BACK
    else if(state == PickBox_Back && robot.front.sensor.countIntersections == 1)
    {
        set_next_state(Box1GO2DropZone);
    }
    else if(state == Box1GO2DropZone && robot.thetae < -1.4)//
    {
        set_next_state(Box1GO2DropZone1);//foll Left
    }
    else if (state == Box1GO2DropZone1 && robot.front.sensor.countIntersections == 3 )
    {
        set_next_state(Box1GO2DropZone2);
    }
    else if (state == Box1GO2DropZone2 && tis > 2)
    {
        set_next_state(DropBox_Back);
    }
    else if(state == DropBox_Back && robot.front.sensor.countIntersections==1)
    {
        set_next_state(B1_LDZ);
    }
    else if(B1_LDZ && robot.thetae < -4.7)
    {
        set_next_state(LAP_2); 
    }
    else if(state==LAP_2 && (robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On))
    {
        set_next_state(PickBox2);
    }
    else if(state == PickBox2 && tis > 1.5) 
    {
        set_next_state(PickBox_Back1);
    }
    else if(state == PickBox_Back1 && robot.front.sensor.countIntersections == 1)
    {
        set_next_state(B2_DZ2);
    }
    else if(state == B2_DZ2 && robot.thetae < -1.8)
    {
        set_next_state(B2_DZ2_1);
    }
    else if(state == B2_DZ2_1 && tis < 2)
    {
        set_next_state(B3_PZ2);
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
        robot.front.sensor.countIntersections = 0;
        //robot.back.sensor.countIntersections = 0;
        //robot.back.sensor.readValues();
        robot.front.sensor.readValues();
    }
    else if(state == FL_frontL)
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == PickBox)
    {
        robot.front.actuators.magnetOn();//Only need to tell the hardware once to turn on the magnet ON!
        
    }
    else if(state == PickBox_Back)
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == Box1GO2DropZone  || state == Box1GO2DropZone1 || 
            state == Box1GO2DropZone2  )
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == DropBox_Back)
    {
        robot.front.actuators.magnetOff();
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == PickBox2)
    {
        robot.front.actuators.magnetOn();
    } 
    else if(state == B1_LDZ)
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == PickBox_Back1 || state == B2_DZ2 || state == B2_DZ2_1)
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == LAP_2)
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == B3_PZ2)
    {
        robot.setRobotVW(0,0);
    }
    
}

void fsm_main::state_actions_rules()
{
    // Run every loop while in current state
    float v_nom,w_nom = 0.0f;
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
        robot.followLine(v_nom,robot.front,Side2Follow::LEFT);
    }
    else if(state == Move_F)
    {
        robot.setRobotVW(0.05,0);
    }
    else if(state== PickBox)
    {
        //Ensure it picks the box! 
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.03,0.2);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.03,-0.2);
        }
        else{
            robot.setRobotVW(0.03,0);
        }
        
    }
    else if(state == PickBox_Back)
    {
        robot.setRobotVW(-0.08,0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);//just to count the intersections!
    }
    else if(state == Box1GO2DropZone)
    {
        if(robot.front.sensor.countIntersections < 4)
        {
            robot.followLine(0.08,robot.front,Side2Follow::RIGHT);
        }
        else if(robot.front.sensor.countIntersections == 4)
        {
            robot.followLine(0.08,robot.front,Side2Follow::LEFT);
        }
        else if(robot.front.sensor.countIntersections == 5){
            robot.followLine(0.08,robot.front,Side2Follow::RIGHT);
        }
    }
    else if(state == Box1GO2DropZone1)
    {
        robot.followLine(0.08, robot.front,Side2Follow::LEFT);
    }
    else if(state == Box1GO2DropZone2)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
    }
    else if (state == DropBox_Back)
    {
        robot.setRobotVW(-0.04,0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);
    }
    else if(state == B1_LDZ)
    {
        if(robot.front.sensor.countIntersections < 4)
        {
            robot.followLine(0.08,robot.front,Side2Follow::RIGHT);
        }
        else if(robot.front.sensor.countIntersections == 4)
        {
            robot.followLine(0.08,robot.front,Side2Follow::LEFT);
        }
        else if(robot.front.sensor.countIntersections == 5){
            robot.followLine(0.08,robot.front,Side2Follow::RIGHT);
        }
    }
    else if(state == LAP_2)
    {
        
        if(robot.front.sensor.countIntersections < 2) robot.followLine(0.08, robot.front,Side2Follow::LEFT);
        else if(robot.front.sensor.countIntersections == 2) robot.followLine(0.05, robot.front,Side2Follow::RIGHT);
        else if(robot.front.sensor.countIntersections==3 )robot.followLine(0.05, robot.front,Side2Follow::LEFT);
        
        if(robot.front.sensor.countIntersections == 1){
            robot.thetae = PI*0.5; // reset the theta! 
        }
    }
    else if(state == PickBox2)
    {
        //Ensure it picks the box! 
        if(robot.front.actuators.isSwitch_left_On)
        {
            robot.setRobotVW(0.03,0.2);
        }
        else if(robot.front.actuators.isSwitch_right_On)
        {
            robot.setRobotVW(0.03,-0.2);
        }
        else{
            robot.setRobotVW(0.03,0);
        }
    }
    else if(state == PickBox_Back1)
    {
        robot.setRobotVW(-0.08,0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);
    }
    else if(state== B2_DZ2)
    {
        if(robot.front.sensor.countIntersections < 3)
        {
            robot.followLine(0.08,robot.front,Side2Follow::RIGHT);

        }
        else if(robot.front.sensor.countIntersections == 3)
        {
            robot.followLine(0.08,robot.front,Side2Follow::LEFT);
        }
        else if(robot.front.sensor.countIntersections >= 4){
            robot.followLine(0.08,robot.front,Side2Follow::RIGHT);
        }
    }
    else if(state == B2_DZ2_1)
    {
        if(robot.front.sensor.countIntersections < 2) robot.followLine(0.08,robot.front,Side2Follow::LEFT);
        else if(robot.front.sensor.countIntersections == 2 ) robot.followLine(0.06,robot.front,Side2Follow::RIGHT);
        else if(robot.front.sensor.countIntersections == 3) robot.followLine(0.08,robot.front,Side2Follow::LEFT);
    }

}
void control(robot_t& robot)
{
    state_machines.step();
}