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
    else if(state == PickBox && tis >= 1.5) 
    {
        set_next_state(PickBox_Back);
    }
    //LOGIC TO GET BACK
    else if(state == PickBox_Back && robot.front.sensor.countIntersections == 1)
    {
        set_next_state(Box1GO2DropZone);
    }
    else if(state == Box1GO2DropZone && robot.front.sensor.countIntersections == 4)
    {
        set_next_state(Box1GO2DropZone1);//foll Left
    }
    else if(state == Box1GO2DropZone1 && robot.front.sensor.countIntersections == 1)
    {
        set_next_state(Box1GO2DropZone2);//follow Right
    }
    else if (state == Box1GO2DropZone2 && robot.thetae < -1.8 )
    {
        set_next_state(Box1GO2DropZone3);
    }
    else if (state == Box1GO2DropZone3 && robot.front.sensor.countIntersections == 3)
    {
        set_next_state(Box1GO2DropZone4);
    }
    else if (state == Box1GO2DropZone4 && tis > 2)
    {
        set_next_state(DropBox_Back);
    }
    else if(state == DropBox_Back && robot.front.sensor.countIntersections==1)
    {
        set_next_state(Box1GO2PickZone);
    }
    //LOGIC TO GET BACK TO START! 
    else if(state == Box1GO2PickZone && robot.front.sensor.countIntersections == 4)
    {
        set_next_state(Box1GO2PickZone1);//fol Left
    }
    else if(state == Box1GO2PickZone1 && robot.front.sensor.countIntersections == 1)
    {
        set_next_state(Box1GO2PickZone2);//follow Right
    }
    else if (state == Box1GO2PickZone2 && robot.thetae < -4.7 )
    {
        set_next_state(Box1GO2PickZone3);//Here reset the theta to 90! when we count 1 intersection! 
    }
    else if (state == Box1GO2PickZone3 && robot.front.sensor.countIntersections == 2)
    {
        set_next_state(Box1GO2PickZone4);
    }
    else if(state == Box1GO2PickZone4 && robot.front.sensor.countIntersections == 1)//Follow Right
    {
        set_next_state(Box1GO2PickZone5);
    }
    else if(state == Box1GO2PickZone5 && (robot.front.actuators.isSwitch_left_On | robot.front.actuators.isSwitch_right_On))
    {
        set_next_state(PickBox);
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
            state == Box1GO2DropZone2 || state == Box1GO2DropZone3 )
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == Box1GO2PickZone  || state == Box1GO2PickZone1 || 
            state == Box1GO2PickZone4 || state == Box1GO2PickZone3 || 
            state == Box1GO2PickZone5  )
    {
        robot.front.sensor.countIntersections = 0;
    }
    else if(state == DropBox_Back)
    {
        robot.front.actuators.magnetOff();
        robot.front.sensor.countIntersections = 0;
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
        robot.setRobotVW(0.03,0);
    }
    else if(state == PickBox_Back)
    {
        robot.setRobotVW(-0.08,0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);
    }
    else if(state == Box1GO2DropZone)
    {
        robot.followLine(0.1, robot.front,Side2Follow::RIGHT);
    }
    else if(state == Box1GO2DropZone1)
    {
        robot.followLine(0.1, robot.front,Side2Follow::LEFT);
    }
    else if(state == Box1GO2DropZone2)
    {
        robot.followLine(0.1, robot.front, Side2Follow::RIGHT);
    }
    else if(state == Box1GO2DropZone3)
    {
        robot.followLine(0.08, robot.front,Side2Follow::LEFT);
    }
    else if (state == DropBox_Back)
    {
        robot.setRobotVW(-0.06,0);
        robot.front.sensor.getLineError(Side2Follow::LEFT);
    }
    else if(state == Box1GO2PickZone)
    {
        robot.followLine(0.08, robot.front,Side2Follow::RIGHT);
    }
    else if(state == Box1GO2PickZone1)
    {
        robot.followLine(0.08, robot.front,Side2Follow::LEFT);
    }
    else if(state == Box1GO2PickZone2)
    {
        robot.followLine(0.08, robot.front, Side2Follow::RIGHT);
    }
    else if(state == Box1GO2PickZone3)
    {
        robot.followLine(0.08, robot.front,Side2Follow::LEFT);
        if(robot.front.sensor.countIntersections == 1){
            robot.thetae = PI*0.5; // reset the theta! 
        }
    }
    else if(state == Box1GO2PickZone4)
    {
        
    }
}
void control(robot_t& robot)
{
    state_machines.step();
}