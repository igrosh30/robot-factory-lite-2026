#include "fsm_main.h"
#include "config.h"

fsm_main::fsm_main(robot_t& r) : robot(r)
{
    // You can set initial state here if you want
    // force_state(Idle);   // optional
}

void fsm_main::next_state_rules()
{
    // Step 3: your transitions go here
    if(state == Idle && robot.back.actuators.isSwitchOn)
    {
        set_next_state(Calibration);
    }
    else if(state == Calibration && robot.thetae >= 6.2)
    {
        for(int i = 0; i<5; i++){
            robot.front.sensor.minValues[i] = robot.front.sensor.minValues[i] - 5;
            robot.front.sensor.maxValues[i] = robot.front.sensor.maxValues[i] + 10;

            robot.back.sensor.minValues[i] = robot.back.sensor.minValues[i] - 5;
            robot.back.sensor.maxValues[i] = robot.back.sensor.maxValues[i] + 10;
        }
        set_next_state(Start);
    }
    else if(state == Start && robot.front.actuators.isSwitchOn)
    {
        set_next_state(FollowLineFrontL);
    }
    else if(state == FollowLineFrontL && robot.front.actuators.isSwitchOn && robot.front.sensor.countIntersections == 3)
    {
        robot.front.sensor.countIntersections = 0;//reset intersections counting
        set_next_state(PickFrontBox);
    }
    else if(state == PickFrontBox && tis >= 0.2)//still need to see this
    {
        set_next_state(FollowLineBackL);
    }
    else if(state == FollowLineBackL && robot.back.actuators.isSwitchOn)
    {
        set_next_state(PickBackBox);
    }
    else if(state == PickBackBox && tis>= 0.2) 
    {
        set_next_state(FollowLine2ExitPickZone);
    }

}

void fsm_main::enter_state_actions_rules()
{
    if(state == Idle)
    {
        robot.setRobotVW(0,0);
        robot.front.actuators.magnetOff();
        robot.back.actuators.magnetOff();
    }
    else if(state == Start)
    {
        robot.setRobotVW(0,0);
        robot.thetae = PI*0.5;
        //Check this!
        robot.xe = -69.5;
        robot.ye = -35.5;
        robot.front.sensor.countIntersections = 0;
        robot.back.sensor.countIntersections = 0;
    }
    else if(state == PickFrontBox)
    {
        robot.front.actuators.magnetOn();//Only need to tell the hardware once to turn on the magnet ON!
        
    }
    else if(state == PickBackBox)
    {
        robot.back.actuators.magnetOn();//Tell once to the hardware to pick the Box
        
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
            robot.front.sensor.calibrate();
            robot.back.sensor.calibrate();
        }
        else{
            robot.front.sensor.setCalibration(HARDCODED_FRONT_MIN,HARDCODED_FRONT_MAX);
            robot.back.sensor.setCalibration(HARDCODED_BACK_MIN,HARDCODED_BACK_MAX);
        }
    }
    else if(state == FollowLineFrontL)
    {
        v_nom = 0.07;
        robot.followLine(v_nom,robot.front,Side2Follow::LEFT);
    }
    else if(state == PickFrontBox)
    {
        robot.setRobotVW(0.04, 0);//make sure we pick the box!
    }
    else if(state == FollowLineBackL)
    {
        v_nom = -0.07;
        robot.followLine(v_nom,robot.back,Side2Follow::LEFT);
        robot.setRobotVW(v_nom,w_nom);
    }
    else if(state == PickBackBox)
    {
        robot.setRobotVW(-0.05,0);
    }
    else if(state == FollowLine2ExitPickZone)
    {
        float error = 0.0f;
        //STOP Robot!
        robot.setRobotVW(0,0);
    }
    
}
void control(robot_t& robot)
{

    
    state_machines.step();
}