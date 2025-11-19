#include <Arduino.h>
#include "proj_types.h"
#include "robot.h"
#include "IRline.h"

void init_control(robot_t& robot)
{
  
}

extern IRLine_t IRLine;

void control(robot_t& robot)
{
    robot.tis = millis() - robot.tes;

    // Rules for the state evolution
     if(robot.state == 0 && robot.LastTouchSwitch && !robot.TouchSwitch) {
      robot.rel_s = 0;
      robot.setState(1);

    } else if (robot.state == 1 && robot.TouchSwitch) {
      robot.setState(2);

    } else if(robot.state == 2 && robot.tis > 100) {
      robot.rel_s = 0;
      robot.setState(3);

    } else if(robot.state == 3 && robot.rel_s < -0.12) {
      robot.rel_theta = 0;
      robot.setState(4);

    //} else if(robot.state == 4 && robot.rel_theta > radians(90) && IRLine.total > 1500) {
    } else if(robot.state == 4 && robot.rel_theta > radians(170)) {
      robot.setState(5);
      IRLine.crosses = 0;

    } else if(robot.state == 5 && IRLine.crosses >= 4) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(6);

    } else if(robot.state == 6 && robot.rel_theta < radians(-70) && IRLine.total > 1500) {
      robot.setState(7);

    } else if(robot.state == 7 && robot.tis > 2000) {
      IRLine.crosses = 0;
      robot.setState(8);

    } else if (robot.state == 202 && robot.tis > robot.T1) {
      robot.setState(200);
    }

    // Actions in each state
    if (robot.state == 0) {         // Robot Stoped            
      robot.solenoid_state = 0;
      robot.setRobotVW(0, 0);
    
    } else if (robot.state == 10) { //Test
      robot.setRobotVW(0.1, 0);      

    } else if (robot.state == 100) {
      robot.v_req = 0;
      robot.w_req = 0;

    } else if (robot.state == 101) {
      robot.setRobotVW(robot.v_req,robot.w_req);
    } else if (robot.state == 199) {
     

    } else if (robot.state == 200) {
      robot.PWM_1 = 0;
      robot.PWM_2 = 0;

    } else if (robot.state == 201) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;
    
    } else if (robot.state == 202) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;      
    }
    else if (robot.state == 180)//folow line
    {
      float w = 0.01 * (IRLine.IR_values[0] - IRLine.IR_values[4]);
      robot.setRobotVW(robot.v_req,w);
    }
    else if (robot.state == 188)
    {
      float w = 0.002*(IRLine.IR_values[1] - 155) + 
                0.01*(IRLine.IR_values[0] - IRLine.IR_values[3]);
      robot.setRobotVW(robot.v_req, w);
    }
  
}
