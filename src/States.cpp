#include "States.h"
#include "robot.h"
#include "MotorController.h"

void States:: setState(RobotState new_state)
{
    float v = 0.0f;
    float w = 0.0f;
    switch (new_state)
    {
    case STATE_FORWARD:
        v = 0.08;
        w = 0.00;
        robot.setRobotVW(v,w);//need a way where in the code we calculate the PID
        //this function sets robot.v_req = v_const and robot.w_req = w

        break;
    case STATE_STOP:
        v = 0.0;
        w = 0.0;
        robot.setRobotVW(v,w);
        break;

    default://stop the robot
        v = 0.0;
        w = 0.0;
        robot.setRobotVW(v,w);
        break;
    }
}