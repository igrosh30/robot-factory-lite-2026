#include "States.h"
#include "robot.h"
#include "MotorController.h"

void States:: setState(RobotState state)
{
    switch (state)
    {
    case MOVE_FOR:
        float v_conts = 0.01;
        float w = 0.01;
        robot.setRobotVW(v_conts,w);//need a way where in the code we calculate the PID
        //this function sets robot.v_req = v_const and robot.w_req = w

        break;
    case MOVE_BAC:
        break;
    default:
        break;
    }
}