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
        v = 0.09;
        w = 0.00;
        robot.setRobotVW(v,w);//need a way where in the code we calculate the PID
        //this function sets robot.v_req = v_const and robot.w_req = w

        break;
    case STATE_STOP:
        v = 0.0;
        w = 0.0;
        robot.setRobotVW(v,w);
        break;

    case STATE_OUTPUT:
        
        Serial.println("-----------------------Robot Variables------------------------");
        Serial.print("Enc0: "); Serial.println(robot.enc1);
        Serial.print("Enc1: "); Serial.println(robot.enc2);
        Serial.print("x [m]: "); Serial.println(robot.xe);
        Serial.print("y [m]: "); Serial.println(robot.ye);
        Serial.print("θ [rad]: "); Serial.println(robot.thetae);
        Serial.print("Rel_s [m]: "); Serial.println(robot.rel_s);
        Serial.println();
        delay(5000);
        break;
    default://stop the robot
        v = 0.0;
        w = 0.0;
        robot.setRobotVW(v,w);
        break;
    }
}