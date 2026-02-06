#include "state_machines.h"
#include "robot.h"

state_machines_t state_machine;
//state_machine.flag = 0;

void state_machines_t:: runStateMachine4Testing(robot_t& r)
{
    float x,y = 0.0f;
    
    Node currentNode = r.targetNode;//Get current Node
    
    // 1. Create a static variable to remember time between loop calls
    static unsigned long stateTimer = 0;

    switch (state_machine.robotState)
    {
    case Start:
        r.xe= 0;
        r.ye= 0;
        r.dropBox();
        r.setRobotVW(0,0);
        if(r.actuators.isSwitchOn)
        {
            state_machine.flag++;
            //state_machine.robotState = FollowLine;
        }
        break;

    case SetVW:

        r.setRobotVW(r.v_req,r.w_req);
        break;

    case GoToXY:
        
        //r.targetNode = setTarget(currentNode, r.hasBox());
        state_machine.robotState = FollowLine;
        
        break;

    case FollowLine:
        if(state_machine.flag > 4 ){
            if(r.actuators.isSwitchOn)
            {
                state_machine.robotState = GrabBox;
            }
            else{
                r.followLineLeft(r.v_req,r.frontSensor.kl);
            }
        }
        else state_machine.flag++;
        
        break;
    
    case GrabBox:
        r.setRobotVW(0,0);
        r.grabBox();

        
        //state_machine.robotState = Return;
        
        break;
    
    case Return:
        stateTimer = millis();
        if(millis() - stateTimer < 4000) r.setRobotVW(-r.v_req,0);
        else state_machine.robotState = Start;
        break;

    default:
        state_machine.robotState= Start;
        enterDefault = true;
        break;
    }
}