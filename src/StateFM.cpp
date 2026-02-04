#include "StateFM.h"
#include "robot.h"

void States:: runStateMachine4Testing(robot_t& r )
{
    float x,y = 0.0f;
    float Vnom = 0.05;
    float k = 0.2;
    Node currentNode = r.targetNode;//Get current Node
    
    // 1. Create a static variable to remember time between loop calls
    static unsigned long stateTimer = 0;

    switch (robotState)
    {
    case Start:
        r.xe= 0;
        r.ye= 0;
        r.dropBox();
        r.setRobotVW(0,0);
        if(r.actuators.switchOn)
        {
            robotState = FollowLine;
            
        }
        break;

    case GoToXY:
        
        //r.targetNode = setTarget(currentNode, r.hasBox());
        robotState = FollowLine;
        
        break;

    case FollowLine:
        Vnom = 0.05;
        k    = 0.8;
        if(r.actuators.switchOn)
        {
            robotState = GrabBox;
        }
        else{
            r.followLineLeft(Vnom,k);
        }
        break;
    
    case GrabBox:
        r.setRobotVW(0,0);
        r.grabBox();

        stateTimer = millis();
        robotState = Return;
        
        break;
    
        case Return:

        if(millis() - stateTimer < 3500) r.setRobotVW(-Vnom,0);
        else robotState = Start;
        break;

    default:
        robotState= Start;
        enterDefault = true;
        break;
    }
}