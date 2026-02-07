#include "state_machines.h"
#include "robot.h"
#include "config.h"

state_machines_t state_machine;

void state_machines_t:: runStateMachine4Testing(robot_t& r)
{
    float x,y = 0.0f;
    
    Node currentNode = r.targetNode;//Get current Node
    
    // 1. Create a static variable to remember time between loop calls
    static unsigned long stateTimer = 0;

    switch (state_machine.robotState)
    {
    case Calibration:
        static int count = 0;
        if(CALIBRATION_MODE)
        {
            if(count < 150){
                r.setRobotVW(0, 0.8);
                r.frontSensor.calibrate(); 
                count++;
            }
            else{
                //add some error to max and min! 5
                for(int i =0; i<5; i++)
                {
                    r.frontSensor.minValues[i] = r.frontSensor.minValues[i] - 5;
                    r.frontSensor.maxValues[i] = r.frontSensor.maxValues[i] + 10;
                }
                state_machine.robotState = Start;
            }
        }
        else{
            r.frontSensor.setCalibration(HARDCODED_MIN,HARDCODED_MAX);//hardcode vallues
            //state_machine.robotState = Start;
        }
        break;

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
        //calculate always the line error!
        r.frontSensor.erro = r.frontSensor.getLineError();
        r.setRobotVW(-0.04,r.frontSensor.erro*r.frontSensor.kl);
        if(state_machine.flag > 4 ){
            if(r.actuators.isSwitchOn)
            {
                state_machine.robotState = GrabBox;
            }
            else{
                
                //r.followLineLeft(r.v_req,r.frontSensor.kl);
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