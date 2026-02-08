#include "state_machines.h"
#include "robot.h"
#include "config.h"

state_machines_t state_machine;
float vel = 0.0f;
float x,y = 0.0f;

void state_machines_t:: runStateMachine4Testing(robot_t& r)
{
    
    
    Node currentNode = r.targetNode;//Get current Node
    
    // 1. Create a static variable to remember time between loop calls
    static unsigned long stateTimer = 0;

    switch (state_machine.robotState)
    {
    case Calibration:
        static int count = 0;
        if(CALIBRATION_MODE)
        {
            if(count < 250){
                r.setRobotVW(0, 0.6);
                r.frontSensor.calibrate();
                r.backSensor.calibrate();
                count++;
            }
            else{
                //add some error to max and min! 5
                for(int i =0; i<5; i++)
                {
                    r.frontSensor.minValues[i] = r.frontSensor.minValues[i] - 5;
                    r.frontSensor.maxValues[i] = r.frontSensor.maxValues[i] + 10;

                    r.backSensor.minValues[i] = r.backSensor.minValues[i] - 5;
                    r.backSensor.maxValues[i] = r.backSensor.maxValues[i] + 10;
                }
                state_machine.robotState = Start;
            }
        }
        else{
            r.frontSensor.setCalibration(HARDCODED_MIN,HARDCODED_MAX);//hardcode vallues
            r.backSensor.setCalibration(HARDCODED_MIN,HARDCODED_MAX);
            state_machine.robotState = Start;
        }
        break;

    case Start:
        r.backSensor.countIntersections = 0;
        r.xe= 0;
        r.ye= 0;
        r.thetae = 0;
        r.dropBox();
        r.setRobotVW(0,0);
        //
        if(r.actuators.isSwitchOn)
        {
            state_machine.flag++;
            state_machine.robotState = Temp;
        }
        break;
    
    case Temp:
        vel = -0.06;
        state_machine.robotState = FollowLine;
        break;

    case Temp1:
        vel = -0.02;
        static int count1 = 0;
        if(count1 < 35){
            r.setRobotVW(vel,0);
            count1++;
        }
        else{
            state_machine.robotState = FollowLine;
        }

        break;
        

    case FollowLine:
        vel = -0.07;
        //calculate always the line error!
        static bool hasBox = false;
        static int countTheta = 0;
        if(r.thetae <= -6.5){
            if(countTheta <= 75)
            {
                countTheta++;
            }
            else{
                state_machine.robotState = Return;
            }
        }
        
        r.frontSensor.erro = r.frontSensor.getLineError();
        r.setRobotVW(vel,r.frontSensor.erro*r.frontSensor.kl);
        
        if(state_machine.flag > 4 && r.actuators.isSwitchOn && !hasBox ){
            state_machine.robotState = GrabBox;
            hasBox= true;
        }
        else state_machine.flag++;
        
        break;
    
    case FollowLineBack:
        vel = 0.07;
        //calculate always the line error!
        if(r.backSensor.countIntersections == 3){
            state_machine.robotState = Turn180;
        }
        r.backSensor.erro = r.backSensor.getLineError();
        r.setRobotVW(vel,r.backSensor.erro*r.backSensor.kl);
        
        break;
    
    case GrabBox:
        static int countGrab = 0;
        r.grabBox();
        //r.setRobotVW(0,0);
        if(countGrab <= 15)
        {
            r.setRobotVW(0.06,0);
            countGrab++;
        }
        else{
            state_machine.robotState = FollowLineBack;
        }
 
        break;
    
    case Turn180:
        
        if(r.thetae >= -3.1416){
            r.setRobotVW(0, 0.6);
        }
        else {
            stateTimer = millis();
            state_machine.robotState = Temp1;
        }
        
        break;
    
    case Return:
        static int countRet = 0;
        r.dropBox();
        if(countRet <= 20 ){
            r.setRobotVW(0.05,0);
            countRet++;
        }
        else r.setRobotVW(0,0);
            
        
            
        //state_machine.robotState = Start;
        break;

    default:
        state_machine.robotState= Start;
        enterDefault = true;
        break;
    }
}