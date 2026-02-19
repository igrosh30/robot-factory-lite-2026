#include "state_machines_Ig.h"
#include "robot.h"
#include "config.h"

//IN SETUP MAIN:: state_machine_init();
//IN LOOP MAIN::  runStateMachine(robot);

state_machinesIg_t state_machineIg;

void state_machinesIg_t:: state_machine_init()
{
    currentState = State::Idle;
    stateEntryTime = millis();
}

void state_machinesIg_t:: runStateMachine(robot_t& r){
    //1- read the inputs:
    updadeInputs(r);//should read all the inputs?! 
    
    //have the inputs -> what's the event?
    getTransition(getEvent(r));//I updade the current state with the newOne! 

    State oldState = currentState;
    if(oldState != currentState)//we entered a new state! 
    {
        //should I exit?!
        onExit(r,currentState);
        
        onEntry(r,currentState);
    }
    else{//we are looping inside the state! 
        onDo(r,currentState);
    }

    //need to do the actions! 

}

void state_machinesIg_t::updadeInputs(robot_t& r){
    r.front.actuators.update();
    r.back.actuators.update();
    r.odometry();//update x,y and theta
    r.front.sensor.getLineError();
    r.back.sensor.getLineError();
    
    //UPDATE_NODE!

    //INPUT THAT I COULD HAVE? -> r.updateLocalisation() -> Input for Localisation! - we can use fresh odometry and intersection info
    //I need to update the currentRobot node! 
    //another input: TIME?!
}

Event state_machinesIg_t:: getEvent(robot_t& r)
{
    if(r.front.actuators.isSwitchOn){
        return Event::SwitchOn;
    }
    else if(r.atPikZone()){
        return Event::ReachedPickZone;
    }
    else if (r.has2Boxes())//still need some thinking!
    {
        return Event::PickedUp2Boxes;
    }
    else if(r.atDropZone())
    {
        return Event::ReachedDropZone;
    }
    else if(r.hasDroped2Boxes())
    {
        return Event::Droped2Boxes;
    }
    else{
        return Event::None;
    }
}

void state_machinesIg_t:: getTransition(Event ev)
{
    for( const auto& t: transitionTable)
    {
        if(t.currentState == currentState && t.event == ev){
            currentState = t.nextState;
            return;
        }
    }
    //what if I don't find??- stay in current state!
}

void state_machinesIg_t:: onExit(robot_t& r, State state)
{
    switch (state)
    {
    case State::FollowLine:
        r.setRobotVW(0,0);
        break;

    case State::DropBox:
        r.dropBox();
        break;

    default:
        break;
    }
}

void state_machinesIg_t:: onDo(robot_t& r, State state)
{
    /*
    switch (state)
    {

    case State::FollowLine:
        //where we are in trouble...
        float error = r.frontSensor.getLineError();
        r.setRobotVW(0.06,error*r.frontSensor.kl);
    default:
        break;
    }
        */
}

void state_machinesIg_t::onEntry(robot_t& r, State state)
{
    switch (state)
    {
    case State::Idle:
        r.setRobotVW(0,0);
        r.dropBox();
        break;
    case State::Start:
        r.currentNode = mappingNodes1[21];
    default:
        break;
    }
}


void state_machinesIg_t:: setDebugState(State newState)
{
    currentState = newState;
    stateEntryTime = millis();
}

/*
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
}*/