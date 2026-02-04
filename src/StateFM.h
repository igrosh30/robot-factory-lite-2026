#ifndef STATEFM_H
#define STATEFM_H

#include "config.h"
#include "robot.h"

class States {
public:
    bool enterDefault = false;
    currentState robotState;

    void runStateMachine4Testing(robot_t& r);
    //void runStateMachineV2(robot_t& r);

    Node setTarget (Node node,bool hasBox);

};

#endif