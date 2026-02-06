#ifndef STATE_MACHINES_H
#define STATE_MACHINES_H

#include "config.h"
#include "robot.h"

class state_machines_t {
public:
    bool enterDefault = false;
    int flag = 0;
    currentState robotState;

    void runStateMachine4Testing(robot_t& r);
    //void runStateMachineV2(robot_t& r);

    Node setTarget (Node node,bool hasBox);

};

extern state_machines_t state_machine; // "There exists somewhere..."

#endif