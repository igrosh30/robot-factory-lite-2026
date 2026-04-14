#ifndef COM_FSM_H
#define COM_FSM_H

#include "state_machines.h"
#include "robot.h"      
#include "config.h"

class fsm_COM : public state_machine_t
{

public:
    robot_t& robot;
    int sendingTries;                

    fsm_COM(robot_t& r);  

    virtual void next_state_rules() override;
    virtual void enter_state_actions_rules() override;
    virtual void state_actions_rules() override;
};

#endif