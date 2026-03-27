#ifndef FSM_MAIN_H
#define FSM_MAIN_H

#include "state_machines.h"
#include "robot.h"      // so you can access your robot (with front/back)

class fsm_main : public state_machine_t
{
public:
    robot_t& robot;                     // reference to your robot

    int state_after_maneuver;
    float ref_s,ref_theta;
    float target_turn_angle = PI/2;
    int turn_direction;//1 for Left  -1 for Right

    fsm_main(robot_t& r);               // constructor

    virtual void next_state_rules() override;
    virtual void enter_state_actions_rules() override;
    virtual void state_actions_rules() override;
};

#endif