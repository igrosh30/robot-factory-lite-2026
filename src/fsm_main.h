#ifndef FSM_MAIN_H
#define FSM_MAIN_H

#include "state_machines.h"
#include "robot.h"      // so you can access your robot (with front/back)


enum PathStrategy {
    TURN_AFTER_DETECTION = 1,    
    DISTANCE_TURN   = 2,    
    THETA_TURN      = 3     
};

class fsm_main : public state_machine_t
{
public:
    robot_t& robot;                

    int path_strategy = TURN_AFTER_DETECTION;
    
    //ALWAYS state what's the next state after manuever
    int state_after_maneuver;
    
    //Parameters to move x meters:
    float ref_s;//store initial value - when we enter state!
    float target_distance = 0.0f;
    int move_direction = 0;//1: forward || -1: bakcwards

    //Parameters to make a turn:
    float ref_theta;//store the initial value - when enter state!
    float target_turn_angle = PI/2;
    int turn_direction = 0;//1 for Left || -1 for Right
    
    
    fsm_main(robot_t& r);               // constructor

    virtual void next_state_rules() override;
    virtual void enter_state_actions_rules() override;
    virtual void state_actions_rules() override;
};

#endif