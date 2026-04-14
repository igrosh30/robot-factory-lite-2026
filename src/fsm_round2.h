#ifndef FSM_ROUND2_H
#define FSM_ROUND2_H

#include "state_machines.h"
#include "robot.h"      
#include "config.h"


enum PathStrategy_to_ProcessBox {
    FOLLOW_LINE = 1,
    FL_AND_TURNS = 2,
};

struct BoxRound2
{
    uint8_t pick_slot;
    uint8_t drop_slot;
    char color;
};


class fsm_round2 : public state_machine_t
{
public:
    robot_t& robot;                
    int path_strategy = FL_AND_TURNS;
    BoxRound2 currentBox;
    int drop_sequence[4];
    int current_box_index;
    bool isFromMachine = false;

    uint8_t green_pick_slots[4];
    uint8_t slave_blue_pick_slots[4];
    uint8_t total_greens = 0;
    uint8_t current_green_index = 0; 

    uint8_t blue_pick_slots[4];
    uint8_t master_blue_pick_slots[2];
    uint8_t total_blues = 0;
    uint8_t num_master_blue_boxes = 0;
    uint8_t num_slave_blue_boxes = 0;
    uint8_t current_blue_index = 0; 
    
    int pick_slot = -1; 
    int drop_slot = -1;  
    
    
    int state_after_maneuver;
    float d_retrive_process_box = 0.19f;
    float d_retrive_from_wearhouse = 0.1f; 
    float d_mv_aft_intersection = 0.05f; //distance to moove before turning!
    float ref_s;
    float target_distance = 0.0f;
    int move_direction = 0;

    float ref_theta;
    float target_turn_angle = PI/2;
    int turn_direction = 0;// 1: left / -1: right 

    fsm_round2(robot_t& r);         
    void build_sequence_from_IR(String ir_data);
    void build_currentBox(BoxRound2& box, NodeId robotID);
    
    //stores the slots and the number of blueBoxes to process! 
    void build_blueBoxPick(uint8_t num_green_box, NodeId robotID);

    virtual void next_state_rules() override;
    virtual void enter_state_actions_rules() override;
    virtual void state_actions_rules() override;

    
    void send_sync_to_slave();
    bool check_sync_from_master();
};

#endif