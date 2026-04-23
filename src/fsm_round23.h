#ifndef FSM_ROUND23_H
#define FSM_ROUND23_H

#include "state_machines.h"
#include "robot.h"      
#include "config.h"



struct BoxRoun2
{
    uint8_t pick_slot;
    uint8_t drop_slot;
    char color;
};


class fsm_round23 : public state_machine_t
{
public:
    robot_t& robot;                
    
    #ifdef ROBOT_MASTER
    
    
    //---------RED BOX-----------//
    uint8_t red_pick_slots[4];
    uint8_t total_reds = 0;
    uint8_t current_red_index = 0;

    //---------GREEN BOX-----------//
    uint8_t green_pick_slots[4];
    uint8_t total_greens = 0;

    //---------BLUE BOX-----------//
    uint8_t blue_pick_slots[4]; //here we have all the slots of the blue boxes at the beggining of the round
    uint8_t total_blues = 0;
    
    uint8_t M_green_PICK[4];
    uint8_t MASTER_greenBox = 0;

    uint8_t M_blue_PICK[2];
    uint8_t MASTER_blueBox = 0;

    void build_sequence_from_IR(String ir_data);
    void build_slaveSlots();
    #endif

    #ifdef ROBOT_SLAVE_00
    uint8_t processBox_MachineB;
    #endif

    #if defined(ROBOT_SLAVE_01) || defined(ROBOT_MASTER)
    //This will be sent to SLAVE_01 by MASTER!!
    uint8_t SLAVE_greenBox = 0;
    uint8_t S_green_PICK[2];
    uint8_t current_green_index = 0; // do I need current blue index? I just need box_index
    uint8_t SLAVE_blueBox = 0; // number of times that the slave will need to pick from that spot a box!
    uint8_t S_blue_PICK[2];
    uint8_t current_blue_index = 0; // do I need current blue index? I just need box_index
    uint8_t processBox_MachineA;
    bool hasBlueBoxesInfo;
    #endif

    int intersections_trigger = 0;

    float v_req_nav;
    float v_req_leaving_pickZ;

    BoxRound2 currentBox;
    int current_box_index;
    int drop_sequence[4]; //this is Globaly defined!

    int pick_slot = -1; 
    int drop_slot = -1;  
    
    bool isFromMachine = false;//flag to Master know if he's entering from the processMahine or not!
    int state_after_maneuver;
    float d_leave_docking = 0.25f;
    float d_retrive_process_box = 0.19f;
    float d_retrive_from_wearhouse = 0.1f; 
    float d_mv_aft_intersection = 0.03f; //distance to moove before turning!
    float d_mv_aft_intersection_drop = 0.03f;
    float d_mv_aft_intersection_B = 0.06f;
    float d_aft_dropRed = 0.16f;
    float ref_s;
    float target_distance = 0.0f;
    int move_direction = 0;

    float ref_theta;
    float target_turn_angle = PI/2;
    int turn_direction = 0;// 1: left / -1: right 

    fsm_round23(robot_t& r);   
    void build_currentBox(BoxRound2& box);
    //stores the slots and the number of blueBoxes to process at MASTER side - then SEND TO SLAVE it's number! 
    

    virtual void next_state_rules() override;
    virtual void enter_state_actions_rules() override;
    virtual void state_actions_rules() override;


};

#endif