#ifndef STATE_MACHINES_IG_H
#define STATE_MACHINES_IG_H

#include "config.h"
#include "robot.h"

enum class Event{
    None,
    SwitchOn,
    ReachedPickZone,
    PickedUp2Boxes,
    ReachedDropZone,
    Droped2Boxes,
    TimerExpired
};

enum class State{
    Idle,
    Start,
    FollowLine,
    PickBox,
    DropBox
};

struct Transition {
    State currentState;
    Event event;
    State nextState;
};

//Table with possible Transitions 
const Transition transitionTable[] = {
    {State::Idle,      Event::SwitchOn,        State::Start},
    {State::Start,     Event::None,            State::FollowLine},
    {State::Start,     Event::TimerExpired,    State::Idle},
    {State::FollowLine,Event::ReachedPickZone, State::PickBox},
    {State::FollowLine,Event::ReachedDropZone, State::DropBox},
    {State::PickBox,   Event::PickedUp2Boxes,  State::FollowLine},
    {State::DropBox,   Event::Droped2Boxes,    State::Start},
};

class state_machinesIg_t {
public:
    void state_machine_init();
    void runStateMachine(robot_t& r);
    void updadeInputs(robot_t& r);
    void getTransition(Event ev);
    Event getEvent(robot_t& r);
    
    //Debug Functions
    void setDebugState(State newState);
    State getCurrentState() const { return currentState; }
    
    //Action Functions:
    void onEntry(robot_t& r,State state);
    void onDo   (robot_t& r,State state);
    void onExit (robot_t& r,State state);

private:
    State currentState;
    unsigned long stateEntryTime;//millisenconds when current state was entered

/*
    1.⁠ ⁠update de variaveis input
    2.⁠ ⁠update do tis (time in state)
    3.⁠ ⁠transições
    4.⁠ ⁠⁠update dos estados de acordo com as transições
    5.⁠ ⁠⁠ações
    
    bool enterDefault = false;
    int flag = 0;
    currentState robotState;

    void runStateMachine4Testing(robot_t& r);
    //void runStateMachineV2(robot_t& r);

    Node setTarget (Node node,bool hasBox);
*/
};

extern state_machinesIg_t state_machineIg; // "There exists somewhere..."

#endif