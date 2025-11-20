
class States
{
    typedef enum
    {
        STOP = 0,
        MOVE_FOR = 101,
        MOVE_BAC = 100,
    }RobotState;
private:
    /* data */
    void setState(RobotState state);
public:
    States(/* args */);
    ~States();
};

States::States(/* args */)
{
}

States::~States()
{
}
