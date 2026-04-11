#ifndef APP_LAYER_H
#define APP_LAYER_H

#include "ComProtocol.h"
#include "dataLink.h"

class AppLayer {

protected:
    NodeId myId;
    DataLink ddl;

    // --- MAILBOX FLAGS (Used by the Robot FSM) ---
    volatile bool pongReceived = false;
    volatile bool ackReceived = false;
    volatile bool hasNewCommand = false;
    uint8_t lastCommandSent = 0;
    uint8_t latestCmd = 0;

public:
    // Constructor
    void init(Stream* port, NodeId myId);


    // --- INTERNAL PROTOCOL LOGIC ---
    void update();
    void processFrame(const Frame& f);
    void buildHeader(Frame& f, NodeId dest, uint8_t type, uint8_t payloadLen);
    void sendFrame(const Frame& f);

    
    // --- MASTER ACTIONS ---
    void sendPing(NodeId dst);
    void sendCommand(NodeId dst, uint8_t cmdId);
    
    bool hasReceivedPong();
    bool hasReceivedAck();

   

    // --- SLAVE ACTIONS ---
    bool getLatestCommand(uint8_t& outCmd);
    void sendErrorAck(uint8_t cmdID);

};

#endif