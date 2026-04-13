#ifndef APPLAYER_H
#define APPLAYER_H

#include "dataLink.h"
#include "ComProtocol.h"
#include <string.h>   // for memcpy

class AppLayer
{
private:
    DataLink ddl;           // low-level serial + parser

    NodeId myId = MASTER;

    // Communication status flags
    bool pongReceived = false;
    bool ackReceived = false;
    uint8_t lastCommandSent = 0;

    // NEW: Received command from the other robot (used by SLAVE mainly)
    bool     hasNewCommandReceived = false;
    uint8_t  receivedCmdId         = 0;
    uint8_t  receivedDirection     = 0;   // 0 = CMD, 1 = STATUS
    uint8_t  receivedParamLen      = 0;
    uint8_t  receivedParams[MAX_PAYLOAD - 2] = {0};

    void sendFrame(const Frame& f);
    void processFrame(const Frame& f);
    void buildHeader(Frame& f, NodeId dest, uint8_t type, uint8_t payloadLen);

public:
    void init(Stream* port, NodeId id);

    // --- Sending ---
    void sendPing(NodeId dst);
    void sendCommand(NodeId dst, uint8_t cmdId);                    // old simple version (wrapper)
    void sendCommandWithData(NodeId dst, uint8_t cmdId,
                             const uint8_t* data, uint8_t dataLen); // new powerful version

    void sendErrorAck(uint8_t cmdID);

    // --- Receiving ---
    bool update();          // must be called every loop

    bool hasReceivedPong() const { return pongReceived; }
    bool hasReceivedAck()  const { return ackReceived; }

    // New getters for the FSM (mainly on the SLAVE side)
    bool           hasNewCommand()      const { return hasNewCommandReceived; }
    uint8_t        getReceivedCmdId()   const { return receivedCmdId; }
    uint8_t        getReceivedDirection() const { return receivedDirection; }
    uint8_t        getReceivedParamLen()const { return receivedParamLen; }
    const uint8_t* getReceivedParams()  const { return receivedParams; }

    void clearNewCommand() {receivedCmdId= 0; hasNewCommandReceived = false; }//I should reset the receiveCmdID and param!
};

#endif