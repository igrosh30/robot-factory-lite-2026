#include "AppLayer.h"


void AppLayer::init(Stream *port, NodeId id)
{
    this->myId = id;
    this->ddl.init(port);
}

void AppLayer::sendFrame(const Frame& f)
{
    this->ddl.write(f);
}


void AppLayer::sendPing(NodeId dst) {
    this->pongReceived = false; 
    this->lastCommandSent = MsgType::PING;
    Frame pingFrame;
    buildHeader(pingFrame, dst, MsgType::PING, 0); 
    sendFrame(pingFrame);
}

void AppLayer::update()
{
    this->ddl.update();

    if(this->ddl.hasNewFrame())
    {
        Frame incomingFrame;
        this->ddl.getFrame(incomingFrame);
        this->processFrame(incomingFrame);
    }
}

void AppLayer::sendCommand(NodeId dst, uint8_t cmdId)
{
    this->lastCommandSent = cmdId;
    this->ackReceived = false;

    //NOTE cmdID > 20 is asking something!
    uint8_t direction = (cmdId >= 20) ? 1 : 0;   // 0 = CMD, 1 = STATUS

    Frame cmdFrame;
    buildHeader(cmdFrame, dst, MsgType::DATA, 2);   // direction (1 byte) + cmdId (1 byte)

    cmdFrame.payload.data.direction = 0;   // 0 = CMD (MASTER -> SLAVE)
    cmdFrame.payload.data.cmdId     = cmdId;

    sendFrame(cmdFrame);
}

void AppLayer::processFrame(const Frame& f) 
{ 
    uint8_t type = f.header.type;
    uint8_t cmd  = f.payload.data.cmdId;

    if(myId == MASTER)
    {
        if(type == MsgType::PONG)pongReceived = true;
        else if(type == MsgType::ACK && f.payload.action_ack.status == 0) ackReceived = true;
    }
    else if(myId == SLAVE)
    {
        if (type == MsgType::PING)
        {
            Frame pong;
            this->buildHeader(pong,MASTER,PONG,0);
            this->sendFrame(pong); 
            return;
        }
        else if(type == MsgType::DATA)
    {
        if(f.payload.data.direction == 0) // CMD
        {
            Frame ackFrame;
            buildHeader(ackFrame, MASTER, MsgType::ACK, 2);
            ackFrame.payload.action_ack.cmdId = f.payload.data.cmdId;
            ackFrame.payload.action_ack.status = 0;   // OK
            sendFrame(ackFrame);

            // === NOVO ===
            hasNewCommand = true;
            latestCmd = f.payload.data.cmdId;   // precisas de declarar latestCmd
            return;
        }
    }
    }

}

// Inside AppLayer.cpp
void AppLayer::sendErrorAck(uint8_t cmdID) {
    Frame reply;
    buildHeader(reply, MASTER, MsgType::ACK, 2);
    reply.payload.action_ack.cmdId = cmdID;
    reply.payload.action_ack.status = 1; 
    //transmitReply(reply);
}
void AppLayer::buildHeader(Frame& f,NodeId dest, uint8_t type, uint8_t payloadLen) {
    f.sof = SOF_VALUE;
    f.header.src = this->myId; // Automatically set to our ID
    f.header.dest = dest;
    f.header.type = type;
    f.header.len = payloadLen;
}

bool AppLayer::hasReceivedPong()
{
    return pongReceived;
}
bool AppLayer::hasReceivedAck()
{
    return ackReceived;
}