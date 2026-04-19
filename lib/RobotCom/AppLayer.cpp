#include "AppLayer.h"


void AppLayer::init(Stream *port, NodeId id)
{
    this->myId = id;
    this->ddl.init(port);
    // Reset all reception flags
    hasNewCommandReceived = false;
    receivedCmdId         = 0;
    receivedDirection     = 0;
    receivedParamLen      = 0;
    pongReceived          = false;
    ackReceived           = false;

}

void AppLayer::sendFrame(const Frame& f)
{
    this->ddl.write(f);
}


void AppLayer::sendPing(uint8_t dst) 
{
    this->pongReceived = false; 
    this->pingTo_id = dst;
    this->lastCommandSent = MsgType::PING;
    Frame pingFrame;
    buildHeader(pingFrame, dst, MsgType::PING, 0); 
    sendFrame(pingFrame);
}


void AppLayer::sendCommand(uint8_t dst, uint8_t cmdId)
{
    sendCommandWithData(dst, cmdId, nullptr, 0);
}

void AppLayer::sendCommandWithData(uint8_t dst, uint8_t cmdId, const uint8_t* data, uint8_t dataLen)
{
    this->lastCommandSent = cmdId;
    this->ackReceived = false;

    uint8_t direction = (cmdId >= 20) ? 1 : 0;
    uint8_t totalLen = 2 + dataLen;
    if (totalLen > MAX_PAYLOAD) totalLen = MAX_PAYLOAD;

    Frame cmdFrame;
    buildHeader(cmdFrame, dst, MsgType::DATA, totalLen);

    cmdFrame.payload.data.direction = direction;
    cmdFrame.payload.data.cmdId     = cmdId;

    if (dataLen > 0 && data != nullptr)
        memcpy(cmdFrame.payload.data.param, data, dataLen);

    sendFrame(cmdFrame);
}

// ================================================================
//  FRAME PROCESSING
// ================================================================
bool AppLayer::update()
{
    this->ddl.update();

    if(this->ddl.hasNewFrame())
    {
        Frame incomingFrame;
        this->ddl.getFrame(incomingFrame);
        if(incomingFrame.header.dest == myId) this->processFrame(incomingFrame);
        
        return true;
    }
    return false;
}

void AppLayer::processFrame(const Frame& f) 
{ 
    uint8_t type = f.header.type;

    if(myId == MASTER)
    {
        if(type == MsgType::PONG){
            Serial.printf("\n[MASTER APP] PONG ARRIVED! Frame SRC: %d | I am waiting for: %d\n", f.header.src, this->pingTo_id);
            if(f.header.src == this->pingTo_id)
            {
                pongReceived = true;
                Serial.println("[MASTER APP] IDs Match! pongReceived = true");
                this->pingTo_id = -1;//Reset 
            }
            else Serial.println("[MASTER APP] ERROR: IDs DO NOT MATCH! Ignoring PONG.");
        } 
        
        else if(type == MsgType::ACK && f.payload.action_ack.status == 0)
        {
            cmdACK = f.payload.action_ack.cmdId;
            ackReceived = true;
        }
        
        else if(type == MsgType::DATA && f.payload.data.direction == 1)
        {
            hasNewCommandReceived = true;
            receivedCmdId         = f.payload.data.cmdId;
            receivedDirection     = 1;
            receivedParamLen      = (f.header.len > 2) ? f.header.len - 2 : 0;
            if (receivedParamLen > 0)
                memcpy(receivedParams, f.payload.data.param, receivedParamLen);
        }
    }
    else
    {
        if (type == MsgType::PING)
        {

            Serial.printf("\n[SLAVE APP] Heard a PING! Dest ID: %d | My ID: %d\n", f.header.dest, this->myId);
            Serial.println("[SLAVE APP] The PING is for me! Sending PONG back...");
            slave_ReceivedPing = true;
            Frame pong;
            buildHeader(pong, f.header.src, MsgType::PONG, 0);
            delay(40);//Give time to hardware
            sendFrame(pong);
            return;
        }
        else if(type == MsgType::DATA)
        {
            if(f.payload.data.direction == 0) // CMD from master
            {
                // Send ACK 
                Frame ackFrame;
                buildHeader(ackFrame, f.header.src, MsgType::ACK, 2);//I can easilly respond to who sent the frame! f.header.src is the dest!
                ackFrame.payload.action_ack.cmdId = f.payload.data.cmdId;
                ackFrame.payload.action_ack.status = 0;
                delay(40);
                sendFrame(ackFrame);

                // === STORE THE COMMAND + PARAMETERS ===
                hasNewCommandReceived = true;
                receivedCmdId         = f.payload.data.cmdId;
                receivedDirection     = 0;
                receivedParamLen      = (f.header.len > 2) ? f.header.len - 2 : 0;
                if (receivedParamLen > 0)
                    memcpy(receivedParams, f.payload.data.param, receivedParamLen);
                return;
            }
            else if(f.payload.data.direction == 1)
            {
                //Put logic here to respond! where it is! 
                
            }

        }
    }
}
// ================================================================
//  HELPERS
// ================================================================


void AppLayer::buildHeader(Frame& f, uint8_t dest, uint8_t type, uint8_t payloadLen) {
    f.sof = SOF_VALUE;
    f.header.src = this->myId; // Automatically set to our ID
    f.header.dest = dest;
    f.header.type = type;
    f.header.len = payloadLen;
}
