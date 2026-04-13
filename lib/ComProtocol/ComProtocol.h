#ifndef COMPROTOCOL_H
#define COMPROTOCOL_H

#include "Arduino.h"

// 1. Tell the compiler to pack structs tightly without padding!
#pragma pack(push, 1)

const uint8_t SOF_VALUE     = 0x7E;
const uint8_t MAX_PAYLOAD   = 12; 

/*

SOF = 0x7E

HEADER (4 bytes):
  SRC  : 1 byte (node id)
  DEST : 1 byte (node id)
  TYPE : 1 byte (command)
  LEN  : 1 byte (payload size)

PAYLOAD:
  LEN bytes

CRC:
  2 bytes (CRC16 of HEADER+PAYLOAD)
  */

//note that this will need to be only 1byte! 
enum NodeId //change to be for now 
{
    MASTER = 0,
    SLAVE = 1
};

enum MsgType//maybe we can change the DATA type...
{
    PING        = 1,
    PONG        = 2,
    DATA        = 3,    //SEND & ASK - direction in payload specifies 
    ACK         = 4     //Processed correctly your message!
};

struct Header
{
    uint8_t src;
    uint8_t dest;
    uint8_t type;
    uint8_t len;
};

union PayloadData//Size of a union is always the size of it's biggest member!
{
    uint8_t raw[MAX_PAYLOAD];

    struct {
        uint8_t direction;  //Byte 0   : 0-> CMD / 1-> ASK
        uint8_t cmdId;      //Byte 1   : corresponding enum

        uint8_t param[MAX_PAYLOAD-2];     //Byte 2-7 : bytes for data
    }data;

    struct { 
        uint8_t cmdId;     // Which command are we acknowledging?
        uint8_t status;    // 0 = OK, 1 = Error
    } action_ack;
    
    // Named views for common replies (same memory!)
    struct {  uint8_t cmdId; uint8_t battery_V;} battery_reply; 
    struct { uint16_t depth_mm; } pop_up_params;
};

//CMD_ID is what we change based on our application!
enum CMD_ID : uint8_t
{
    // --- MASTER TO SLAVE COMMANDS (Actions) ---
    CMD_WAIT                  = 1,   // Tell slave to hold position
    CMD_GO_PROCESS_MACHINE    = 2,   // Tell slave to navigate to the process machine (Your GO_PICK_GREEN)
    INFO_GREEN_BOX            = 3,
    INFO_BLUE_BOX             = 4,
    INFO_BLUE_PICK_SLOT       = 5,
    CMD_EXECUTE_PICK_GREEN    = 6,   // Tell slave the box is ready, pick it and deliver (Your PICK_GREEN)
    CMD_EXECUTE_PICK_BLUE     = 7,   // Tell slave to go handle a blue box (Your PICK_BLUE)
    CMD_GO_RETRIEVE_ZONE      = 8,   // Tell slave to go to the final retrieve zone
    

    // --- SLAVE TO MASTER STATUS only when asked! - direction 1
    STATUS_IN_POSITION        = 20,  // Slave tells Master: "I am at the machine waiting"
    STATUS_GREEN_DELIVERED    = 21,  // Slave tells Master: "I dropped the green box at the warehouse" (Your WAS_PICKED_GREEN)
    STATUS_BLUE_DELIVERED     = 22,  // Slave tells Master: "I dropped the blue box"
    STATUS_ERROR              = 99,  // Slave tells Master: "I failed / dropped the box"

    // --- SYSTEM COMMANDS ---
    CMD_END_MISSION           = 100
};

//--------PARSER LOGIC-------------//
struct Frame 
{
    uint8_t sof;                    // Always 0x7E
    Header header;                  // Your 4-byte header
    PayloadData payload;        // Maximum allowed payload
    uint16_t crc;                   // 2-byte error check
};

enum RxState
{
    STATE_WAIT_FOR_SOF, // Waiting to see 0x7E
    STATE_READ_HEADER,  // Reading the next 4 bytes
    STATE_READ_PAYLOAD, // Reading 'len' amount of bytes
    STATE_READ_CRC      // Reading the 2 error-check bytes
};
class FrameParser //reception tool for each Station
{
private:

public:
    RxState currentState;
    uint8_t countBytesH;//count header bytes to process
    uint8_t countBytesP;//count payload bytes to process
    uint8_t countBytesCRC;
    Frame frame;
    bool newFrameReady;
    
    void init();
    //void printDebug();

    void processIncomingByte(uint8_t byte);
    uint16_t calculateCRC16(const uint8_t* data, uint8_t len);

};


#pragma pack(pop)
#endif