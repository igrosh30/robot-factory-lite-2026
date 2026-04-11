#include "dataLink.h"
#include <string.h>

// Default constructor - does nothing
DataLink::DataLink() 
    : serialPort(nullptr), 
      parser() 
{
 
}

void DataLink::init(Stream* port)
{
    if(port != nullptr)
    {
        this->serialPort = port;
        this->parser.init();
    }
}

void DataLink::write(const Frame& f) {

    uint8_t txBuffer[32]; 
    uint8_t index = 0;

    txBuffer[index++] = f.sof;

    memcpy(&txBuffer[index], &f.header, 4);
    index += 4;

    if (f.header.len > 0) 
    {
        memcpy(&txBuffer[index], f.payload.raw, f.header.len);
        index += f.header.len;
    }

    //CRC calculation to validade the information on the other side!
    uint16_t outCrc = parser.calculateCRC16((const uint8_t*)&f.header, 4 + f.header.len);
    memcpy(&txBuffer[index], &outCrc, 2);
    index += 2;

    
    serialPort->write(txBuffer, index);

    

    serialPort->flush();
}

void DataLink::update() {//Read bytes available
    uint8_t bytesCount = 0;
    while(serialPort->available() && bytesCount < MAX_BYTES_PER_CYCLE) //16 bytes Serial.available() && 
    {
        uint8_t incomingByte = (uint8_t)serialPort->read();   
        parser.processIncomingByte(incomingByte);//Update the State!
        bytesCount++;
    }
}

bool DataLink::hasNewFrame() const {
    return parser.newFrameReady;
}

void DataLink::getFrame(Frame& outFrame) {
    parser.newFrameReady = false;//IMPORTANT to RESET the flag! we got the frame 
    memcpy(&outFrame,&parser.frame,sizeof(Frame));
}

void DataLink::reset() {
    parser.init();
}