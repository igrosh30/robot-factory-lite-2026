#include "ComProtocol.h"

void FrameParser ::init()
{
    currentState = STATE_WAIT_FOR_SOF;
    countBytesH     = 0;
    countBytesP     = 0;
    countBytesCRC   = 0;
    newFrameReady   = false;
}

void FrameParser :: processIncomingByte(uint8_t byte)
{
    switch (currentState)
    {
    case STATE_WAIT_FOR_SOF://Faz sentido verificar se recebo duas flags?
        
        countBytesP = 0;    //reset variables
        countBytesH = 0;    //reset variables
        countBytesCRC = 0;
        
        newFrameReady = false;
        if(byte == SOF_VALUE)
        {
            frame.sof = byte;
            currentState = STATE_READ_HEADER;            
        }
        break;

    case STATE_READ_HEADER:

        switch (countBytesH)
        {
            
            case 0: frame.header.src = byte; break;
            case 1: frame.header.dest = byte;break;
            case 2: frame.header.type = byte;break;
            case 3: 
                if(byte > MAX_PAYLOAD)
                {//NOTE: if sent a higher len! 
                    currentState = STATE_WAIT_FOR_SOF;
                    return;
                }
                frame.header.len = byte;
                if(byte == 0)
                {
                    currentState = STATE_READ_CRC;//Don't need to read the PayLoad message! 
                }
                else{
                    currentState = STATE_READ_PAYLOAD;
                }
                break;
        }
        countBytesH++;
        break;
    
    case STATE_READ_PAYLOAD:
        if(countBytesP < frame.header.len)
        {
            frame.payload.raw[countBytesP++] = byte;
            if(countBytesP == frame.header.len)//stored all the values
            {   
                currentState = STATE_READ_CRC;
            }
        }
        break;
    
    case STATE_READ_CRC:
        //implement the function that will perform the verification....
        //First I need to read the src bytes sent...
        if(countBytesCRC == 0)
        {
            countBytesCRC++;
            frame.crc = (uint16_t)byte;
        }
        else if(countBytesCRC == 1)
        {
            frame.crc |= (uint16_t)byte << 8 ;// first byte recieved = high byte! 
            uint16_t calculated = calculateCRC16((const uint8_t*)&frame.header, 4 + frame.header.len);
            if (calculated == frame.crc)
            {
                newFrameReady = true;
            }
            currentState = STATE_WAIT_FOR_SOF;//after processing the 2 bytes for CRC...
        }
        break;
    }
}

uint16_t FrameParser::calculateCRC16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0x0000;                // common init for CCITT
    const uint16_t poly = 0x1021;         // CRC-16-CCITT polynomial
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;    // XOR next byte into high 8 bits
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
