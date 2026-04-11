#ifndef DATALINK_H
#define DATALINK_H

#include "ComProtocol.h"
#include <Arduino.h>



class DataLink {
private:
    Stream* serialPort;      

public:
    
    FrameParser parser;
    const int MAX_BYTES_PER_CYCLE = 32;
    
    DataLink();
    void init(Stream* port);
    void write(const Frame& f);
    void update();
    bool hasNewFrame() const;
    void getFrame(Frame& outFrame) ;

    void reset();
};

#endif