#ifndef _H_TXDecoder
#define _H_TXDecoder

#include "Arduino.h"



enum
{
    kDataFlag_temp     = 1 << 0,
    kDataFlag_humidity = 1 << 1,
    kDataFlag_rain     = 1 << 2,
    kDataFlag_wind     = 1 << 3,
    kDataFlag_gust     = 1 << 4
};


typedef struct
{
    uint8_t   magic;
    uint8_t   station_id;
    uint8_t   flags;         // the flags tell you which fields are valid in this message
    uint8_t   humidity;      // 0 - 100
    float     tempC;         // celsius
    float     windSpeedMs;   // meters/sec
    float     windGustMs;    // meters/sec
    float     rain;
    float     windDirection; // in degrees
    uint8_t   CRC;           // will be zero if did not match
    uint8_t   frameLength;
} __attribute__ ((__packed__)) Frame;

const uint8_t FRAME_LENGTH = 13;

uint8_t CalculateCRC( uint8_t data[], uint8_t frameLength = FRAME_LENGTH );
uint8_t DecodeFrame( uint8_t* data, Frame* frame);
bool    AnalyzeFrame( uint8_t* data );

#endif  // !_H_TXDecoder

// EOF
