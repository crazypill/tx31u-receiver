#ifndef _H_TXDecoder
#define _H_TXDecoder

#include "Arduino.h"



typedef enum
{
    kDataFlag_Temp     = 1 << 0,
    kDataFlag_Humidity = 1 << 1,
    kDataFlag_Rain     = 1 << 2,
    kDataFlag_Wind     = 1 << 3,
    kDataFlag_Gust     = 1 << 4
} DataFlags;


typedef struct
{
    uint8_t   magic;
    uint8_t   station_id;
    uint8_t   numSets;
    DataFlags flags;
    float     tempC;        // celcius
    uint8_t   humidity;     // 0 - 100
    float     windSpeedMs;  // meters/sec
    float     windGustMs;   // meters/sec
    float     rain;
    float     windDirection;
    const char* windBearing;
    uint8_t   CRC;          // will be zero if did not match
    uint8_t   frameLength;
} Frame;

const uint8_t FRAME_LENGTH = 13;

uint8_t CalculateCRC( uint8_t data[], uint8_t frameLength = FRAME_LENGTH );
uint8_t DecodeFrame( uint8_t* data, Frame* frame);
uint8_t DisplayFrame( uint8_t* data, Frame* frame );
bool    AnalyzeFrame( uint8_t* data );

#endif  // !_H_TXDecoder

// EOF
