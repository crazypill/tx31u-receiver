//
//  TXDecoderFrame.h
//  
//
//  Created by Alex Lelievre on 7/17/20.
//

#include <stdint.h>

enum
{
    kDataFlag_temp     = 1 << 0,
    kDataFlag_humidity = 1 << 1,
    kDataFlag_rain     = 1 << 2,
    kDataFlag_wind     = 1 << 3,
    kDataFlag_gust     = 1 << 4,
    kDataFlag_intTemp  = 1 << 5,
    kDataFlag_pressure = 1 << 6
};


typedef struct
{
    uint8_t   magic;
    uint8_t   station_id;
    uint8_t   flags;         // the flags tell you which fields are valid in this message
    uint8_t   humidity;      // 0 - 100
    float     tempC;         // temp from weather sensor in celsius
    float     intTempC;      // internal temp in celsius
    float     pressure;      // internal pressure in millibars
    float     windSpeedMs;   // meters/sec
    float     windGustMs;    // meters/sec
    float     rain;
    float     windDirection; // in degrees
    uint8_t   CRC;           // just used as part of validation, really doesn't need to be here at all
    uint8_t   frameLength;   // ditto with this guy
} __attribute__ ((__packed__)) Frame;

const uint8_t FRAME_LENGTH = 13;
