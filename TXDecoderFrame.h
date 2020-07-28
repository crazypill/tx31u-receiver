//
//  TXDecoderFrame.h
//  
//
//  Created by Alex Lelievre on 7/17/20.
//

#include <stdint.h>

enum
{
    kDataFlag_temp       = 1 << 0,
    kDataFlag_humidity   = 1 << 1,
    kDataFlag_rain       = 1 << 2,
    kDataFlag_wind       = 1 << 3,
    kDataFlag_gust       = 1 << 4,
    kDataFlag_intTemp    = 1 << 5,
    kDataFlag_pressure   = 1 << 6,
    kDataFlag_airQuality = 1 << 7,
    
    kDataFlag_allMask    = 0xFF
};


typedef struct
{
    uint8_t   magic;
    uint8_t   station_id;
    uint8_t   flags;         // the flags tell you which fields are valid in this message
    uint8_t   humidity;      // 0 - 100
    float     tempC;         // temp from weather sensor in celsius
    float     intTempC;      // internal temp in celsius -- this is the internal case temp, not air temp.
    float     pressure;      // internal pressure in millibars
    float     windDirection; // in degrees
    float     windSpeedMs;   // meters/sec
    float     windGustMs;    // meters/sec
    float     rain;

    uint16_t pm10_standard;       // Standard PM1.0
    uint16_t pm25_standard;       // Standard PM2.5
    uint16_t pm100_standard;      // Standard PM10.0
    uint16_t pm10_env;            // Environmental PM1.0
    uint16_t pm25_env;            // Environmental PM2.5
    uint16_t pm100_env;           // Environmental PM10.0
    uint16_t particles_03um;      // 0.3um Particle Count
    uint16_t particles_05um;      // 0.5um Particle Count
    uint16_t particles_10um;      // 1.0um Particle Count
    uint16_t particles_25um;      // 2.5um Particle Count
    uint16_t particles_50um;      // 5.0um Particle Count
    uint16_t particles_100um;     // 10.0um Particle Count

    // remove these !!@
    uint8_t   CRC;           // just used as part of validation, really doesn't need to be here at all
    uint8_t   frameLength;   // ditto with this guy
} __attribute__ ((__packed__)) Frame;

const uint8_t FRAME_LENGTH = 13;
