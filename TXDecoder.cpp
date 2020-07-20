//
//  TXDecoder.c
//  
//
//  Created by Alex Lelievre on 7/16/20.
//

#include "TXDecoder.h"
#include <Adafruit_BMP085.h>


/*
http://www.g-romahn.de/ws1600/Datepakete_raw.txt

Data - organized in nibbles - are structured as follows (exammple with blanks added for clarity):

 a 5a 5 0 628 1 033 2 000 3 e00 4 000 bd

 data always start with "a"
 from next 1.5 nibbles (here 5a) the 6 msb are identifier of transmitter,
 bit 1 indicates acquisition/synchronizing phase (so 5a >> 58 thereafter)
 bit 0 will be 1 in case of error (e.g. no wind sensor 5a >> 5b)
 next nibble (here 5) is count of quartets to betransmitted
 up to 5 quartets of data follow
 each quartet starts with type indicator (here 0,1,2,3,4)
 0: temperature, 3 nibbles bcd coded tenth of °c plus 400 (here 628-400 = 22.8°C)
 1: humidity, 3 nibbles bcd coded (here 33 %rH), meaning of 1st nibble still unclear
 2: rain, 3 nibbles, counter of contact closures
 3: wind, first nibble direction of wind vane (multiply by 22.5 to obtain degrees,
    here 0xe*22.5 = 315 degrees)
    next two nibbles wind speed in m per sec (i.e. no more than 255 m/s; 9th bit still not found)
 4: gust, speed in m per sec (yes, TX23 sensor does measure gusts and data are transmitted
    but not displayed by WS1600), number of significant nibbles still unclear
 next two bytes (here bd) are crc.
 During acquisition/synchronizing phase (abt. 5 hours) all 5 quartets are sent, see examplke above. Thereafter
 data strings contain only a few ( 1 up ton 3) quartets, so data strings are not! always of
 equal length.

After powering on, the complete set of data will be transmitted every 4.5 secs for 5 hours during acquisition phase.
Lateron only selected sets of data will be transmitted.

Stream of received data follows:

1st line: raw data in hex format as received from sensors
2nd line: meteorological data from outdoor sensors decoded, Same values
      are displayed on basestation (last duet is "calculated crc" - always 00).

a1250444109120003808401b89 00
Temp  044 Humi 91 Rain 000 Wind 028  Dir 180 Gust 097  ( 4.4 °C, 91 %rH, no rain, wind 2.8 km/h from south, gust 9.7 km/h)
*/


//#define PRINT_BAD_DATA
//#define PRINT_PAYLOAD
//#define DEBUG_PRINT




#ifdef DEBUG_PRINT
// debug print
#define DebugPrint(x) DebugPrint_P(PSTR(x))
void SerialWrite ( uint8_t c ) {
    Serial.write ( c );
}

void DebugPrint_P(PGM_P str, void (*f)(uint8_t) = SerialWrite ) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) (*f)(c);
}
#else
#define DebugPrint(x)
#endif





enum
{
    kType_temp,
    kType_humidity,
    kType_rain,
    kType_wind,
    kType_gust
};



Adafruit_BMP085 bmp;



#pragma mark -


uint8_t UpdateCRC(uint8_t res, uint8_t val) {
    for (int i = 0; i < 8; i++) {
      uint8_t tmp = (uint8_t)((res ^ val) & 0x80);
      res <<= 1;
      if (0 != tmp) {
        res ^= 0x31;
      }
      val <<= 1;
    }
  return res;
}


uint8_t CalculateCRC(uint8_t *data, uint8_t len) {
  uint8_t res = 0;
  for (int j = 0; j < len; j++) {
    uint8_t val = data[j];
    res = UpdateCRC(res, val);
  }
  return res;
}


uint8_t reverseBits( uint8_t num )
{
    unsigned int count = sizeof(num) * 8 - 1;
    unsigned int reverse_num = num;
      
    num >>= 1;
    while(num)
    {
       reverse_num <<= 1;
       reverse_num |= num & 1;
       num >>= 1;
       count--;
    }
    reverse_num <<= count;
    return reverse_num;
}



#pragma mark -


void TxDecoderInit()
{
    if( !bmp.begin() )
    {
        DebugPrint( "Could not find a valid BMP085 sensor, check wiring!\n" );
        while (1) {}
    }
}


uint8_t DecodeFrame( uint8_t* bytes, Frame* frame )
{
    // look at the buffer and see if we can process it (before doing CRC on it) - the first nibble is the
    uint8_t quartets = bytes[1] & 0xF;
    frame->magic = (bytes[0] & 0xF0) >> 4;
    frame->frameLength = quartets * 2 + 2 + 1;
    frame->CRC = bytes[frame->frameLength - 1];
    frame->flags = 0;

    if( frame->CRC != CalculateCRC( bytes, frame->frameLength - 1 ) )
    {
#ifdef PRINT_BAD_DATA
        Serial.print( "DecodeFrame: bad CRC..." );
        Serial.print( "data: " );
        for( int i = 0; i < frame->frameLength; i++ )
            Serial.print( bytes[i], HEX );
        Serial.println();
#endif
        return 0;
    }
    
#ifdef PRINT_PAYLOAD
    Serial.print( "Payload: " );
    for( int i = 0; i < frame->frameLength; i++ )
        Serial.print( bytes[i], HEX );
    Serial.println();
#endif
    
    // split the data into a nibble stream
    uint16_t nibbleCount = frame->frameLength * 2;
    uint8_t* nibbles = (uint8_t*)malloc( nibbleCount );
    if( !nibbles )
        return 0;

    for( int i = 0; i < nibbleCount; i++ )
    {
        if( i & 1 )
            nibbles[i] = bytes[i/2] & 0xF;
        else
            nibbles[i] = (bytes[i/2] & 0xF0) >> 4;
    }
    
    frame->station_id = nibbles[1] << 4 | (nibbles[2] & 0xC);   // did I assemble this backwards?  strip out the error and aquire bits
    // !!@ don't forget to include error and acquiring bits in frame struct
    bool errorBit = nibbles[2] & 0x1;   // this means the wind data is not valid...  (it could be more, but so far)
    
#ifdef DEBUG_PRINT
    Serial.print( "Weather station id: " );
    Serial.print( frame->station_id, HEX );
    Serial.print( " error: " );
    Serial.print( nibbles[2] & 0x1 );
    Serial.print( " aquire: " );
    Serial.print( nibbles[2] & 0x2 );
    Serial.print( " quartets: " );
    Serial.println( quartets );
#endif
    
#ifdef DEBUG_PRINT
    char textBuffer[128];
    const char* compass[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };
    const char* types[]   = { "temp:    ", "humidity:", "rain:    ", "wind:    ", "gust:    " };
#endif
    
    if( nibbles[0] == 0xA )
    {
        unsigned char* q = &nibbles[4];
        for( int i = 0; i < quartets; i++ )
        {
            int type = q[0];

#ifdef DEBUG_PRINT
            const char* typeStr = NULL;
            if( type >= 0 && type <= kType_gust )
                typeStr = types[type];
            else
                typeStr = "unknown: ";

            DebugPrint( typeStr );
            DebugPrint( " " );
#endif
            
            if( type == kType_temp )
            {
                float t = 0;
                t += q[1] * 100.0;
                t += q[2] * 10.0;
                t += q[3] * 1.0;
                t = t / 10;
                t -= 40;
                frame->tempC = t;
                frame->flags |= kDataFlag_temp;

#ifdef DEBUG_PRINT
                sprintf( textBuffer, "%0.2f°F, %0.1f°C", c2f(t), t );
                DebugPrint( textBuffer );
                DebugPrint( "\n"  );
#endif
            }
            else if( type == kType_rain )
            {
                frame->rain = 0;    // !!@ not implemented yet
                frame->flags |= kDataFlag_rain;

#ifdef DEBUG_PRINT
                sprintf( textBuffer, "%u %u %u", q[1], q[2], q[3] );
                DebugPrint( textBuffer );
                DebugPrint( "\n"  );
#endif
            }
            else if( type == kType_humidity )
            {
                uint8_t h = 0;
                h += q[1] * 100;
                h += q[2] * 10;
                h += q[3] * 1;
                frame->humidity = h;
                frame->flags |= kDataFlag_humidity;

#ifdef DEBUG_PRINT
                sprintf( textBuffer, "%d%%", h );
                DebugPrint( textBuffer );
                DebugPrint( "\n"  );
#endif
            }
            else if( type == kType_wind && !errorBit )
            {
                // first nibble direction of wind vane (multiply by 22.5 to obtain degrees, here 0xe*22.5 = 315 degrees)
                // next two nibbles wind speed in m per sec (i.e. no more than 255 m/s; 9th bit still not found)
                int         windspeed = (q[2] << 4) | q[3];
                float       speed     = windspeed / 10.0;
                frame->windSpeedMs = speed;
                frame->windDirection = q[1] * 22.5f;
                frame->flags |= kDataFlag_wind;

#ifdef DEBUG_PRINT
                const char* direction = compass[q[1]];
                sprintf( textBuffer, "%0.1f°, %0.2f mph, %0.1f m/s %s", q[1] * 22.5f, ms2mph( speed ), speed, direction );
                DebugPrint( textBuffer );
                DebugPrint( "\n"  );
#endif
            }
            else if( type == kType_gust )
            {
                // gust speed in m per sec - !!@ check this
                int windgust = q[1] * 256 + q[2];
                frame->windGustMs = windgust;
                frame->flags |= kDataFlag_gust;

#ifdef DEBUG_PRINT
                sprintf( textBuffer, "%d m/s", windgust );
                DebugPrint( textBuffer );
                DebugPrint( "\n"  );
#endif
            }
#ifdef DEBUG_PRINT

            else
                DebugPrint( "\n"  );
#endif

            // also shove the internal temp and pressure in there
            frame->intTempC = bmp.readTemperature();
            frame->flags |= kDataFlag_intTemp;

            frame->pressure = (bmp.readPressure() * pascal2millibar);
            frame->flags |= kDataFlag_pressure;

            // advance to next quartet
            q += 4;
        }
    }

    free( nibbles );
    
    return frame->frameLength;
}


bool AnalyzeFrame( uint8_t* data )
{
    Frame frame = {};
    if( DecodeFrame( data, &frame ) )
    {
        Serial1.write( (uint8_t*)&frame, sizeof( Frame ) );
        return true;
    }
    return false;
}


// EOF
