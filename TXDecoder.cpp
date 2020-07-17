//
//  TXDecoder.c
//  
//
//  Created by Alex Lelievre on 7/16/20.
//

#include "TXDecoder.h"

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




#define c2f( a ) (((a) * 1.8000) + 32)
#define ms2mph( a ) ((a) * 2.23694)
#define km2mph( a ) ((a) / 0.621371)



enum
{
    kType_temp,
    kType_humidity,
    kType_rain,
    kType_wind,
    kType_gust
};




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


uint8_t DecodeFrame( uint8_t* bytes, Frame* frame )
{
    // look at the buffer and see if we can process it (before doing CRC on it) - the first nibble is the
    uint8_t quartets = bytes[1] & 0xF;
    frame->magic = (bytes[0] & 0xF0) >> 4;
    frame->frameLength = quartets * 2 + 2 + 1;
    frame->CRC = bytes[frame->frameLength - 1];

//    Serial.print( "\nFrame magic: " );
//    Serial.println( frame->magic, HEX );
//
//    Serial.print( "\nFrame length: " );
//    Serial.println( frame->frameLength );
//
//    Serial.print( "\nFrame CRC: " );
//    Serial.println( frame->CRC, HEX );
    
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

//    Serial.print( "\nHibbles: " );
    for( int i = 0; i < nibbleCount; i++ )
    {
        if( i & 1 )
            nibbles[i] = bytes[i/2] & 0xF;
        else
            nibbles[i] = (bytes[i/2] & 0xF0) >> 4;

//        Serial.print( nibbles[i], HEX );
    }
//    Serial.println();

    
    uint8_t station_id = nibbles[1] << 4 | (nibbles[2] & 0xC);   // did I assemble this backwards?  strip out the error and aquire bits
    Serial.print( "Weather station id: " );
    Serial.print( station_id, HEX );
    Serial.print( " error: " );
    Serial.print( nibbles[2] & 0x1 );
    Serial.print( " aquire: " );
    Serial.print( nibbles[2] & 0x2 );
    Serial.print( " quartets: " );
    Serial.println( quartets );

    char textBuffer[128];
    const char* compass[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };
    const char* types[]   = { "temp:    ", "humidity:", "rain:    ", "wind:    ", "gust:    " };
    if( nibbles[0] == 0xA )
    {
        unsigned char* q = &nibbles[4];
        for( int i = 0; i < quartets; i++ )
        {
            int type = q[0];
            const char* typeStr = NULL;
            if( type >= 0 && type <= kType_gust )
                typeStr = types[type];
            else
                typeStr = "unknown: ";

            Serial.print( "Quartet: " );
            Serial.print( typeStr );
            Serial.print( " " );

            if( type == kType_temp )
            {
                float t = 0;
                t += q[1] * 100.0;
                t += q[2] * 10.0;
                t += q[3] * 1.0;
                t = t / 10;
                t -= 40;
                
//                sprintf( textBuffer, "%0.2f°F, %0.2f°C", c2f(t), t );
                sprintf( textBuffer, "%0.1f°F, %0.1f°C", c2f(t), t );
                Serial.println( textBuffer );
            }
            else if( type == kType_rain )
            {
                sprintf( textBuffer, "%u %u %u", q[1], q[2], q[3] );
                Serial.println( textBuffer );
            }
            else if( type == kType_humidity )
            {
                float h = 0;
                h += q[1] * 100.0;
                h += q[2] * 10.0;
                h += q[3] * 1.0;
                sprintf( textBuffer, "%g%%", h );
                Serial.println( textBuffer );
            }
            else if( type == kType_wind )
            {
                // first nibble direction of wind vane (multiply by 22.5 to obtain degrees, here 0xe*22.5 = 315 degrees)
                // next two nibbles wind speed in m per sec (i.e. no more than 255 m/s; 9th bit still not found)
                const char* direction = compass[q[1]];
                int windspeed = (q[2] << 4) | q[3];
                float speed = windspeed / 10.0;
                sprintf( textBuffer, "%0.1f°, %0.2f mph, %0.1f m/s %s", q[1] * 22.5f, ms2mph( speed ), speed, direction );
                Serial.println( textBuffer );
            }
            else if( type == kType_gust )
            {
                // gust speed in m per sec - !!@ check this
                int windgust = q[1] * 256 + q[2];
                sprintf( textBuffer, "%d m/s", windgust );
                Serial.println( textBuffer );
            }
            else
                Serial.println();

            q += 4;
        }
    }

    free( nibbles );
    
    return frame->frameLength;
    
    
    
//
//
//    uint8_t* sbuf = bytes;
//    uint8_t dataSets = sbuf[1] & 0xF;
//  frame->IsValid = true;
//  frame->Header = (bytes[0] & 0xF0) >> 4;
//  frame->frameLength = dataSets * 2 + 2 + 1;
//
//  frame->CRC = bytes[frame->frameLength-1];
//  if (frame->CRC != CalculateCRC(bytes, frame->frameLength)) {
//    frame->IsValid = false;
//  }
//  if (frame->Header != 0xA) {
//    frame->IsValid = false;
//  }
//  if (!frame->IsValid) {
//      return 0;
//  }
//    static const char *compass[] = {"N  ", "NNE", "NE ", "ENE", "E  ", "ESE", "SE ", "SSE", "S  ", "SSW", "SW ", "WSW", "W  ", "WNW", "NW ", "NNW"};
//    static const char *sensors[] = {"Temp", "Hum ", "Rain", "Wind", "Gust"};
//    uint8_t windbearing = 0;
//    // station id
//    uint8_t stationid = ((sbuf[0] & 0x0F) << 6) | ((sbuf[1] & 0xC0) >>6);
//    int8_t temp = 0;
//    int8_t tempDeci = 0;
//    //humidity
//    uint8_t humidity = 0;
//    //wind speed
//    uint8_t windspeed = 0;
//    //wind gust
//    uint8_t windgust = 0;
//    //rainfall
//    uint16_t rain = 0;
//
//    for (uint8_t i = 0; i < dataSets; i++) {
//        uint8_t j = 2 + i*2;
//        uint8_t sensorType = (sbuf[j] & 0xF0) >> 4;
//        switch (sensorType) { // e.g. a 5a 5 0 628 1 033 2 000 3 e00 4 000 bd
//            case 0:    //  0: temperature, 3 nibbles bcd coded tenth of °c plus 400 (here 628-400 = 22.8°C)
//                temp = BCD2bin(sbuf[j] & 0x0F) * 10 + BCD2bin((sbuf[j + 1] & 0xF0)>>4);
//                temp = temp;
//                tempDeci = BCD2bin((sbuf[j + 1] & 0x0F));
//                frame->SensorType[i] = sensors[sensorType];
//                frame->Temperature = ((temp * 10 + tempDeci) - 400) / 10;
//                break;
//            case 1: // 1: humidity, 3 nibbles bcd coded (here 33 %rH), meaning of 1st nibble still unclear
//                humidity = BCD2bin(sbuf[j + 1]);
//                frame->SensorType[i] = sensors[sensorType];
//                frame->Humidity = humidity;
//                break;
//            case 2: // 2: rain, 3 nibbles, counter of contact closures
//                rain = ((sbuf[j] & 0x0F) + (sbuf[j + 1]) * 100);
//                frame->SensorType[i] = sensors[sensorType];
//                frame->Rain = rain;
//                break;
//            case 3:    //3: wind, first nibble direction of wind vane (multiply by 22.5 to obtain degrees,
//                    // here 0xe*22.5 = 315 degrees)
//                    // next two nibbles wind speed in m per sec (i.e. no more than 255 m/s; 9th bit still not found)
//                windbearing = (sbuf[j] & 0x0F);
//                windspeed = (sbuf[j + 1]);
//                frame->SensorType[i] = sensors[sensorType];
//                frame->WindSpeed = windspeed;
//                frame->WindBearing = compass[windbearing];
//                break;
//            case 4: // 4: gust, speed in m per sec (yes, TX23 sensor does measure gusts and data are transmitted
//                    // but not displayed by WS1600), number of significant nibbles still unclear
//                windgust = ((sbuf[j] & 0x0F) * 256 + (sbuf[j + 1]));
//                frame->SensorType[i] = sensors[sensorType];
//                frame->WindGust = windgust;
//                break;
//            default:
//                frame->SensorType[i] = "Unkown";
//                break;
//        }
//
//    }
//  frame->ID = stationid;
//  frame->DataSets = dataSets;
//  return frame->frameLength;
}


uint8_t DisplayFrame( uint8_t* data, Frame* frame )
{
    return 0;
}


bool AnalyzeFrame( uint8_t* data )
{
    Frame frame = {};
    if( DecodeFrame( data, &frame ) )
    {
        DisplayFrame( data, &frame );
        return true;
    }
    return false;
}


// EOF
