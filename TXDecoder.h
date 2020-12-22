//
//  TXDecoder.h
//
//  Created by Alex Lelievre
//  Copyright © 2020 Far Out Labs. All rights reserved.
//

#ifndef _H_TXDecoder
#define _H_TXDecoder

#include "Arduino.h"

#include "TxDecoderFrame.h"

#define c2f( a )         (((a) * 1.8000) + 32)
#define pascal2millibar  0.01

#define ms2mph( a )      ((a) * 2.23694)
#define km2mph( a )      ((a) / 0.621371)

void     TxDecoderInit();
uint8_t  CalculateCRC( uint8_t data[], uint8_t frameLength = FRAME_LENGTH );
uint8_t  DecodeFrame( uint8_t* data, uint8_t len, Frame* frame);
bool     AnalyzeFrame( uint8_t* data, uint8_t len );

#endif  // !_H_TXDecoder

// EOF
