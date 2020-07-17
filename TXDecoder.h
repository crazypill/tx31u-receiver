#ifndef _H_TXDecoder
#define _H_TXDecoder

#include "Arduino.h"

#include "TxDecoderFrame.h"


uint8_t CalculateCRC( uint8_t data[], uint8_t frameLength = FRAME_LENGTH );
uint8_t DecodeFrame( uint8_t* data, Frame* frame);
bool    AnalyzeFrame( uint8_t* data );

#endif  // !_H_TXDecoder

// EOF
