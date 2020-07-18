// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>

#include "TXDecoder.h"


//for debugging
//#define DEBUG
//#define REGISTER_DETAIL



#ifdef REGISTER_DETAIL
static const char* s_reg_names[] =
{
    "REG_FIFO",
    "REG_OPMODE",
    "REG_DATAMODUL",
    "REG_BITRATEMSB",
    "REG_BITRATELSB",
    "REG_FDEVMSB",
    "REG_FDEVLSB",
    "REG_FRFMSB",
    "REG_FRFMID",
    "REG_FRFLSB",
    "REG_OSC1",
    "REG_AFCCTRL",
    "REG_LOWBAT",
    "REG_LISTEN1",
    "REG_LISTEN2",
    "REG_LISTEN3",
    "REG_VERSION",
    "REG_PALEVEL",
    "REG_PARAMP",
    "REG_OCP",
    "REG_AGCREF",
    "REG_AGCTHRESH1",
    "REG_AGCTHRESH2",
    "REG_AGCTHRESH3",
    "REG_LNA",
    "REG_RXBW",
    "REG_AFCBW",
    "REG_OOKPEAK",
    "REG_OOKAVG",
    "REG_OOKFIX",
    "REG_AFCFEI",
    "REG_AFCMSB",
    "REG_AFCLSB",
    "REG_FEIMSB",
    "REG_FEILSB",
    "REG_RSSICONFIG",
    "REG_RSSIVALUE",
    "REG_DIOMAPPING1",
    "REG_DIOMAPPING2",
    "REG_IRQFLAGS1",
    "REG_IRQFLAGS2",
    "REG_RSSITHRESH",
    "REG_RXTIMEOUT1",
    "REG_RXTIMEOUT2",
    "REG_PREAMBLEMSB",
    "REG_PREAMBLELSB",
    "REG_SYNCCONFIG",
    "REG_SYNCVALUE1",
    "REG_SYNCVALUE2",
    "REG_SYNCVALUE3",
    "REG_SYNCVALUE4",
    "REG_SYNCVALUE5",
    "REG_SYNCVALUE6",
    "REG_SYNCVALUE7",
    "REG_SYNCVALUE8",
    "REG_PACKETCONFIG1",
    "REG_PAYLOADLENGTH",
    "REG_NODEADRS",
    "REG_BROADCASTADRS",
    "REG_AUTOMODES",
    "REG_FIFOTHRESH",
    "REG_PACKETCONFIG2",
    "REG_AESKEY1",
    "REG_AESKEY2",
    "REG_AESKEY3",
    "REG_AESKEY4",
    "REG_AESKEY5",
    "REG_AESKEY6",
    "REG_AESKEY7",
    "REG_AESKEY8",
    "REG_AESKEY9",
    "REG_AESKEY10",
    "REG_AESKEY11",
    "REG_AESKEY12",
    "REG_AESKEY13",
    "REG_AESKEY14",
    "REG_AESKEY15",
    "REG_AESKEY16",
    "REG_TEMP1",
    "REG_TEMP2"
};

// SERIAL PRINT
// replace Serial.print("string") with SerialPrint("string")
#define SerialPrint(x) SerialPrint_P(PSTR(x))
void SerialWrite ( uint8_t c ) {
    Serial.write ( c );
}

void SerialPrint_P(PGM_P str, void (*f)(uint8_t) = SerialWrite ) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) (*f)(c);
}
#else
#define SerialPrint(x)
#endif



/************ Radio Setup ***************/

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      19
  #define RFM69_INT     18
  #define RFM69_RST     17
  #define LED           LED_BUILTIN
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  //
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif

#define RF_FDEVMSB_90000            0x05
#define RF_FDEVLSB_90000            0xC3


/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/


// RegPaLevel
#define RF_PALEVEL_PA0_ON      0x80  // Default
#define RF_PALEVEL_PA0_OFF    0x00
#define RF_PALEVEL_PA1_ON     0x40
#define RF_PALEVEL_PA1_OFF    0x00  // Default
#define RF_PALEVEL_PA2_ON     0x20
#define RF_PALEVEL_PA2_OFF    0x00  // Default

#define RF_PALEVEL_OUTPUTPOWER_00000      0x00
#define RF_PALEVEL_OUTPUTPOWER_00001      0x01
#define RF_PALEVEL_OUTPUTPOWER_00010      0x02
#define RF_PALEVEL_OUTPUTPOWER_00011      0x03
#define RF_PALEVEL_OUTPUTPOWER_00100      0x04
#define RF_PALEVEL_OUTPUTPOWER_00101      0x05
#define RF_PALEVEL_OUTPUTPOWER_00110      0x06
#define RF_PALEVEL_OUTPUTPOWER_00111      0x07
#define RF_PALEVEL_OUTPUTPOWER_01000      0x08
#define RF_PALEVEL_OUTPUTPOWER_01001      0x09
#define RF_PALEVEL_OUTPUTPOWER_01010      0x0A
#define RF_PALEVEL_OUTPUTPOWER_01011      0x0B
#define RF_PALEVEL_OUTPUTPOWER_01100      0x0C
#define RF_PALEVEL_OUTPUTPOWER_01101      0x0D
#define RF_PALEVEL_OUTPUTPOWER_01110      0x0E
#define RF_PALEVEL_OUTPUTPOWER_01111      0x0F
#define RF_PALEVEL_OUTPUTPOWER_10000      0x10
#define RF_PALEVEL_OUTPUTPOWER_10001      0x11
#define RF_PALEVEL_OUTPUTPOWER_10010      0x12
#define RF_PALEVEL_OUTPUTPOWER_10011      0x13
#define RF_PALEVEL_OUTPUTPOWER_10100      0x14
#define RF_PALEVEL_OUTPUTPOWER_10101      0x15
#define RF_PALEVEL_OUTPUTPOWER_10110      0x16
#define RF_PALEVEL_OUTPUTPOWER_10111      0x17
#define RF_PALEVEL_OUTPUTPOWER_11000      0x18
#define RF_PALEVEL_OUTPUTPOWER_11001      0x19
#define RF_PALEVEL_OUTPUTPOWER_11010      0x1A
#define RF_PALEVEL_OUTPUTPOWER_11011      0x1B
#define RF_PALEVEL_OUTPUTPOWER_11100      0x1C
#define RF_PALEVEL_OUTPUTPOWER_11101      0x1D
#define RF_PALEVEL_OUTPUTPOWER_11110      0x1E
#define RF_PALEVEL_OUTPUTPOWER_11111      0x1F  // Default



// RegOcp
#define RF_OCP_OFF                0x0F
#define RF_OCP_ON                 0x1A  // Default


// RegPacketConfig2
#define RF_PACKET2_RXRESTARTDELAY_1BIT        0x00  // Default
#define RF_PACKET2_RXRESTARTDELAY_2BITS       0x10
#define RF_PACKET2_RXRESTARTDELAY_4BITS       0x20
#define RF_PACKET2_RXRESTARTDELAY_8BITS       0x30
#define RF_PACKET2_RXRESTARTDELAY_16BITS      0x40
#define RF_PACKET2_RXRESTARTDELAY_32BITS      0x50
#define RF_PACKET2_RXRESTARTDELAY_64BITS      0x60
#define RF_PACKET2_RXRESTARTDELAY_128BITS     0x70
#define RF_PACKET2_RXRESTARTDELAY_256BITS     0x80
#define RF_PACKET2_RXRESTARTDELAY_512BITS     0x90
#define RF_PACKET2_RXRESTARTDELAY_1024BITS    0xA0
#define RF_PACKET2_RXRESTARTDELAY_2048BITS    0xB0
#define RF_PACKET2_RXRESTARTDELAY_NONE        0xC0
#define RF_PACKET2_RXRESTART                  0x04

#define RF_PACKET2_AUTORXRESTART_ON           0x02  // Default
#define RF_PACKET2_AUTORXRESTART_OFF          0x00

#define RF_PACKET2_AES_ON                     0x01
#define RF_PACKET2_AES_OFF                    0x00  // Default



// RegIrqFlags2
#define RF_IRQFLAGS2_FIFOFULL              0x80
#define RF_IRQFLAGS2_FIFONOTEMPTY         0x40
#define RF_IRQFLAGS2_FIFOLEVEL            0x20
#define RF_IRQFLAGS2_FIFOOVERRUN          0x10
#define RF_IRQFLAGS2_PACKETSENT           0x08
#define RF_IRQFLAGS2_PAYLOADREADY         0x04
#define RF_IRQFLAGS2_CRCOK                0x02
#define RF_IRQFLAGS2_LOWBAT               0x01



// RegSyncConfig
#define RF_SYNC_ON                0x80  // Default
#define RF_SYNC_OFF               0x00

#define RF_SYNC_FIFOFILL_AUTO     0x00  // Default -- when sync interrupt occurs
#define RF_SYNC_FIFOFILL_MANUAL   0x40

#define RF_SYNC_SIZE_1            0x00
#define RF_SYNC_SIZE_2            0x08
#define RF_SYNC_SIZE_3            0x10
#define RF_SYNC_SIZE_4            0x18  // Default
#define RF_SYNC_SIZE_5            0x20
#define RF_SYNC_SIZE_6            0x28
#define RF_SYNC_SIZE_7            0x30
#define RF_SYNC_SIZE_8            0x38

#define RF_SYNC_TOL_0             0x00  // Default
#define RF_SYNC_TOL_1             0x01
#define RF_SYNC_TOL_2             0x02
#define RF_SYNC_TOL_3             0x03
#define RF_SYNC_TOL_4             0x04
#define RF_SYNC_TOL_5             0x05
#define RF_SYNC_TOL_6             0x06
#define RF_SYNC_TOL_7             0x07


#define RF_RXBW_DCCFREQ_000           0x00
#define RF_RXBW_DCCFREQ_001           0x20
#define RF_RXBW_DCCFREQ_010           0x40  // Default
#define RF_RXBW_DCCFREQ_011           0x60
#define RF_RXBW_DCCFREQ_100           0x80
#define RF_RXBW_DCCFREQ_101           0xA0
#define RF_RXBW_DCCFREQ_110           0xC0
#define RF_RXBW_DCCFREQ_111           0xE0

#define RF_RXBW_MANT_16               0x00
#define RF_RXBW_MANT_20               0x08
#define RF_RXBW_MANT_24               0x10  // Default

#define RF_RXBW_EXP_0                 0x00
#define RF_RXBW_EXP_1                 0x01
#define RF_RXBW_EXP_2                 0x02
#define RF_RXBW_EXP_3                 0x03
#define RF_RXBW_EXP_4                 0x04
#define RF_RXBW_EXP_5                 0x05  // Default
#define RF_RXBW_EXP_6                 0x06
#define RF_RXBW_EXP_7                 0x07
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)  -- huh? I get 66.7572021484375

#define kBlinkThreshold  1700000    // these are the number of loops that need to happen before we blink-  not _really_ time based...

// these were found using a RTL-SDR to find the best carrier frequency
#define RF69_FREQ  910.167
#define RF69_FREQ1 915.000
#define RF69_FREQ2 919.966

#define outputPort Serial


// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

static int16_t packetnum = 0;  // packet counter, we increment per xmission
static int  s_lna_gain = 0;
static bool s_output_temp = false;
static bool s_read_rssi = false;
static int  s_blink_counter = 0;
static uint8_t s_freq = 0;
 
void setup()
{
//    outputPort.begin( 9600 );

    Serial.begin( 9600 );
    Serial1.begin( 9600 );

#ifdef DEBUG
    while (!outputPort) { delay(1); } // wait until serial console is open, remove if not tethered to computer
#endif

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
  
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    
    if( !rf69.init() ) {
        SerialPrint("RFM69 radio init failed\n");
        while (1);
    }

    TxDecoderInit();
    
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if( !rf69.setFrequency(RF69_FREQ) ) {
        SerialPrint("setFrequency failed\n");
    }

    rf69.spiWrite(RH_RF69_REG_13_OCP, RF_OCP_OFF);
    rf69.spiWrite(RH_RF69_REG_02_DATAMODUL, RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE);

    int dataRate = 8621;
    unsigned long r = ((32000000UL + (dataRate / 2)) / dataRate);
    rf69.spiWrite(RH_RF69_REG_03_BITRATEMSB, r >> 8);
    rf69.spiWrite(RH_RF69_REG_04_BITRATELSB, r & 0xFF);
    rf69.spiWrite(RH_RF69_REG_05_FDEVMSB, RF_FDEVMSB_90000);
    rf69.spiWrite(RH_RF69_REG_06_FDEVLSB, RF_FDEVLSB_90000);
    rf69.spiWrite(RH_RF69_REG_37_PACKETCONFIG1, RH_RF69_PACKETCONFIG1_CRCAUTOCLEAROFF);
    rf69.spiWrite(RH_RF69_REG_3D_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);

    // receive bandwidth
    rf69.spiWrite(RH_RF69_REG_19_RXBW,  RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2); // 125 kHz <- default
    rf69.spiWrite(RH_RF69_REG_1A_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2); // 125 kHz <- default
    
    rf69.setPreambleLength( 2 );
    rf69.spiWrite(RH_RF69_REG_2E_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
    rf69.spiWrite(RH_RF69_REG_6F_TESTDAGC, RH_RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF);

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
    
    SerialPrint("RFM69 based TX31U-IT receiver starting up...\nFar Out Labs, LLC (c) 2020\n");
}

void receive()
{
    if( rf69.available() )
    {
        // Should be a message for us now
        uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
        memset( buf, 0, sizeof( buf ) );
        uint8_t len = sizeof(buf);
        if( rf69.recv( buf, &len ) )
        {
            if (!len)
            {
                SerialPrint( "Empty receive buffer!\n" );
                return;
            }

            if( !s_output_temp )  // don't mess up plot with radio data
            {
                if( AnalyzeFrame( buf ) )
                {
                    // super fast blink
                    Blink(LED, 100, 1);
                    frequencyHop();
                }
                SerialPrint( ".\n" );
            }
        }
        else
        {
            SerialPrint("Receive failed\n");
        }
    }
}


void frequencyHop()
{
    ++s_freq;
    if( s_freq > 2 )
        s_freq = 0;
    
    switch( s_freq )
    {
        case 0:
            if( !rf69.setFrequency( RF69_FREQ ) )
                SerialPrint("setFrequency failed: 910MHz\n");
            SerialPrint("\nhop to: 910MHz\n");
            break;

        case 1:
            if( !rf69.setFrequency( RF69_FREQ1 ) )
                SerialPrint("setFrequency failed: 915MHz\n");
            SerialPrint("\nhop to: 915MHz\n");
            break;

        case 2:
            if( !rf69.setFrequency( RF69_FREQ2 ) )
                SerialPrint("setFrequency failed: 920MHz\n");
            SerialPrint("\nhop to: 920MHz\n");
            break;
    }
}


void loop()
{
    if( outputPort.available() )
    {
        char k = outputPort.read();
        if( k == 'r' )
            ReadAllRegs();
        else if ( k == 'z' )
        {
            s_read_rssi = !s_read_rssi;
        }
        else if( k == 'l' )
        {
            ++s_lna_gain;
            if( s_lna_gain > 6 )
            s_lna_gain = 0;

            rf69.spiWrite(RH_RF69_REG_18_LNA, s_lna_gain);
#ifdef REGISTER_DETAIL
            outputPort.print( "LNA gain: " ); outputPort.println( s_lna_gain );
#endif
        }
    }


    receive();
    
    // heartbeat
    if( ++s_blink_counter >= kBlinkThreshold )
    {
        Blink(LED, 400, 1); //blink LED 1 time, 400ms between blink
        s_blink_counter = 0;
    }

    if( s_read_rssi )
    {
#ifdef REGISTER_DETAIL
        outputPort.print( "rssiRead: " );
        outputPort.println( rf69.rssiRead() );
#endif
    }
}


void Blink(byte PIN, int DELAY_MS, byte loops)
{
    for (byte i=0; i<loops; i++)
    {
        digitalWrite(PIN,HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN,LOW);
        delay(DELAY_MS);
    }
}




void ReadAllRegs()
{
  uint8_t regVal;

#ifdef REGISTER_DETAIL
  int capVal;

  //... State Variables for intelligent decoding
  uint8_t modeFSK = 0;
  int bitRate = 0;
  int freqDev = 0;
  long freqCenter = 0;
#endif
  
  outputPort.println("Address - HEX - BIN");
  for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    regVal = rf69.spiRead(regAddr);

    outputPort.print(regAddr, HEX);
    outputPort.print(" - ");
#ifdef REGISTER_DETAIL
    if( regAddr <= RH_RF69_REG_4F_TEMP2 )
    {
      outputPort.print( s_reg_names[regAddr] );
      outputPort.print(" - ");
    }
#endif
    outputPort.print(regVal,HEX);
    outputPort.print(" - ");
    outputPort.println(regVal,BIN);

#ifdef REGISTER_DETAIL
    switch ( regAddr )
    {
        case 0x1 : {
            SerialPrint ( "Controls the automatic Sequencer ( see section 4.2 )\nSequencerOff : " );
            if ( 0x80 & regVal ) {
                SerialPrint ( "1 -> Mode is forced by the user\n" );
            } else {
                SerialPrint ( "0 -> Operating mode as selected with Mode bits in RegOpMode is automatically reached with the Sequencer\n" );
            }
            
            SerialPrint( "\nEnables Listen mode, should be enabled whilst in Standby mode:\nListenOn : " );
            if ( 0x40 & regVal ) {
                SerialPrint ( "1 -> On\n" );
            } else {
                SerialPrint ( "0 -> Off ( see section 4.3)\n" );
            }
            
            SerialPrint( "\nAborts Listen mode when set together with ListenOn=0 See section 4.3.4 for details (Always reads 0.)\n" );
            if ( 0x20 & regVal ) {
                SerialPrint ( "ERROR - ListenAbort should NEVER return 1 this is a write only register\n" );
            }
            
            SerialPrint("\nTransceiver's operating modes:\nMode : ");
            capVal = (regVal >> 2) & 0x7;
            if ( capVal == 0b000 ) {
                SerialPrint ( "000 -> Sleep mode (SLEEP)\n" );
            } else if ( capVal = 0b001 ) {
                SerialPrint ( "001 -> Standby mode (STDBY)\n" );
            } else if ( capVal = 0b010 ) {
                SerialPrint ( "010 -> Frequency Synthesizer mode (FS)\n" );
            } else if ( capVal = 0b011 ) {
                SerialPrint ( "011 -> Transmitter mode (TX)\n" );
            } else if ( capVal = 0b100 ) {
                SerialPrint ( "100 -> Receiver Mode (RX)\n" );
            } else {
                outputPort.print( capVal, BIN );
                SerialPrint ( " -> RESERVED\n" );
            }
            SerialPrint ( "\n" );
            break;
        }
        
        case 0x2 : {
        
            SerialPrint("Data Processing mode:\nDataMode : ");
            capVal = (regVal >> 5) & 0x3;
            if ( capVal == 0b00 ) {
                SerialPrint ( "00 -> Packet mode\n" );
            } else if ( capVal == 0b01 ) {
                SerialPrint ( "01 -> reserved\n" );
            } else if ( capVal == 0b10 ) {
                SerialPrint ( "10 -> Continuous mode with bit synchronizer\n" );
            } else if ( capVal == 0b11 ) {
                SerialPrint ( "11 -> Continuous mode without bit synchronizer\n" );
            }
            
            SerialPrint("\nModulation scheme:\nModulation Type : ");
            capVal = (regVal >> 3) & 0x3;
            if ( capVal == 0b00 ) {
                SerialPrint ( "00 -> FSK\n" );
                modeFSK = 1;
            } else if ( capVal == 0b01 ) {
                SerialPrint ( "01 -> OOK\n" );
            } else if ( capVal == 0b10 ) {
                SerialPrint ( "10 -> reserved\n" );
            } else if ( capVal == 0b11 ) {
                SerialPrint ( "11 -> reserved\n" );
            }
            
            SerialPrint("\nData shaping: ");
            if ( modeFSK ) {
                SerialPrint( "in FSK:\n" );
            } else {
                SerialPrint( "in OOK:\n" );
            }
            SerialPrint ("ModulationShaping : ");
            capVal = regVal & 0x3;
            if ( modeFSK ) {
                if ( capVal == 0b00 ) {
                    SerialPrint ( "00 -> no shaping\n" );
                } else if ( capVal == 0b01 ) {
                    SerialPrint ( "01 -> Gaussian filter, BT = 1.0\n" );
                } else if ( capVal == 0b10 ) {
                    SerialPrint ( "10 -> Gaussian filter, BT = 0.5\n" );
                } else if ( capVal == 0b11 ) {
                    SerialPrint ( "11 -> Gaussian filter, BT = 0.3\n" );
                }
            } else {
                if ( capVal == 0b00 ) {
                    SerialPrint ( "00 -> no shaping\n" );
                } else if ( capVal == 0b01 ) {
                    SerialPrint ( "01 -> filtering with f(cutoff) = BR\n" );
                } else if ( capVal == 0b10 ) {
                    SerialPrint ( "10 -> filtering with f(cutoff) = 2*BR\n" );
                } else if ( capVal == 0b11 ) {
                    SerialPrint ( "ERROR - 11 is reserved\n" );
                }
            }
            
            SerialPrint ( "\n" );
            break;
        }
        
        case 0x3 : {
            bitRate = (regVal << 8);
            break;
        }
        
        case 0x4 : {
            bitRate |= regVal;
            SerialPrint ( "Bit Rate (Chip Rate when Manchester encoding is enabled)\nBitRate : ");
            unsigned long val = 32UL * 1000UL * 1000UL / bitRate;
            outputPort.println( val );
            SerialPrint( "\n" );
            break;
        }
        
        case 0x5 : {
            freqDev = ( (regVal & 0x3f) << 8 );
            break;
        }
        
        case 0x6 : {
            freqDev |= regVal;
            SerialPrint( "Frequency deviation\nFdev : " );
            unsigned long val = RF69_FSTEP * freqDev;
            outputPort.println( val );
            SerialPrint ( "\n" );
            break;
        }
        
        case 0x7 : {
            unsigned long tempVal = regVal;
            freqCenter = ( tempVal << 16 );
            break;
        }
       
        case 0x8 : {
            unsigned long tempVal = regVal;
            freqCenter = freqCenter | ( tempVal << 8 );
            break;
        }

        case 0x9 : {
            freqCenter = freqCenter | regVal;
            SerialPrint ( "RF Carrier frequency\nFRF : " );
            unsigned long val = RF69_FSTEP * freqCenter;
            outputPort.println( val );
            SerialPrint( "\n" );
            break;
        }

        case 0xa : {
            SerialPrint ( "RC calibration control & status\nRcCalDone : " );
            if ( 0x40 & regVal ) {
                SerialPrint ( "1 -> RC calibration is over\n" );
            } else {
                SerialPrint ( "0 -> RC calibration is in progress\n" );
            }
        
            SerialPrint ( "\n" );
            break;
        }

        case 0xb : {
            SerialPrint ( "Improved AFC routine for signals with modulation index lower than 2.  Refer to section 3.4.16 for details\nAfcLowBetaOn : " );
            if ( 0x20 & regVal ) {
                SerialPrint ( "1 -> Improved AFC routine\n" );
            } else {
                SerialPrint ( "0 -> Standard AFC routine\n" );
            }
            SerialPrint ( "\n" );
            break;
        }
        
        case 0xc : {
            SerialPrint ( "Reserved\n\n" );
            break;
        }

        case 0xd : {
            byte val;
            SerialPrint ( "Resolution of Listen mode Idle time (calibrated RC osc):\nListenResolIdle : " );
            val = regVal >> 6;
            if ( val == 0b00 ) {
                SerialPrint ( "00 -> reserved\n" );
            } else if ( val == 0b01 ) {
                SerialPrint ( "01 -> 64 us\n" );
            } else if ( val == 0b10 ) {
                SerialPrint ( "10 -> 4.1 ms\n" );
            } else if ( val == 0b11 ) {
                SerialPrint ( "11 -> 262 ms\n" );
            }
            
            SerialPrint ( "\nResolution of Listen mode Rx time (calibrated RC osc):\nListenResolRx : " );
            val = (regVal >> 4) & 0x3;
            if ( val == 0b00 ) {
                SerialPrint ( "00 -> reserved\n" );
            } else if ( val == 0b01 ) {
                SerialPrint ( "01 -> 64 us\n" );
            } else if ( val == 0b10 ) {
                SerialPrint ( "10 -> 4.1 ms\n" );
            } else if ( val == 0b11 ) {
                SerialPrint ( "11 -> 262 ms\n" );
            }

            SerialPrint ( "\nCriteria for packet acceptance in Listen mode:\nListenCriteria : " );
            if ( 0x8 & regVal ) {
                SerialPrint ( "1 -> signal strength is above RssiThreshold and SyncAddress matched\n" );
            } else {
                SerialPrint ( "0 -> signal strength is above RssiThreshold\n" );
            }
            
            SerialPrint ( "\nAction taken after acceptance of a packet in Listen mode:\nListenEnd : " );
            val = (regVal >> 1 ) & 0x3;
            if ( val == 0b00 ) {
                SerialPrint ( "00 -> chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3)\n" );
            } else if ( val == 0b01 ) {
                SerialPrint ( "01 -> chip stays in Rx mode until PayloadReady or Timeout interrupt occurs.  It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3)\n" );
            } else if ( val == 0b10 ) {
                SerialPrint ( "10 -> chip stays in Rx mode until PayloadReady or Timeout occurs.  Listen mode then resumes in Idle state.  FIFO content is lost at next Rx wakeup.\n" );
            } else if ( val == 0b11 ) {
                SerialPrint ( "11 -> Reserved\n" );
            }
            
            
            SerialPrint ( "\n" );
            break;
        }
        
        default : {
        }
    }
#endif
  }
}

