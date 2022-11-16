/*

For debugging, you can activate some options in your own file named _myDebugOptions.h
This file can use one or several of the options given here below.

This file _possibleDebugOptions.h is delivered with the project just to give a list of all possible values.
Only the values set in you own _myDebugOptions.h are used at compilation.


// debug options
-------------------
//#define DEBUG_HELP_FUNCTIONS        // create some functions to help debugging; one print
//#define DEBUG_EEPROM               // print EEPROM data
//#define DEBUG_BIND                // print bind data
//#define DEBUG_FHSS                  // print the generated fhss channels
//#define DEBUG_INCOMMING_SPORTDATA   // print a frame that has been read from a sport sensor (no PHID but with stuffing and original CRC) 
//#define DEBUG_SPORT_SPORTDATA // print the original Sport data from sensor (or simulated)
#define DEBUG_ON_GPIO3          // allow to generate pulse on pin 3 (normaly Sport pin) for debuging; disable automatically MSW_SERIAL
                                  // code contains then quite many pulses on pin 3 that allows to check that RX is synchronized with TX
//#define DEBUG_SIM_SPORT_SENSOR  // generate dummy Sport data; allow to use SPORT_pin 3 for generating pulses
#define DEBUG_SPORT_INTERVAL 500  // interval in msec between 2 dummy frames (must be at least about 22 to let SX1280 sent the frame)

//#define DEBUG_SPORT_SIM_GENERATION // print the dummy Sport data
//#define DEBUG_DOWNLINK_TLM_FRAME  // print the data in the downlink tlm frame
//#define DEBUG_SEND_POLLING        // print message to say that we output some bytes on sport
//#define DEBUG_ON_GPIO1            // allow to generate pulses on pin 1 (e.g. to debug receiving timming on Sport); 
                                // disable automatically SBUS and DEBUG_EEPROM DEBUG_BIND, DEBUG_SPORT_SPORTDATA, DEBUG_SPORT_SIM_GENERATION,
                                // DEBUG_DOWNLINK_TLM_FRAME , DEBUG_SEND_POLLING, DEBUG_MSP, DEBUG_LOOP_TIMING , #define DEBUG_SERVODATA ,
                                // DEBUG_HELP_FUNCTIONS 
//#define DEBUG_MSP
//#define DEBUG_LOOP_TIMING
//#define DEBUG_SERVODATA
#define DEBUG_RC_CHANNEL_DATA       // print 8 Rcchannels values received in a frame
#define DEBUG_UPLINK_TLM_DATA       // print uplink tlm data received
#define DEBUG_UPLINK_TLM_SENT_TO_SPORT // print the uplink tlm as it would be sent on Sport port
#define DEBUG_SEQUENCE              // print info about up and downlink counter when a frame is sent/received

//#define DEBUG_FHSS
//#define DEBUG_WITH_FIXED_FHSS

*/