
/* **************************
    * By Midelic on RCGroups *
    **************************
    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    MiLo Rx code is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/
//#define DEBUG_HELP_FUNCTIONS        // create some functions to help debugging; one print
//#define DEBUG_EEPROM               // print EEPROM data
//#define DEBUG_BIND                // print bind data
//#define DEBUG_FHSS                  // print the generated fhss channels
//#define DEBUG_INCOMMING_SPORTDATA   // print a frame that has been read from a sport sensor (no PHID but with stuffing and original CRC) 
//#define DEBUG_SPORT_SPORTDATA // print the original Sport data from sensor (or simulated)
//#define DEBUG_ON_GPIO3          // allow to generate pulse on pin 3 (normaly Sport pin) for debuging; disable automatically MSW_SERIAL
                                  // code contains then quite many pulses on pin 3 that allows to check that RX is synchronized with TX
//#define DEBUG_SIM_SPORT_SENSOR  // generate dummy Sport data; allow to use SPORT_pin 3 for generating pulses
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
//#define DEBUG_RC_CHANNEL_DATA       // print 8 Rcchannels values received in a frame
//#define DEBUG_UPLINK_TLM_DATA       // print uplink tlm data received

#ifdef ESP8266
	#define ESP8266_PLATFORM
#endif
//----Modules defs--------
//#define EL24P//flywoo RX
//#define DIY_RX
//#define MATEK_RX_R24D
//#define MATEK_RX_R24S
//#define BETA_FPV_RX_NANO // or Cycloon clone
//#define NAMIMNO_RX_NANO_FLASH
#define ESP8266_E28_2G4M20S

#define MSW_SERIAL
//#define HC_BIND
//#define USER_MAX_POWER
#define TELEMETRY 


#if defined MATEK_RX_R24D ||defined NAMIMNO_RX_NANO_FLASH || defined DIY_RX
  #define DIVERSITY
#endif

//#define SWAMPING
//#define RSSI_AVG
#define SPORT_TELEMETRY 
#define FAILSAFE

//#define MinPower PWR_10mW
//#define MaxPower PWR_100mW
#if defined MATEK_RX_R24D ||defined NAMIMNO_RX_NANO_FLASH || defined MATEK_RX_R24S || defined BETA_FPV_RX_NANO 
    #define HAS_PA_LNA
    #define MinPower -10//10mW//  powerValues[4]= [-10,-6,-3,1];(Beta_FPV&Matek)//powerValues[4]=[-15,-10,-7,-3];(NAMIMNO_RX_NANO_FLASH)
    #define MaxPower 1//100mW
    #ifdef USER_MAX_POWER
        #define UserPower -10//10mW for example can be defined whatever you need
    #endif
#endif

#ifdef ESP8266_E28_2G4M20S
    #define HAS_PA_LNA
    #define MinPower -13//10mW
    #define MaxPower -2//100mW
    #define USER_MAX_POWER -13 //10mW for example can be defined whatever you need
#endif

#if defined EL24P || defined DIY_RX//No PA/LNA
    #define MinPower 10 // 10mW
    #define MaxPower 13 // 20mW
    #define USER_MAX_POWER 10 //10mW for example can be defined whatever you need
#endif

#define SBUS // mstrens removed to test
#define TX_FAILSAFE
//#define PWM_SERVO
//#define ADC_VOLT
#define STATISTIC
//#define SERVO_RATE
//#define PARALLEL_SERVO
//#define EU_LBT
#define USE_WIFI

#ifdef USE_WIFI
    #include "devWIFI_elegantOTA.h"
#endif

#ifdef DEBUG_ON_GPIO3
    #undef MSW_SERIAL
    //#undef TELEMETRY
    //#undef SPORT_TELEMETRY 
    #define G3ON digitalWrite(3,HIGH)
    #define G3OFF digitalWrite(3,LOW)
    #define G3TOGGLE digitalWrite(3,!digitalRead(3))
    #define G3PULSE(usec) digitalWrite(3,HIGH);delayMicroseconds(usec); digitalWrite(3,LOW)
#else
 #define G3ON 
 #define G3OFF 
 #define G3TOGGLE 
 #define G3PULSE(usec) 
#endif

#ifdef DEBUG_ON_GPIO1
    #undef SBUS
    #undef DEBUG_HELP_FUNCTIONS
    #undef DEBUG_EEPROM
    #undef DEBUG_BIND
    #undef DEBUG_SPORT_SPORTDATA
    #undef DEBUG_SPORT_SIM_GENERATION
    #undef DEBUG_DOWNLINK_TLM_FRAME
    #undef DEBUG_SEND_POLLING
    #undef DEBUG_MSP
    #undef DEBUG_LOOP_TIMING
    #undef DEBUG_SERVODATA
    #define G1ON digitalWrite(1,HIGH)
    #define G1OFF digitalWrite(1,LOW)
    #define G1TOGGLE digitalWrite(1,!digitalRead(1))
    #define G1PULSE(usec) digitalWrite(1,HIGH);delayMicroseconds(usec); digitalWrite(1,LOW)
#else
    #define G1ON 
    #define G1OFF 
    #define G1TOGGLE 
    #define G1PULSE(usec) 
#endif

#ifdef DEBUG_SIM_SPORT_SENSOR
    #undef MSW_SERIAL
    #define SPORT_TELEMETRY
    #define TELEMETRY
    #define DEBUG
#endif

#if defined(DEBUG_SPORT_SPORTDATA) || defined(DEBUG_SPORT_SIM_GENERATION) || defined(DEBUG_DOWNLINK_TLM_FRAME) ||\
        defined(DEBUG_BIND) || defined (DEBUG_EEPROM) || defined (DEBUG_MSP) || defined (DEBUG_LOOP_TIMING)||defined (DEBUG_SERVODATA) ||\
        defined(DEBUG_SEND_POLLING) || defined(DEBUG_INCOMMING_SPORTDATA) || defined(DEBUG_FHSS) || defined(DEBUG_RC_CHANNEL_DATA) ||\
        defined(DEBUG_UPLINK_TLM_DATA)
    #undef SBUS  // disable SBUS because it uses the same pin
    #define DEBUG_WITH_SERIAL_PRINT
    #define debugln(msg, ...)  { sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}
    #define debug(msg, ...)  { sprintf(debug_buf, msg , ##__VA_ARGS__); Serial.write(debug_buf);}
#else
    #define debugln(...) { } 
    #define debug(...) { }
#endif

