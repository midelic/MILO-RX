#pragma once

/**************************/
/*** CONFIG VALIDATION  ***/
/**************************/

#ifdef ESP8266
	#define ESP8266_PLATFORM
#endif

#if defined MATEK_RX_R24D ||defined NAMIMNO_RX_NANO_FLASH || defined DIY_RX
  #define DIVERSITY
#endif

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

#ifdef RP2040_E28_2G4M12S
    #undef HAS_PA_LNA
    #define MinPower 10//10mW
    #define MaxPower 13//20mW
    #define USER_MAX_POWER 13 //20mW for example can be defined whatever you need
#endif


#if defined EL24P || defined DIY_RX//No PA/LNA
    #define MinPower 10 // 10mW
    #define MaxPower 13 // 20mW
    #define USER_MAX_POWER 10 //10mW for example can be defined whatever you need
#endif

#ifdef RP2040_PLATFORM
    #undef USE_WIFI 
    #undef MSW_SERIAL
#endif

#ifdef USE_WIFI
    #include "devWIFI_elegantOTA.h"
#endif

#if defined(DEBUG_ON_GPIO3) && defined(ESP8266_PLATFORM)
    #undef MSW_SERIAL
    #define G3PIN 3
    #define G3ON digitalWrite(G3PIN,HIGH)
    #define G3OFF digitalWrite(G3PIN,LOW)
    #define G3TOGGLE digitalWrite(G3PIN,!digitalRead(G3PIN))
    #define G3PULSE(usec) digitalWrite(G3PIN,HIGH);delayMicroseconds(usec); digitalWrite(G3PIN,LOW)
#elif defined(DEBUG_ON_GPIO3) && defined(RP2040_PLATFORM)
    #ifndef G3PIN
        #define G3PIN 14
    #endif
    #define G3ON gpio_put(G3PIN,true)
    #define G3OFF gpio_put(G3PIN,false)
    #define G3TOGGLE gpio_put(G3PIN,!gpio_get(G3PIN))
    #define G3PULSE(usec) gpio_put(G3PIN,true);delayMicroseconds(usec); gpio_put(G3PIN,false)
#else
    #undef G3PIN
    #define G3ON 
    #define G3OFF 
    #define G3TOGGLE 
    #define G3PULSE(usec) 
#endif

#if defined(DEBUG_ON_GPIO1) && defined(ESP8266_PLATFORM)
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
    #define G1PIN 1
    #define G1ON digitalWrite(G1PIN,HIGH)
    #define G1OFF digitalWrite(G1PIN,LOW)
    #define G1TOGGLE digitalWrite(G1PIN,!digitalRead(G1PIN))
    #define G1PULSE(usec) digitalWrite(G1PIN,HIGH);delayMicroseconds(usec); digitalWrite(G1PIN,LOW)
#elif defined(DEBUG_ON_GPIO1) && defined(RP2040_PLATFORM)
    //#undef DEBUG_HELP_FUNCTIONS
    //#undef DEBUG_EEPROM
    //#undef DEBUG_BIND
    //#undef DEBUG_SPORT_SPORTDATA
    //#undef DEBUG_SPORT_SIM_GENERATION
    //#undef DEBUG_DOWNLINK_TLM_FRAME
    //#undef DEBUG_SEND_POLLING
    //#undef DEBUG_MSP
    //#undef DEBUG_LOOP_TIMING
    //#undef DEBUG_SERVODATA
    #ifndef G1PIN
        #define G1PIN 15
    #endif
    #define G1ON gpio_put(G1PIN,true)
    #define G1OFF gpio_put(G1PIN,false)
    #define G1TOGGLE gpio_put(G1PIN,!gpio_get(G1PIN))
    #define G1PULSE(usec) gpio_put(G1PIN,true);delayMicroseconds(usec); gpio_put(G1PIN,false)
#else
    #undef G1PIN
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
        defined(DEBUG_UPLINK_TLM_DATA) || defined(DEBUG_SEQUENCE) || defined(DEBUG_UPLINK_TLM_SENT_TO_SPORT)
    #ifdef ESP8286_PLATFORM
        #undef SBUS  // disable SBUS because it uses the same pin
    #endif
    #define DEBUG_WITH_SERIAL_PRINT
    #define debugln(msg, ...)  { sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}
    #define debug(msg, ...)  { sprintf(debug_buf, msg , ##__VA_ARGS__); Serial.write(debug_buf);}
#else
    #define debugln(...) { } 
    #define debug(...) { }
#endif

