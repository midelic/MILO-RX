
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
//#define DEBUG_HELP_FUNCTIONS                      // create some functions to help debugging; one print
//#define DEBUG_EEPROM               // print EEPROM data
//#define DEBUG_BIND                // print bind data
//#define DEBUG_FHSS                  // print the generated fhss channels
//#define DEBUG_INCOMMING_SPORTDATA   // print a frame that has been read from a sport sensor (no PHID but with stuffing and original CRC) 
//#define DEBUG_SPORT_SPORTDATA // print the original Sport data from sensor (or simulated)
//#define DEBUG_ON_GPIO3          // allow to generate pulse on pin 3 (normaly Sport pin) for debuging; disable automatically MSW_SERIAL
//#define DEBUG_SIM_SPORT_SENSOR  // generate dummy Sport data; allow to use SX1280_SPORT_pin 3 for generating pulses
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



#ifdef ESP8266
	#define ESP8266_PLATFORM
#endif
//----Modules defs--------
//#define EL24P//flywoo RX
//#define DIY_RX
//#define MATEK_RX_R24D
//#define MATEK_RX_R24S
//define BETA_FPV_RX_NANO
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

//#define SBUS // mstrens removed to test
//#define TX_FAILSAFE
//#define PWM_SERVO
//#define ADC_VOLT
#define STATISTIC
//#define SERVO_RATE
//#define PARALLEL_SERVO
//#define EU_LBT
//#define USE_WIFI

#ifdef USE_WIFI
    #include "devWIFI_elegantOTA.h"
#endif

#ifdef DEBUG_ON_GPIO3
    #undef MSW_SERIAL
    #undef TELEMETRY
    #undef SPORT_TELEMETRY 
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
        defined(DEBUG_SEND_POLLING) || defined(DEBUG_INCOMMING_SPORTDATA) || defined(DEBUG_FHSS)
    #undef SBUS  // disable SBUS because it uses the same pin
    #define DEBUG_WITH_SERIAL_PRINT
    #define debugln(msg, ...)  { sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}
    #define debug(msg, ...)  { sprintf(debug_buf, msg , ##__VA_ARGS__); Serial.write(debug_buf);}
#else
    #define debugln(...) { } 
    #define debug(...) { }
#endif


/*
    Protocol description:
    2.4Ghz LORA modulation
    - 142 Hz frame rate(7ms)
    - Data Rate ~76kb/s(-108dBm)
    - Bw-812; SF6 ; CR -LI -4/7 .
    - Preamble length 12 symbols
    - Fixed length packet format(implicit) -16 bytes
    - Downlink telemetry rate(1:3)
    - Uplink telemetry rate(1:6)
    - Hardware CRC is ON.
    
    # Normal frame channels 1-8; frame rate 7ms.
    
    //0. reserve 2 bits (bits 7..6) | next expected telemetry down link frame counter(sequence) (bits 5..4 (2 bits=4 val)) | reserve (bit 3) | Frame type(bits 2..0 (3 lsb bits))
    0.- bits 7..6 next expected telemetry down link frame counter(sequence) (2 bits=4 val))
      - bits 5..3 Failsafe ID (3 bits) 
      - bits 2..0 Frame type (3 bits)
    1. txid1 TXID on 16 bits
    2. txid2
    3. flag next frame must be dwn tlm frame (bit 7) | flag requesing starting WIFI (bit 6) | Model ID /Rx_Num(bits 5....0 = 6 bits) 
    4. channels 8 channels/frame ; 11bits/channel
    5. channels total 11 bytes of channels data in the packet frame
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11. channels
    12. channels
    13. channels
    14. channels
    15. reserve

    # Normal frame channels 9-16 separate; frame rate 7ms.
    0. reserve 2 bits (bits 7..6) | next expected telemetry down link frame counter(sequence) (bits 5..4 (2 bits=4 val)) | reserve (bit 3) | Frame type(bits 2..0 (3 lsb bits))
    1. txid1 TXID on 16 bits
    2. txid2
    3. flag next frame must be dwn tlm frame (bit 7) | flag requesing starting WIFI (bit 6) | Model ID /Rx_Num(bits 5....0 = 6 bits) 
    4. channels 8 channels/frame ; 11bits/channel
    5. channels total 11 bytes of channels data in the packet frame
    6. channels
    7. channels
    8. channels
    9. channels
    10. channels
    11. channels
    12. channels
    13. channels
    14. channels
    15. reserve
    
    # TX uplink telemetry frame can be sent separate ;frame rate 7ms;1:6 telemetry data rate.
    0. reserve 2 bits (bits 7..6) | next expected telemetry down link frame counter(sequence) (bits 5..4 (2 bits=4 val)) | reserve (bit 3) | Frame type(bits 2..0 (3 lsb bits))
    1. txid1 TXID on 16 bits
    2. txid2
    3. no. of bytes in sport frame(on max 4bits 7..4) | reserve (2bits 3..2) | telemetry uplink counter sequence(2 bits 1..0)
    4.Sport data byte1
    5.Sport data byte 2
    6.Sport data byte 3
    7.Sport data byte 4
    8.SPort data byte 5
    9.SPort data byte 6
    10.SPort data byte 7
    11.SPort data byte 8
    12.SPort data byte 9
    13.SPort data byte 10
    14.SPort data byte 11 ; 
    15.SPort data byte 12 ;  12 bytes sport telemetry

    # RX downlink telemetry frame sent separate at a fixed rate of 1:3;frame rate 7ms. Can contain 2 Sport frame 
    0. - bits 7...2 : MSB of TXID1 (6 bits)
       - bits 1...0 : current downlink tlm counter (2 bits); when received TX should send this counter + 1type of link data packet(RSSI/SNR /LQI) (2 bits= 3 values currently) 
    1. - bits 7...2 : MSB of TXID2 (6 bits)
       - bits 1...0 : last upllink tlm counter received (2 bits); 
    2. - bit 7 : reserve
         bits 6..5 : recodified PRIM from sport frame1 (0X30=>0, 0X31=>1,0X32=>2, 0X10=>3)
         bits 4..0 : PHID from sport frame1 (5 bits using a mask 0X1F; 0X1F = no data; 0X1E = link quality data)
    3. - bit 7 : reserve
         bits 6..5 : recodified PRIM from sport frame2 (0X30=>0, 0X31=>1,0X32=>2, 0X10=>0)
         bits 4..0 : PHID from sport frame2 (5 bits using a mask 0X1F; 0X1F = no data; 0X1E = link quality data)
    4. field ID1 from sport frame1
    5. field ID2 from sport frame1
    6...9. Value from sport frame1 (4 bytes)
    10. field ID1 from sport frame2
    11. field ID2 from sport frame2
    12...15. Value from sport frame2 (4 bytes)
    
    
    # bind packet
    0. Frame type = BIND_PACKET = 0
    1. rx_tx_addr[3];
    2. rx_tx_addr[2];
    3. rx_tx_addr[1];
    4. rx_tx_addr[0];
    5. RX_num;
    6. chanskip;
    7. up to 15.  0xA7


    # Frame Sequence
    0- downlink telemetry
    1- RC channels 1_8_1 
    2- RC channels 9_16
    3- downlink telemetry
    4- RC channels 1_8_2
    5 -uplink telemetry                 
    6- downlink telemetry
    7- RC channels 9_16
    8- RC channels 1_8_1
    9- downlink telemetry
    10- RC channels 9_16
    11- uplink telemetry               
    12- downlink telemetry
    13- RC channels 1_8_2
    14- RC channels 9_16
    15- downlink telemetry
    
    
    0 - downlink telemetry
    1- RC channels 1_8_1         
    2- RC channels 1_8_2      
    3- downlink telemetry      
    4- RC channels 1_8_1       
    5 -uplink telemetry              
    6- downlink telemetry      
    7- RC channels 1_8_2         
    8- RC channels 1_8_1          
    9- downlink telemetry            
    10- RC channels 1_8_2       
    11- uplink telemetry           
    12- downlink telemetry
    13- RC channels 1_8_1            
    14- RC channels 1_8_2              
    15- downlink telemetry
    16- RC channels 1_8_1            
    17  uplink telemetry              
    15- downlink telemetry
    
    
*/
