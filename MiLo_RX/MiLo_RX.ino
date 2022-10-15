/* **************************
    By Midelic on RCGroups
    **************************
    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    MiLo_RX code is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/

#undef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR IRAM_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))

#include <SPI.h>
#include "_config.h"
#include "pins.h"
#include "iface_sx1280.h"
#include "MiLo_FHSS.h"
#include "SX1280.h"

#ifdef SW_SERIAL
    #undef MSW_SERIAL
    #include <SoftwareSerial.h>
    SoftwareSerial swSer;
#elif defined MSW_SERIAL
    #undef SW_SERIAL
#endif

#define RATE_DEFAULT 0
#define RATE_BINDING 0
#define RATE_100HZ 1 //100HZ
#define RATE_150HZ 0 //150HZ
#define RATE_MAX 3

//Failsafe
#ifdef TX_FAILSAFE
    bool setFSfromTx = false;
    bool fsStarted      = false;
    bool fsChanged   = false;
#endif
//Failsafe RX
bool setFSfromRx = false;
uint16_t countFS = 0;

//EEPROM
uint32_t  address = 0;//EEPROM start adress

//Channels
#define NO_PULSE  0
#define HOLD  2047
#define MAX_MISSING_PKT 100 // when missingPackets reachs this value, failsafe is ON and connection is lost

uint16_t ServoData[16]; // rc channels to be used for Sbus (received from Tx or failsafe)
volatile int32_t missingPackets = 0;
bool packetToDecode = false; // true means that a packet (any type) has been received from Tx and must be decoded
uint8_t jumper = 0;
uint16_t c[8];
uint32_t t_out = FHSS_CHANNELS_NUM;
uint32_t t_tune = 500;
uint16_t wordTemp;
volatile uint8_t all_off = 0;   // 1 means NO pulse
uint32_t packetTimer;
volatile bool frameReceived = false;
uint16_t LED_count = 0;
uint8_t RxData[15];
uint32_t bindingTime = 0;
uint32_t MProtocol_id = 0;
uint8_t PayloadLength = 15;
uint8_t FrameType = 0;
uint32_t LastReceivedPacketTime;
//Debug
//uint16_t BackgroundTime;
uint32_t debugTimer;
char debug_buf[64];
uint8_t packetSeq = 0;
bool startWifi = false;
bool processSportflag = false;
enum { // types of packet being received from TX
    BIND_PACKET = 0,
    CH1_8_PACKET1,
    CH1_8_PACKET2,
    CH1_16_PACKET,
    TLM_PACKET,
    FLSF_PACKET1,
    FLSF_PACKET2,
};


typedef struct {
    uint8_t txid[2];
    uint8_t rx_num;
    uint8_t chanskip;
    uint16_t FS_data[16];//failsafe data
} STORAGE_MILO_DATA;

STORAGE_MILO_DATA *MiLoStrgPtr , MiLoStorage;

#include "Flash.h"
#include "SBUS.h"

//Link Statistics
uint8_t uplinkLQ;
int8_t LastPacketRSSI = 0;
int8_t LastPacketSNR = 0;
uint8_t aPacketSeen = 0;
uint32_t TotalDroppedPacketCount ;
uint32_t TotalCrcErrors ;
uint32_t TotalPktErrors = 0;
//uint32_t AntennaMissingPackets = 0 ;
//uint16_t AntennaSwaps ;
uint16_t DroppedPacketCount ;
uint8_t ThisPacketDropped ;
uint8_t DropHistory[100] ;
uint8_t DropHistoryIndex ;
uint8_t DropHistoryPercent ;
uint8_t DropHistorySend ;
bool packetCount = false;
uint8_t antenna = 0;

uint8_t countUntilWiFi = 0;
uint8_t sportCount = 0;
    

typedef struct
{
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
    
} MiLo_statistics;
MiLo_statistics MiLoStats;

//TELEMETRY
#ifdef TELEMETRY
    uint8_t frame[15];// frame to be sent to TX
    uint32_t pass = 0;
    //uint8_t telemetryRX = 0;// when 1 next slot is downlink telemetry
    uint8_t TelemetryExpectedId;
    uint8_t TelemetryId;
    uint8_t UplinkTlmId = 0;
    bool skipUntilStart = false;
    volatile bool sportMSPflag = false;
    uint8_t sportMSPdatastuff[16];
    uint8_t sportMSPdata[16];
    volatile uint8_t idxs = 0;
    uint8_t smartPortRxBytes = 0;
    uint8_t ReceivedSportData[11];//transfer all sport telemetry data received from TX in a buffer including No. of bytes in sport telemetry frame(uplink frame)
#endif

#define NOP() __asm__ __volatile__("nop")

void ICACHE_RAM_ATTR dioISR();
void ICACHE_RAM_ATTR callSportSwSerial(void);
void ICACHE_RAM_ATTR SportPollISR(void);
void ICACHE_RAM_ATTR MiloTlmSent(void);
void ICACHE_RAM_ATTR MiLoTlm_build_frame();
uint8_t ICACHE_RAM_ATTR MiLoTlm_append_sport_data(uint8_t *buf);
uint8_t ICACHE_RAM_ATTR MiLoTlmDataLink(uint8_t pas);

#ifdef SPORT_TELEMETRY
    #include "Sport_serial.h"
#endif

//MILO-SX1280 RF parameters
uint8_t currOpmode = SX1280_MODE_SLEEP;
bool IQinverted = false;
uint32_t currFreq = 0;
uint8_t LoRaBandwidth;
uint32_t FreqCorrection;
uint32_t FreqCorrectionRegValue;
uint16_t timeout = 0xFFFF;
uint8_t packetLengthType;

typedef struct MiLo_mod_settings_s
{
    uint8_t index;
    uint8_t radio_type;//RADIO_TYPE_SX128x_LORA
    uint8_t frame_rate_type;
    uint8_t bw;
    uint8_t sf;
    uint8_t cr;
    uint32_t interval;          // interval in us seconds that corresponds to that frequency
    uint8_t TLMinterval;        // every X packets is a response TLM packet
    uint8_t PreambleLen;         //no.symbols(default 12)
    uint8_t PayloadLength;      // Number of OTA bytes to be sent.
} MiLo_mod_settings_t;

typedef struct MiLo_rf_pref_params_s
{
    uint8_t index;
    uint8_t frame_rate;                    // not used by MILO
    int32_t RXsensitivity;                // expected RF sensitivity based on - not used
    uint32_t TOA;                         // time on air in microseconds - not used
    uint32_t DisconnectTimeoutMs;         // Time without a packet before receiver goes to disconnected (ms) - from ExpressLRS - not used
    uint32_t RxLockTimeoutMs;             // Max time to go from tentative -> connected state on receiver (ms) - from ExpressLRS - not used
} MiLo_rf_pref_params_t;

enum
{
    TLM_RATIO_NO_TLM = 0,
    TLM_RATIO_1_1 = 1,
    TLM_RATIO_1_2 = 2,
    TLM_RATIO_1_3 = 3,
    TLM_RATIO_1_4 = 4
} ;

MiLo_mod_settings_s *MiLo_currAirRate_Modparams;
MiLo_rf_pref_params_s *MiLo_currAirRate_RFperfParams;

MiLo_mod_settings_s MiLo_AirRateConfig[RATE_MAX] =
{
    {0, RADIO_TYPE_SX128x_LORA, RATE_LORA_150HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_7, 7000, TLM_RATIO_1_3, 12, 15 },
    {1, RADIO_TYPE_SX128x_LORA, RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_6, 9000, TLM_RATIO_1_3, 12, 15}
};

MiLo_rf_pref_params_s MiLo_AirRateRFperf[RATE_MAX] =
{
    {0, RATE_LORA_150HZ,  -108,  5060, 3500, 2500},
    {1, RATE_LORA_100HZ,  -112,  7605, 3500, 2500}
};

void  ICACHE_RAM_ATTR MiLo_SetRFLinkRate(uint8_t index) // Set speed of RF link (hz) index values
{
    MiLo_mod_settings_s *const ModParams = &MiLo_AirRateConfig[index];
    MiLo_rf_pref_params_s *const RFperf = &MiLo_AirRateRFperf[index];
    
    bool invertIQ = 0x01;
    if ((ModParams == MiLo_currAirRate_Modparams)
            && (RFperf == MiLo_currAirRate_RFperfParams)
            && (invertIQ == IQinverted))
    return;
    //uint32_t interval = 0XFFFF;//use micros() instead
    SX1280_Config(ModParams->bw, ModParams->sf, ModParams->cr, GetCurrFreq(),
    ModParams->PreambleLen, invertIQ, ModParams->PayloadLength);
    
    MiLo_currAirRate_Modparams = ModParams;
    MiLo_currAirRate_RFperfParams = RFperf;
}
//end MILO-RF parameters

void setup()
{
    SetupTarget();
    delay(10);//wait for stabilization
    
    #if defined(DEBUG) || defined(DEBUG_BIND) || defined (DEBUG_EEPROM) || defined (DEBUG_MSP) || defined (DEBUG_LOOP_TIMING)||defined (DEBUG_DATA)||defined (DEBUG_SPORT)
        #undef SBUS
        Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
        #define debugln(msg, ...)  { sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}
    #else
        #define debugln(...) { } 
    #endif
    
    #ifdef SBUS
        init_SBUS(); // sbus uses Serial.begin with inverted signal
    #endif
    #if defined SPORT_TELEMETRY
        initSportUart();// sport to sensort uses SW serial with the same pin for TX and Rx, signal is inverted
        SportHead = SportTail = 0;
    #endif
    
    Fhss_Init();// setup some fhss fixed variables
    ReadEEPROMdata(address);
    #if defined(DEBUG_EEPROM)
        delay(1000);
        debugln("txid1 = %d,txid2 = %d,rx_num = %d,chanskip = %d", MiLoStorage.txid[0], MiLoStorage.txid[1], MiLoStorage.rx_num, MiLoStorage.chanskip);
        debugln("MP_id = %d" ,MProtocol_id );
        //for(uint8_t i = 0 ;i < 16;i++)
        //{
        //   Serial.println(MiLoStorage.FS_data[i]);
        //}
    #endif
    #ifdef HC_BIND
        MProtocol_id = 7059696;
        MiLoStorage.rx_num = 0;
        MiLoStorage.txid[0] = 240;
        MiLoStorage.txid[1] = 184;          
        //MProtocol_id = 13788120;
        //MiLoStorage.rx_num = 0;
        //MiLoStorage.txid[0] = 216;
        //MiLoStorage.txid[1] = 99;        
    #endif
    is_in_binding = false;
    //MiLoRxBinding(0); // ReadEEPROMdata and set is_in_binding = false;
    Fhss_generate(MProtocol_id); // generates the used frequency (based on param in EEPROM)
    currFreq = GetCurrFreq(); //set frequency first or an error will occur!!!
    bool init_success = SX1280_Begin();
    if (!init_success)
    {
        debugln("Init of SX1280 failed");
        //return; // commented by mstrens in order to start testing without a SX1280 module
    }
    else
    {
        all_off = 1;    //no pulse
        //MiLo_SetRFLinkRate(RATE_100HZ));
        MiLo_SetRFLinkRate(RATE_150HZ);
        SX1280_SetFrequencyReg(currFreq);
        PayloadLength = MiLo_currAirRate_Modparams->PayloadLength;
        SX1280_SetOutputPower(MaxPower); // set power to max. We do not start sending tlm immediately so it is not an issue
        #ifdef HAS_PA_LNA
            SX1280_SetTxRxMode(RX_EN);//LNA enable
        #endif
        SX1280_SetMode(SX1280_MODE_RX);
        frameReceived = false;
        bindingTime = millis() ;
        #ifdef SPORT_TELEMETRY
            ConfigTimer();
        #endif
        
    }
}

void applyFailsafe() 
// put failsafe values in ServoData[] and generates Sbus every 14 msec.
{
    for (uint8_t i = 0; i < 16; i++)
    {
        uint16_t word;
        word = MiLoStorage.FS_data[i];
        if (word == NO_PULSE) //no pulse
        {
            ServoData[i] = 0;
            all_off = 1; // all_off = 1 means that pulses/sbus may not be generated
        }
        else if (word == HOLD) //2047 equivalent is hold value so no update data.
        {
            //do nothing
        }
        else
        {
            ServoData[i] = word;
        }              
        #if defined(SBUS)
            channel[i] = (ServoData[i]-881)*1.6;//881-2159 to 0-2047
            channel[i] = constrain(channel[i],0,2047);
        #endif
    }
    #if defined SBUS
        SBUS_frame(); // fill Sbus frame with channels data
        if (millis() - sbus_timer > 14)
        { //only when connection lost, every 14ms
            sbus_timer = millis();
            if (all_off == 0) // send Sbus only if at least one packet has already been received and failsafe do not use "NO_PULSE" 
                for (uint8_t i = 0; i < TXBUFFER_SIZE; i++)
                    Serial.write(sbus[i]);
        }               
    #endif
} 

void handleShortTimeout()
{ // when a short timeout occurs (7msec), change antenna, update some counters and perform a freq hop
    #ifdef DIVERSITY
        if ((missingPackets & 0x03 ) == 3) // change once every 4 consecutive missing packets 
        {
            if (antenna ==1)
            {
                SX1280_ANT_SEL_off;
                antenna = 0;//ant 1
            }
            else
            {
                SX1280_ANT_SEL_on;
                antenna = 1;//ant2
            }
        }
    #endif
    
    if (missingPackets > MAX_MISSING_PKT)// we just lost the connection
    {
        t_out = FHSS_CHANNELS_NUM;// wait max 68 slots of 7 msec before exiting while()
        countFS = 0;
        #ifdef TELEMETRY
            packetSeq = 0;
        #endif
        uplinkLQ = 0;    
    }

    if (jumper)//jumper = 1 when failsafe is activated
        countFS ++;
    
    #ifdef STATISTIC
        if ( aPacketSeen > 5 )//count dropped packets when the receiver is receiveing normally packets
        {
            ThisPacketDropped = 1;
            if (packetCount)//downlink uses one time slot so it will take one missing packet
            {
                ThisPacketDropped = 0;//compensate for downlink missing packet
                packetCount = false;
            }
        }
    #endif      
    
    missingPackets++;
    if (missingPackets > 2)
        t_tune = 0;
    nextChannel(1);
    SX1280_SetFrequencyReg(GetCurrFreq());
}

void handleLongTimeout()
{ // when 68 time interval expires without receiving a frame, process LED, failsafe setup and frequency hop. 
    if (jumper == 0) 
    { // When button is not pressed toggle LED to show that link is lost
        LED_count++ & 0x02 ? LED_on : LED_off;
    }
    else
    { //when button is pressed while Rx is not connected, Failsafe are reset on 0 (= no pulse) 
        uint8_t n = 10;
        while (n--)
        { //fast blinking until resetting  FS data from RX
            LED_toggle;
            delay(100); //blink LED
        }
        bool  saveFsToEprom = false;
        detachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin));    
        for (uint8_t i = 0 ; i < 16; i++)
        {
            if (MiLoStorage.FS_data[i] != 0)
            {
                MiLoStorage.FS_data[i] = 0;//reset the FS values while RX is wating to connect                                             
                EEPROMWriteInt(address + 4 + 2 * i, MiLoStorage.FS_data[i]);                       
                saveFsToEprom = true;
            }
        }                   
        if (saveFsToEprom)
            EEPROM.commit();
        attachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin), dioISR, RISING); //attach interrupt to DIO1 pin
        jumper = 0;
    }
    nextChannel(3); // frequency hop after 68 slots. So Rx stays listening on the same channel while Tx hop every slot 
    SX1280_SetFrequencyReg(GetCurrFreq());
}



void loop()
{
    static uint32_t interval = MiLo_currAirRate_Modparams->interval;
    if (bind_jumper() && jumper == 0// when button is pushed and previously it was not pressed
            #ifdef TX_FAILSAFE
            && setFSfromTx == false
            #endif
            )
        if (countFS == 0) //only at boot reset (Failsafe) no reset while the rx is bound
        {
            countFS = 1;
            jumper = 1;
        }
    if ((millis() - bindingTime) > 20000 && aPacketSeen == 0)
    { //Perform automatically a bind 20 sec after startup if a packet has not yet been received
        timer0_detachInterrupt();//timer0 is needed for wifi
        MiLoRxBind();  // note : we never exit this process: when binding is done, led is blinking and RX must be powerwed off and on again
    }
    #if defined(FAILSAFE)
        if (missingPackets > MAX_MISSING_PKT)
            applyFailsafe() ; // put failsafe values in ServoData[] and generates Sbus every 14 msec.
    #endif
    packetTimer = micros();
    while (1)
    {
        if ((micros() - packetTimer) >= ((t_out * interval) + t_tune))
        {// when timeout occurs (after 1 or FHSS_CHANNELS_NUM * interval; so after 7ms or 476 msec
            if (t_out < FHSS_CHANNELS_NUM)// if we where connected and just waited for 1 interval = 7 msec
            {
                handleShortTimeout(); //change antenna, update some counters and perform a freq hop
            }
            else// t_out = 68 and so it means no connection or connection has already been lost (after to many missing packets) 
            {
                handleLongTimeout(); //process LED, failsafe setup and frequency hop
            }
            break;// exit while() when a time out occurs
        }
        
        if (frameReceived == true)// a frame has been received from the TX (flag has been set in DioISR)
        {
            uint8_t const FIFOaddr = SX1280_GetRxBufferAddr();
            SX1280_ReadBuffer(FIFOaddr, RxData, PayloadLength);
            SX1280_GetLastPacketStats();
            if ((RxData[1] == MiLoStorage.txid[0]) && RxData[2] == MiLoStorage.txid[1]) // Only if correct txid will pass
            {
                nextChannel(1);
                SX1280_SetFrequencyReg(GetCurrFreq());
                uint32_t packet_Timer = micros() ;                  
                int32_t diff = packet_Timer - LastReceivedPacketTime ;
                LastReceivedPacketTime = packet_Timer ;
                if (( diff > 6300) && ( diff < 7700) )
                {
                    interval = interval+ 0.5*(diff - interval ); // automatic update of interval with smoothing formula
                }
                #ifdef DEBUG_LOOP_TIMING
                    debugln("intval = %d ,diff = %d",interval,diff);
                #endif              
                #ifdef HAS_PA_LNA
                    #ifdef EU_LBT
                        BeginClearChannelAssessment();
                    #endif
                #endif
                cli();
                frameReceived = false;  // reset the flag saying a frame has been received
                sei();
                missingPackets = 0;  // reset the number of consecutive missing packets
                t_out = 1;           // set timeout on 1 because next packet should be within 7 mse
                t_tune = 500;
                FrameType = (RxData[0] & 0x07);
                if (FrameType != TLM_PACKET)
                { //only when no uplink telemetry
                    if ((RxData[3] & 0x3F) != MiLoStorage.rx_num)
                        break;//if other receiver with different modelID                   
                    #ifdef USE_WIFI
                        if (RxData[3] & 0x40)
                        { //receive Flag from tx to start wifi server
                            if (++countUntilWiFi >= 5) // we wait 5 consecutive WIFI request before starting
                            {
                                #ifdef HAS_PA_LNA
                                    SX1280_SetTxRxMode(TXRX_OFF);//stop PA/LNA to reduce current before starting WiFi
                                #endif
                                SX1280_SetMode(SX1280_MODE_SLEEP);//start sleep mode to reduce SX120 current before starting WiFi
                                timer0_detachInterrupt();//timer0 is needed for wifi
                                detachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin));
                                uint32_t Now = millis();
                                WIFI_start();
                                while(1){
                                    WIFI_event();
                                    if ((millis() - Now)>= 50) 
                                    {
                                        Now = millis();
                                        LED_toggle;
                                    }
                                }
                            }
                        } else {
                            countUntilWiFi = 0; // reset the counter because we have to expect 5 consecutive WIFI frame
                        }

                    #endif
                }

                #ifdef TELEMETRY
                    if (FrameType == TLM_PACKET)
                    {
                        if ((RxData[3] & 0x0F) > 0)
                        { //Frame type uplink telemetry and no. of sport bytes >0
                            sportCount = (RxData[3] & 0x0F);
                            UplinkTlmId = (RxData[3] >> 4);
                            for (uint8_t i = 0; i <= sportCount; i++)
                                ReceivedSportData[i] = RxData[i + 3]; //transfer all sport telemetry data in a buffer including No. of bytes in sport telemetry (uplink frame)
                        }
                    }
                #endif
                
                if (++aPacketSeen > 10 ) //number packets received
                {
                    aPacketSeen = 10 ;
                }
                #if defined(TELEMETRY)
                    if ((FrameType != TLM_PACKET && (RxData[3] >> 7)== 1) || FrameType == TLM_PACKET )
                    { 
                        packetSeq = 1; // 1 means that next slot must be used to send a downlink telemetry packet
                    }
                #endif
                packetToDecode = true;//flag ,packet ready to decode (can be any type)              
                if (jumper == 0)
                    LED_on;
                all_off = 0; 
                break;
            }
        }
        #if defined MSW_SERIAL || defined SW_SERIAL
            callSportSwSerial();
        #endif  
    } // end while(1)
    
    // when we arrive here, it means that OR a frame has been received OR a time out occurs
    #if defined(TELEMETRY)
        if (packetSeq == 1 )
        { // next slot must be used to send a downlink telemetry packet.
            #ifdef HAS_PA_LNA
                #ifdef EU_LBT
                    if (!ChannelIsClear()) 
                        SX1280_SetOutputPower(MinPower);
                    else
                #endif
                SX1280_SetOutputPower(MaxPower);
            #endif
            MiloTlmSent();
            sbus_counter++;
            #ifdef STATISTIC
                if ( aPacketSeen > 5)
                {
                    packetCount = true;
                }
            #endif
            t_tune = 0;
            packetSeq = (packetSeq + 1) % 3;
        }
        else
    #endif
    {
        if (packetToDecode == true || missingPackets > 0)
        {
            sbus_counter++;
            t_tune = 500;
            #ifdef HAS_PA_LNA
                SX1280_SetTxRxMode(RX_EN);// do first to allow LNA stabilise
            #endif
            SX1280_SetMode(SX1280_MODE_RX);
            packetSeq = (packetSeq + 1) % 3;
        }
    }
    
    
    if (packetToDecode)
    {       // a frame has been received and must be decoded 
            // BIND_PACKET are discarded here because we wait for 5 consecutive frames and then we process them in another place
        if (FrameType != TLM_PACKET && FrameType != BIND_PACKET)
        {    
            c[0]  = (uint16_t)((RxData[4] | RxData[5]  << 8) & 0x07FF);
            c[1]  = (uint16_t)((RxData[5]  >> 3  |  RxData[6] << 5) & 0x07FF);
            c[2]  = (uint16_t)((RxData[6]  >> 6  | RxData[7]  << 2  | RxData[8] << 10) & 0x07FF);
            c[3]  = (uint16_t)((RxData[8]  >> 1 | RxData[9]  << 7) & 0x07FF);
            c[4]  = (uint16_t)((RxData[9]  >> 4 | RxData[10]  << 4) & 0x07FF);
            c[5]  = (uint16_t)((RxData[10]  >> 7 | RxData[11]  << 1  | RxData[12] << 9 ) & 0x07FF);
            c[6]  = (uint16_t)((RxData[12]  >> 2 | RxData[13] << 6) & 0x07FF);
            c[7]  = (uint16_t)((RxData[13] >> 5 | RxData[14] << 3) & 0x07FF);
            
            uint8_t j = 0;
            #if defined TX_FAILSAFE
                fs_started = false;
                static uint8_t chan = 7;
            #endif
            switch (FrameType)
            {
                case CH1_8_PACKET1:
                case CH1_8_PACKET2:
                    j = 0;
                    break;
                case CH1_16_PACKET:
                    j = 8;
                    break;
                #if defined TX_FAILSAFE
                case  FLSF_PACKET1:
                    fs_started = true;
                    setFSfromTx = true;
                    j = 0;
                    break;
                case  FLSF_PACKET2:
                    fs_started = true;
                    setFSfromTx = true;
                    j = 8;
                    break;
                #endif
                default:
                    j = 0;
                    break;
            }
            #if defined TX_FAILSAFE
                if (fs_started)
                    chan = (chan + 1) % 8;
            #endif
            
            for (uint8_t i = 0; i < 8; i++)
            {
                wordTemp = c[i];    
                #if defined TX_FAILSAFE
                    if (fs_started && i == chan)
                    {
                        MiLoStorage.FS_data[chan + j] = word_temp; //custom FS
                        //FS from TX is not saved in EEPROM!
                        #ifdef DEBUG_FS
                            if (fs_started)
                                debugln("FS_data = %d" , MiLoStorage.FS_data[chan + j]);
                        #endif
                    }
                    else
                #endif
                {
                    if (wordTemp > 800 && wordTemp < 2200)
                    {
                        ServoData[i + j] = wordTemp;
                        #ifdef DEBUG_DATA
                            debugln(" S2 = %d", ServoData[2]);//throttle
                        #endif
                        #if defined SBUS
                            channel[i+j] = (ServoData[i+j]-881)*1.6;//881-2159 to 0-2047
                            channel[i+j] = constrain(channel[i],0,2047);
                        #endif
                    }
                }
                
            }
            
            if (jumper
                    #ifdef TX_FAILSAFE
                    && fs_started == false
                    #endif
                    )
            {
                if (countFS++ >= MAX_MISSING_PKT)
                {
                    (countFS & 0x10) ? LED_off : LED_on;
                }
                if (countFS >= (2 * MAX_MISSING_PKT))
                {
                    detachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin));    
                    for (uint8_t i = 0; i < 16; i++)
                    {
                        if (MiLoStorage.FS_data[i] != ServoData[i]) //only changed values
                            EEPROMWriteInt(address + 4 + 2 * i, MiLoStorage.FS_data[i]);
                    }
                    EEPROM.commit();
                    attachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin), dioISR, RISING); //attach interrupt to DIO1 pin
                    jumper = 0;
                }
            }
        }
        #ifdef SPORT_TELEMETRY
            if (sportCount > 0)
            {
                for (uint8_t i = 1; i <= sportCount; i++)
                    smartPortDataReceive(ReceivedSportData[i]);// process all bytes of uplink tlm frame
                sportCount = 0;
            }
        #endif
        packetToDecode = false;// received packet has been decoded       
        #ifdef STATISTIC
            LQICalc();
        #endif
        #if defined(SBUS)
            SBUS_frame();
        #endif
    }
    
    #if defined  SBUS
        if (sbus_counter == 2)//sent out sbus on  every 14ms (timed by interval)
        { 
            sbus_counter = 0;
            if (all_off == 0)
            {
                for (uint8_t i = 0; i < TXBUFFER_SIZE; i++)
                {
                    Serial.write(sbus[i]);
                }
            }
            
        }
    #endif
    
    #if defined MSW_SERIAL || defined SW_SERIAL
        callSportSwSerial();// process Sport data that could be received from sensor.
    #endif
}

void   SetupTarget()
{
    pinMode(LED_pin, OUTPUT);
    pinMode(BIND_pin, INPUT);
    pinMode(SX1280_RST_pin , OUTPUT);
    pinMode(SX1280_BUSY_pin , INPUT);
    pinMode(SX1280_DIO1_pin , INPUT);
    pinMode(SX1280_CSN_pin , OUTPUT); 
    digitalWrite(SX1280_CSN_pin, HIGH);
    #ifdef DIVERSITY        
        pinMode(SX1280_ANTENNA_SELECT_pin , OUTPUT);    
        SX1280_ANT_SEL_on;
    #endif
    //SPI
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(10000000);
    #ifdef HAS_PA_LNA
        pinMode(SX1280_TXEN_pin , OUTPUT);
        pinMode(SX1280_RXEN_pin , OUTPUT);
    #endif
    EEPROM.begin(EEPROM_SIZE);
}

/*
void  MiLoRxBinding(uint8_t bind) {
    while (1)
    {
        if (bind == 0)
        {
            ReadEEPROMdata(address);//if flashing again new firmware need rebinding
            #if defined(DEBUG_EEPROM)
                delay(1000);
                debugln("txid1 = %d,txid2 = %d,rx_num = %d,chanskip = %d", MiLoStorage.txid[0], MiLoStorage.txid[1], MiLoStorage.rx_num, MiLoStorage.chanskip);
                debugln("MP_id = %d" ,MProtocol_id );
                //for(uint8_t i = 0 ;i < 16;i++)
                //{
                //   Serial.println(MiLoStorage.FS_data[i]);
                //}
            #endif
            #ifdef HC_BIND
                MProtocol_id = 7059696;
                MiLoStorage.rx_num = 0;
                MiLoStorage.txid[0] = 240;
                MiLoStorage.txid[1] = 184;          
                //MProtocol_id = 13788120;
                //MiLoStorage.rx_num = 0;
                //MiLoStorage.txid[0] = 216;
                //MiLoStorage.txid[1] = 99;        
            #endif
            is_in_binding = false;
            break;
        }
        else
        { //binding time
            digitalWrite(LED_pin, HIGH);
            MiLoRxBind();
        }
        yield();//shut-up WDT
    }
}
*/

void MiLoRxBind(void)
{
    LED_on;
    is_in_binding = true;
    #ifndef TUNE_FREQ
        currFreq = GetInitialFreq(); //set frequency first or an error will occur!!!
        SX1280_Begin();//config
        MiLo_SetRFLinkRate(RATE_BINDING);
    #endif
    SX1280_SetFrequencyReg(currFreq);
    SX1280_SetOutputPower(MinPower);
    #ifdef HAS_PA_LNA
        SX1280_SetTxRxMode(RX_EN);// do first to enable LNA
    #endif
    SX1280_SetMode(SX1280_MODE_RX);
    
    while (1)
    {
        if (frameReceived)
        {
            frameReceived = false;
            uint8_t const FIFOaddr = SX1280_GetRxBufferAddr();
            SX1280_ReadBuffer(FIFOaddr, RxData, PayloadLength);
            if ((RxData[0] & 0x07) == 0)
            { //bind frametype
                MiLoStorage.txid[0] = RxData[1] ;
                MiLoStorage.txid[1] = RxData[2];
                MiLoStorage.rx_num = RxData[5];
                MiLoStorage.chanskip = RxData[6];
                LoRaBandwidth = LORA_BW_0800;
                FreqCorrection = SX1280_GetFrequencyError();// get frequency offset in HZ
                FreqCorrection /= 1000;
                FreqCorrectionRegValue = SX1280_FREQ_MHZ_TO_REG(FreqCorrection);
                break;
            }
        }
        yield();//shut-up WDT
    }
    detachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin));    
    MProtocol_id = (RxData[1] | (RxData[2] << 8) | (RxData[3] << 16) | (RxData[4] << 24));
    #if defined(DEBUG_BIND)
        debugln("txid1 = %d,txid2= %d,rx_num = %d,chanskip = %d", MiLoStorage.txid[0], MiLoStorage.txid[1], MiLoStorage.rx_num, MiLoStorage.chanskip);
        //debugln("FreqCorr = %d ", FreqCorrection);
        //debugln("FreqCorrRegV = %d ", FreqCorrectionRegValue);
        debugln("MP_id = %d", MProtocol_id);
    #endif
    for (uint8_t i = 0; i < 16; i++)
        MiLoStorage.FS_data[i] = NO_PULSE;
    StoreEEPROMdata(address);
    while (1)
    {
        LED_on;
        delay(500);
        LED_off;
        delay(500);
    }
}

#ifdef SPORT_TELEMETRY
    uint8_t  ICACHE_RAM_ATTR MiLoTlm_append_sport_data(uint8_t *buf)
    {
        uint16_t next;
        uint8_t index = 0;
        
        if (TelemetryId == TelemetryExpectedId)
            idxOK = SportTail;//update read pointer to last ack'ed packet
        else
            SportTail = idxOK;
        
        TelemetryExpectedId = (TelemetryId + 1) & 0x1F;
        
        while (index < 10 )
        { //max 10 bytes in a frame
            if (SportTail == SportHead) //if no sport data ,no send, buffer empty
            {
                break;
            }
            buf[index] = SportData[SportTail];
            next = (SportTail + 1)&0x3F;
            SportTail = next;
            index += 1;
        }
        return index;
    }

    void  ConfigTimer()
    {
        noInterrupts();
        timer1_attachInterrupt(SportPollISR);
        timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
        timer1_write(60000); //12000 us     
        interrupts();
    }

    void  ICACHE_RAM_ATTR SportPollISR()
    {       
        if (sportMSPflag)
        {
            memcpy((void *)sTxData, sportMSPdatastuff, idxs);
            sendMSPpacket();
            sportMSPflag = 0;
        }
        else
            tx_sport_poll();
    }

#endif // end SPORT_TELEMETRY

#ifdef TELEMETRY
    void  ICACHE_RAM_ATTR MiLoTlm_build_frame()
    {
        uint8_t nbr_bytesIn;
        
        TelemetryId = (RxData[0] >> 3) & 0x1F;
        
        frame[0] = MiLoStorage.txid[0]; ;
        frame[1] = MiLoStorage.txid[1];
        frame[2] = MiLoTlmDataLink(pass);
        frame[3] = TelemetryId | (pass << 5);
        #ifdef SPORT_TELEMETRY
            nbr_bytesIn = MiLoTlm_append_sport_data(&frame[5]);
        #endif
        frame[4] = nbr_bytesIn | ((UplinkTlmId & 0x0F) << 4);
        #ifdef DEBUG_SPORT
            if(nbr_bytesIn){
                for(uint8_t i = 0;i<nbr_bytesIn;i++){
                    Serial.print(frame[i+4],HEX);
                    Serial.print(";");
                }
                Serial.println("");
            }
        #endif  
        pass = (pass + 1)%3;
    }

    void  ICACHE_RAM_ATTR MiloTlmSent()
    {
        MiLoTlm_build_frame();
        delayMicroseconds(50);//just in case
        #ifdef HAS_PA_LNA
            SX1280_SetTxRxMode(TX_EN);//PA enabled
        #endif
        SX1280_WriteBuffer(0x00, frame, PayloadLength); //
        SX1280_SetMode(SX1280_MODE_TX);
    }

    uint8_t  ICACHE_RAM_ATTR MiLoTlmDataLink(uint8_t bit)
    {
        static uint8_t link = 0 ;
        getRFlinkInfo();
        switch (bit)
        {
        case 0:
            if (antenna)
                link = MiLoStats.uplink_RSSI_2;//antenna
            else
                link = MiLoStats.uplink_RSSI_1;
            break;
        case 1:
            link = MiLoStats.uplink_SNR;
            break;
        case 2:
            link = MiLoStats.uplink_Link_quality;
            break;
        }
        return link;
    }
#endif // end TELEMETRY

#ifdef SPORT_TELEMETRY
    void  sportMSPstuff(uint8_t *p)//stuffing back
    {
        uint16_t crc_s = 0;
        uint8_t indx = 0;
        
        sportMSPdatastuff[indx++] = 0x7E;
        sportMSPdatastuff[indx++] = p[0];    
        for (uint8_t i = 1 ; i < 9; i++)
        {
            if (i == 8)
                p[i] = 0xff - crc_s;//this is the crc byte
            if (p[i] == 0x7D || p[i] == 0x7E)
            {
                sportMSPdatastuff[indx++] = 0x7D;
                sportMSPdatastuff[indx++] = p[i] ^ 0x20;
            }
            else
            {
                sportMSPdatastuff[indx++] = p[i];   ;
            }
            crc_s += p[i]; //0-1FF
            crc_s += crc_s >> 8; //0-100
            crc_s &= 0x00ff;
        }
        idxs = indx;
    }

    //Sports received from TX(MSP)
    void  smartPortDataReceive(uint8_t c)
    {
        static bool byteStuffing = false;
        if (c == 0x7E)
        {
            smartPortRxBytes = 0;
            skipUntilStart = false;
            return;
        }
        else if (skipUntilStart)
        {
            return;
        }
        uint8_t* rxBuffer = (uint8_t*)&sportMSPdata;
        
        if (smartPortRxBytes == 0)
        {
            if (c)
                rxBuffer[smartPortRxBytes++] = c;
            else
                skipUntilStart = true;
        }
        else
        {
            if (c == 0x7D)
            {
                byteStuffing = true;
                return;
            }
            if (byteStuffing)
            {
                c ^= 0x20;
                byteStuffing = false;
            }
            rxBuffer[smartPortRxBytes++] = c;
            if (smartPortRxBytes >= 8)
            {
                #ifdef DEBUG_MSP
                    debugln("sportMSPdata = %d,smartPortRxBytes= %d" sportMSPdata[smartPortRxBytes],smartPortRxBytes);
                #endif
                #ifdef HAS_LUA
                    //if(checkRxconfigfromTX()){}//future dev.this are configurations from TX to RX(LUA/basic scripts.)
                    //else
                #endif
                {
                    cli();
                    sportMSPflag = true;
                    sportMSPstuff(sportMSPdata);
                    sei();
                }
                smartPortRxBytes = 0;
                skipUntilStart = true;
            }
        }
    }

    #ifdef MSW_SERIAL
        void  ICACHE_RAM_ATTR callSportSwSerial(){
            cli();
            sport_index = sportindex;
            if (sport_index >= 8) {
                if ((micros() - sportStuffTime) > 500){//If not receiving any new sport data in 500us
                    disable_interrupt_serial_pin();//no need to keep interrupt on serial pin after transferring all data    
                    memcpy((void*)sRxData,(const void*)sportbuff,sport_index);
                    processSportflag = true;                
                }
            }
            sei();
            if(processSportflag){
                ProcessSportData();
                processSportflag = false;
            } 
        }

    #elif defined SW_SERIAL
        void  ICACHE_RAM_ATTR callSportSwSerial()
        {
            uint8_t c;
            while (swSer.available() > 0)
            {
                c = swSer.read();
                if ( c == START_STOP)      // reset the buffer when we get a 0x7E
                    sport_index = 0;
                if (sport_index < 16)
                    sRxData[sport_index++] = c;
                if (sport_index >= 8)
                    sportStuffTime = micros();
            }
            if (sport_index >= 8) {
                if ((micros() - sportStuffTime) > 500)//If not receive any new sport data in 500us
                    ProcessSportData();
            }
        }
    #endif //SPORT SERIAL

#endif //end SPORT_TELEMETRY

uint8_t bind_jumper(void)
{
    pinMode(BIND_pin, INPUT_PULLUP);
    if ( digitalRead(BIND_pin) == LOW)
    {
        return 1;
    }
    return  0;
}


void ICACHE_RAM_ATTR dioISR()
{
    uint16_t irqStatus = SX1280_GetIrqStatus();
    #ifdef DEBUG_LOOP_TIMING
        //callMicrosSerial();
    #endif
    SX1280_ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if (irqStatus & SX1280_IRQ_TX_DONE)
    {
        #ifdef HAS_PA_LNA
            SX1280_SetTxRxMode(TXRX_OFF);
        #endif
        currOpmode = SX1280_MODE_FS; // radio goes to FS after TX
    }
    else if (irqStatus & (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT))
    {
        uint8_t const fail =
            ((irqStatus & SX1280_IRQ_CRC_ERROR) ? SX1280_RX_CRC_FAIL : SX1280_RX_OK) +
            ((irqStatus & SX1280_IRQ_RX_TX_TIMEOUT) ? SX1280_RX_TIMEOUT : SX1280_RX_OK);
        // In continuous receive mode, the device stays in Rx mode
        if (timeout != 0xFFFF)
        {
            // From table 11-28, pg 81 datasheet rev 3.2
            // upon successsful receipt, when the timer is active or in single mode, it returns to STDBY_RC
            // but because we have AUTO_FS enabled we automatically transition to state SX1280_MODE_FS
            currOpmode = SX1280_MODE_FS;
        }
        if (fail == SX1280_RX_OK){
            frameReceived  = true ;
        }       
        if (irqStatus & SX1280_IRQ_CRC_ERROR)
        {
            #ifdef STATISTIC
                TotalCrcErrors += 1 ;//bad packets that not pass the crc check
            #endif
        }   
    }

}

#ifdef STATISTIC
    void LQICalc(){
        uint8_t oldDropBit = DropHistory[DropHistoryIndex];
        DropHistory[DropHistoryIndex] = ThisPacketDropped ;
        if ( ++DropHistoryIndex >= 100 )
        {
            DropHistoryIndex = 0 ;
        }
        DropHistoryPercent += ThisPacketDropped ;
        DropHistoryPercent -= oldDropBit ;
        ThisPacketDropped = 0 ;
        if ( ++DropHistorySend >= 30 )
        {
            if (DropHistoryPercent < 100)
                uplinkLQ = (100 - DropHistoryPercent ) ;
            DropHistorySend = 0 ;
        }   
}
#endif // end STATISTIC

#ifdef TUNE_FREQ
    void  Sx1280_FreqTunning()
    {
        currFreq = GetCurrFreq(); //set frequency first or an error will occur!!! this the reg value not actual freq
        SX1280_Begin();//config
        MiLo_SetRFLinkRate(RATE_BINDING);
        SX1280_SetOutputPower(MinPower);
        #ifdef HAS_PA_LNA
            SX1280_SetTxRxMode(RX_EN);// do first to enable LNA
        #endif
        Sx1280_SetMode(SX1280_MODE_RX);
        uint32_t  t_tune = millis();
        FreqCorrection = 0;
        int32_t freqTemp = 0;
        while (1)
        {
            if ((millis() - t_tune) > 50)
            {
                t_tune = millis();
                if (FreqCorrection)
                    SX1280_SetFrequencyReg(currFreq - FreqCorrectionRegValue);
                else
                    SX1280_SetFrequencyReg(currFreq + FreqCorrectionRegValue);
                SX1280_SetMode(SX1280_MODE_RX);
            }
            if (frameReceived)
            {
                if (RxData[0] & 0x07) == BIND_PACKET)
                {
                    FreqCorrection = SX1280_GetFrequencyError();// get frequency offset in HZ    
                    if (FreqCorrection < 0)
                        freqTemp = - FreqCorrection;
                    FreqCorrectionRegValue = SX1280_FREQ_MHZ_TO_REG((double)freqTemp / 1.0E3);
                    frameReceived  = false;
                    break;
                }
            }
        }
        
        #if defined(DEBUG_FREQ)
            Serial.println(FreqCorrection);
            Serial.println(FreqCorrectionRegValue);
        #endif
    }
#endif
