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


// to do : add CRC to the data stored in EEPROM; when reading EEPROM, if CRC is wrong use default values.
// test failsafe and sbus
// in main while loop, avoid calling some functions if we are closed to the end of the timeout (to ensure handling timeout as soon as possible)



#undef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR IRAM_ATTR
#define ICACHE_RAM_ATTR3
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))

#include <SPI.h>
#include "_config.h"
#include "pins.h"
#include "iface_sx1280.h"
#include "MiLo_FHSS.h"
#include "SX1280.h"

#if defined DEBUG_HELP_FUNCTIONS
    void callMicrosSerial(){
        static uint32_t tim = 0 ;
        static uint32_t timt = 0 ;  
        timt = micros();
        Serial.println(timt - tim);
        tim = micros();
    }
    int32_t debugMicrosInterval(uint8_t idx){
        static uint32_t prevTime[10] = {0}; 
        uint32_t currentTime , savedPrevTime;
        if (idx >=10) return -1;
        savedPrevTime = prevTime[idx];
        currentTime = micros();
        prevTime[idx] = currentTime;
        return currentTime - savedPrevTime;
    }
    uint32_t debugStartedAt[10];
    void debugStartMicros(uint8_t idx){
        if ( idx < 10) debugStartedAt[idx] = micros();
    }
    uint32_t debugIntervalSinceStart(uint8_t idx){  // return 0 when idx is not valid
        if ( idx < 10) return micros() - debugStartedAt[idx];
        return 0;
    }
#endif

#define RATE_DEFAULT 0
#define RATE_BINDING 0
#define RATE_100HZ 1 //100HZ
#define RATE_150HZ 0 //150HZ
#define RATE_MAX 3

//Failsafe
#ifdef TX_FAILSAFE
    bool setFSfromTx = false;
#endif
uint32_t lastReceivedRcFrameMicros = 0;
#define FAILSAFE_INTERVAL 1000 // ms
uint16_t countFS = 0; // used to blink the led when pressing the button to reset the failsafe values

//EEPROM
uint32_t  address = 0;//EEPROM start adress

enum{ // types of packet being received from TX
        BIND_PACKET = 0,
        CH1_8_PACKET, //  channels 1-8
        CH9_16_PACKET, // channels 9-16
        TLM_PACKET,
        FS1_8_PACKET,  //  failsafe values for channels 1-8
        FS9_16_PACKET, //  failsafe values for channels 9-16
    };

//Channels
#define NO_PULSE  0
#define HOLD  2047
#define MAX_MISSING_PKT 100 // when missingPackets reachs this value, failsafe is ON and connection is lost
#define MARGIN_LONG_TIMEOUT 10000 // margin (usec) to be added to the timeout used when not connected (to cover clock difference between TX and RX)
#define MARGIN_SHORT_TIMEOUT 500 // margin (usec) to be added to the first timeout used when connected 

uint16_t ServoData[16]; // rc channels values used for ppm (generated from sbusChannel[])
volatile int32_t missingPackets = 0; // number of consecutive missing packets
bool packetToDecode = false; // true means that a valid packet (any type with Rc or FS channels ot uplinl tlm) has been received from Tx and must be decoded
uint8_t jumper = 0;

bool isConnected2Tx = false; // true means that the RX got a RC/FS channels or a uplink tlm packet that allows synchronization 
//uint8_t t_out = FHSS_CHANNELS_NUM; // replaced by a flag isConnected2Tx
uint32_t t_outMicros ;  // time out for main while() loop
uint32_t t_tune = 500;

bool sbusAllowed = false;   // true means sbus/pwm can be generated because at least 1 rc signal has been received (and if failsafe values are applied, there is no NO PULSE)
uint32_t slotBeginAt;       // theoretical timestamp when current slot starts 
uint16_t LED_count = 0;     // used to toggle the LED when not connected
uint8_t RxData[NBR_BYTES_IN_PACKET]; // store the packet received by SX1280
uint32_t bindingTime = 0;
uint32_t MProtocol_id = 0;  // unique ID provide by TX during binding; used to generate FHSS and to identify the sender of RX packets
uint8_t PayloadLength = NBR_BYTES_IN_PACKET; // number of bytes in a packet
uint8_t FrameType = BIND_PACKET;
uint32_t LastReceivedPacketTime;
uint32_t lastYieldMicros;
int32_t smoothedInterval;          // interval in us seconds that corresponds to that frequency
uint32_t microsInDioISR ;         // timestamp of processing the DIO interrupt occured
uint32_t dioISRMicros;            // saved value of microsInDioISR when a frame with CRC OK is received for processing if frame is really valid
volatile bool dioOccured = false ;     // true when a dio1 interrupt occurs
bool frameReceived = false;           // becomes true when a frame with good CRC has been received

//Debug
uint32_t debugTimer;
char debug_buf[64];

uint8_t packetSeq = 0;  // count the slots in a sequence 0, 1, 2; 1 means that next slot should be used for a downlink tlm frame 
                        // value is forced when a valid frame is received, it is increased by 1 in case of timeout when connected
                        // it is not used when not connected
bool startWifi = false;

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
uint8_t aPacketSeen = 0;  // count (up to 10) the number of packets that have been received
uint32_t TotalCrcErrors ;
uint8_t DropHistory[100] ;
uint8_t DropHistoryPercent ;
uint8_t antenna = 0;

uint8_t countUntilWiFi = 0;
    
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

uint8_t downlinkTlmId;
    
//TELEMETRY
#ifdef TELEMETRY
    uint8_t frame[NBR_BYTES_IN_PACKET];// frame to be sent to TX
    uint32_t tlmDataLinkType = 0;
    uint8_t dwnlnkTlmExpectedId;
    uint8_t uplinkTlmId = 0;
    volatile bool sportPollIsrFlag = false ; // flag to inform main loop that sport polling timer fire.
    
    // to send uplink tlm to sensor
    struct sportMsp_t{
        uint8_t len;        // number of used bytes in frame
        uint8_t frame[16];  // contains a frame ready to be sent to sensor (START + stuff bytes + CRC)
    };
    sportMsp_t sportMspData[4];
    uint8_t sportMspTail = 0; // index in the array where to get data
    uint8_t sportMspHead = 0; // index in the array where to put data
    uint8_t sportMspCount = 0; // number of used items in the array
    
#endif

#define NOP() __asm__ __volatile__("nop")

void   SetupTarget();
void ICACHE_RAM_ATTR dioISR();
void ICACHE_RAM_ATTR3 callSportSwSerial(void);
void ICACHE_RAM_ATTR SportPollISR(void);
void ICACHE_RAM_ATTR3 MiloTlmSent(void);
void ICACHE_RAM_ATTR3 MiLoTlm_build_frame();
//void ICACHE_RAM_ATTR3 MiLoTlm_append_sport_data(uint8_t *buf);
uint8_t ICACHE_RAM_ATTR3 MiLoTlmDataLink(uint8_t pas);
void  ConfigTimer();
void SX1280_SetTxRxMode(uint8_t mode);
void  smartPortDataReceive(uint8_t c);
void  ICACHE_RAM_ATTR3 handleSportPoll();

uint8_t bind_jumper(void);

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
uint16_t timeout = 0xFFFF; // this means that the SX1280 runs in continous mode
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
    {0, RADIO_TYPE_SX128x_LORA, RATE_LORA_150HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF6,
      SX1280_LORA_CR_LI_4_7, 7000, TLM_RATIO_1_3, 12, NBR_BYTES_IN_PACKET },
    {1, RADIO_TYPE_SX128x_LORA, RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,
      SX1280_LORA_CR_LI_4_6, 9000, TLM_RATIO_1_3, 12, NBR_BYTES_IN_PACKET}
};

MiLo_rf_pref_params_s MiLo_AirRateRFperf[RATE_MAX] =
{
    {0, RATE_LORA_150HZ,  -108,  5060, 3500, 2500},
    {1, RATE_LORA_100HZ,  -112,  7605, 3500, 2500}
};

void  ICACHE_RAM_ATTR3 MiLo_SetRFLinkRate(uint8_t index) // Set speed of RF link (hz) index values
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
    
    #if defined(DEBUG_WITH_SERIAL_PRINT) // in this case, SBUS is disabled (in _config.h)
        Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
        delay(3000); // added to have time to get first msg on arduino IDE terminal
        Serial.println("Starting");       
    #endif
    #ifdef DEBUG_ON_GPIO1
      pinMode(1 , OUTPUT);
      G1OFF;
      G1PULSE(5); delay(5);G1PULSE(5); delay(5);G1PULSE(5); 
    #endif
    
    #ifdef DEBUG_ON_GPIO3
      pinMode(3 , OUTPUT);
      G3OFF;

    #endif
    #ifdef SBUS
        init_SBUS(); // sbus uses Serial.begin with inverted signal
    #endif

    #if defined SPORT_TELEMETRY
        initSportUart();// sport to sensort uses SW serial with the same pin for TX and Rx, signal is inverted
        #ifndef DEBUG_ON_GPIO3
            pinMode(SPORT_pin,INPUT);
        #endif
        sportHead = sportTail = 0;
    #endif
    
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
    #ifdef DEBUG_FHSS
    // we generate several fhss sequence just to see if the aglo is OK
        for (uint8_t i ; i< 20; i++){ 
            Fhss_Init();// setup some fhss fixed variables
            Fhss_generate(MProtocol_id+i*1000); // generates the used frequency (based on param in EEPROM)
            yield();
        }
    #endif
    Fhss_Init();// setup some fhss fixed variables
    Fhss_generate(MProtocol_id); // generates the used frequency (based on param in EEPROM)
    isConnected2Tx = false; // previously it was : t_out= FHSS_CHANNELS_NUM; // whe have to start trying to get a connection
    currFreq = GetCurrFreq(); //set frequency first or an error will occur!!!
    bool init_success = SX1280_Begin();
    if (!init_success)
    {
        debugln("Init of SX1280 failed");
        while(1){};// while added by mstrens to block the MCU; comment line in order to start testing without a SX1280 module
    }
    sbusAllowed = false ;    //no pulse
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
        ConfigTimer();  // start timer 1 to send the polling request or the uplink tlm frame to the sensor every 12 msec 
    #endif
    smoothedInterval = MiLo_currAirRate_Modparams->interval;
    t_outMicros= FHSS_CHANNELS_NUM * smoothedInterval * 3 / 2 + MARGIN_LONG_TIMEOUT; // at start up we use a long delay (476msev = 7000*68 usec) 
    slotBeginAt = micros();
    debugln("End of setup: SX1280 is OK");
    
} // end setup()

void applyFailsafe() 
// put failsafe converted values in ServoData[] for ppm and original failsafe values in sbusChannel[] for Sbus
{
    uint16_t word;
    for (uint8_t i = 0; i < 16; i++)
    {
        word = MiLoStorage.FS_data[i];
        if (word == NO_PULSE) //no pulse
        {
            ServoData[i] = 0;
            sbusAllowed = false; // false means that pulses/sbus may not be generated
        }
        else if (word == HOLD) //2047 equivalent is hold value so no update data.
        {
            //do nothing
        }
        else
        {
            ServoData[i] = (((word<<2)+word)>>3)+860; //value range 860<->2140 -125%<->+125%             
            sbusChannel[i] = word;
        }
    }
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

    if (jumper)//jumper = 1 when button to reset failsafe has been activated
        countFS ++; // used for led blinking
    
    if ( packetSeq == 1) { // if timeout occurs when we where in a slot for downlink 
                            // we have to go back in receive mode
        #ifdef HAS_PA_LNA
            SX1280_SetTxRxMode(RX_EN);// do first to allow LNA stabilise
        #endif
        SX1280_SetMode(SX1280_MODE_RX);
    } else {
        missingPackets++;
        #ifdef STATISTIC
            LQICalc(1);  // count this packet as dropped
        #endif    
    }
    if (missingPackets > MAX_MISSING_PKT)// we just lost the connection
    {
        #if defined(FAILSAFE)
            applyFailsafe() ; // put failsafe values in ServoData[] and sbusChannel[]
        #endif
        isConnected2Tx = false;// wait max 37 slots of 7 msec before exiting while()
        t_outMicros = FHSS_CHANNELS_NUM * smoothedInterval * 3 / 2 + MARGIN_LONG_TIMEOUT; // 2000 is to be sure that interval is big enoug to cover 37 slots
        countFS = 0;    
        //packetSeq = 0;
        uplinkLQ = 0;
        setChannelIdx(0); // when connection is lost we go back to the first channel (to listen on a channel that exist 2X in the fhss list)
        G3PULSE(1);// 
        SX1280_SetFrequencyReg(GetCurrFreq());    
    } else {
        packetSeq = (packetSeq + 1) %3; // on each short time out we increase packetSeq
        if ( packetSeq != 1) { // skip on next channel but only if next slot will not be a downlink 
            nextChannel(1);     
            G3PULSE(2);
            SX1280_SetFrequencyReg(GetCurrFreq());
        }       
    }    
}

void handleLongTimeout()
{ // when 37 *time interval expires without receiving a frame, process LED, failsafe setup and frequency hop. 
    if (jumper == 0) 
    { // When button is not pressed toggle LED to show that link is lost
        LED_count++;
        if (LED_pin != -1) {
            (LED_count & 0x02) ? LED_on : LED_off;
        }    
    }
    else
    { //when button is pressed while Rx is not connected, Failsafe are reset on 0 (= no pulse) 
        uint8_t n = 10;
        while (n--)
        { //fast blinking until resetting  FS data from RX
            if ( LED_pin != -1) LED_toggle;
            delay(100); //blink LED
        }
        bool  saveFsToEprom = false;
        for (uint8_t i = 0 ; i < 16; i++)
        {
            if (MiLoStorage.FS_data[i] != 0)
            {
                MiLoStorage.FS_data[i] = 0;//reset the FS values while RX is wating to connect                                             
                EEPROMWriteInt(address + 4 + 2 * i, MiLoStorage.FS_data[i]);                       
                saveFsToEprom = true;
            }
        }                   
        if (saveFsToEprom){
            noInterrupts();
            EEPROM.commit();
            interrupts();
        }    
        jumper = 0;
    }
    G3PULSE(1);
    nextChannel(1); // frequency hop after 68 slots. So Rx stays listening on the same channel while Tx hop every slot 
                    // note: when we are trying to (re)synchronize RX with Tx, we use only the n first channels in the list (this is performed in the function
    SX1280_SetFrequencyReg(GetCurrFreq());
    SX1280_SetMode(SX1280_MODE_FS);//  This help perhaps in case a frame is just being received when time out occurs 
    SX1280_ClearIrqStatus(0XFFFF); //  This is to avoid that we perform here a frequency hop and another just after  
    SX1280_SetMode(SX1280_MODE_RX);//      because frameReceived would be true
    cli();
    dioOccured = false;  // reset the flag saying a DIO interrupt occured
    sei();
}

bool isReceivedFrameValid() { // return true for a valid frame
    // when debugging with pulses, the number of pulses says the reason of reject.
    uint8_t const FIFOaddr = SX1280_GetRxBufferAddr();
    SX1280_ReadBuffer(FIFOaddr, RxData, PayloadLength);
    SX1280_GetLastPacketStats();
    frameReceived = false;  // reset the flag saying a frame with good CRC has been received
    if ((RxData[1] != MiLoStorage.txid[0]) || (RxData[2] != MiLoStorage.txid[1]) ||((RxData[3] & 0x3F) != MiLoStorage.rx_num))
    { // Only if correct txid and rx_num will pass
        G3PULSE(1);
        return false;
    }    
    FrameType = (RxData[0] & 0x07) ;   // extract the frame type (will be used for many checks later on) 
    if ( isConnected2Tx ) {  // when connected 
        if ( getCurrentChannelIdx() != (RxData[15] & 0x3F )) {
            G3PULSE(1);G3PULSE(1);G3PULSE(1);
            return false ;  // reject frame when channel in frame is not the expected one
        }       
    } else { // when not connected = we try to synchronise RX with TX 
        if ( getCurrentChannelIdx() == (RxData[15] & 0x3F )) return true ;
        if ( (getCurrentChannelIdx() == 0) && ( (RxData[15] & 0x3F ) == 18) ){ // synchro at mid channel
            setChannelIdx(18); // synchronize on indx 18
            SX1280_SetFrequencyReg(GetCurrFreq()); // adapt frequency (perhaps we do not perform a freq hop immediately)
            return true ;
        }
        if ( (getCurrentChannelIdx() == 18) && ( (RxData[15] & 0x3F ) == 0) ){ // synchro on first channel
            setChannelIdx(0); // synchronize on indx 0
            SX1280_SetFrequencyReg(GetCurrFreq()); // adapt frequency (perhaps we do not perform a freq hop immediately)
            return true ;
        } 
        G3PULSE(1);G3PULSE(1);G3PULSE(1);G3PULSE(1);G3PULSE(1);
        return false; // reject frames when channel does not match 
    }   
    return true;                 
}                           

void prepareNextSlot() { // a valid frame has been received; perform frequency hop and initialized some flags/counters
    isConnected2Tx = true; // previously it was t_out = 1;           // set timeout on 1 because next packet should be within 7 mse
    t_outMicros = smoothedInterval + MARGIN_SHORT_TIMEOUT; // MARGIN_SHORT_TIMEOUT is a margin e.g. to take care of some jitter   
    if ((FrameType != TLM_PACKET && (RxData[3] >> 7)== 1) || FrameType == TLM_PACKET ) { 
        packetSeq = 1; // 1 means that next slot must be used to send a downlink telemetry packet
    } else { // when next frame is not a downlink, we skip to next channel
        packetSeq = 0; // 0 means that we got a first RC channel and so next slot will NOT be downlink telemetry packet
        nextChannel(1);
        G3PULSE(1);
        SX1280_SetFrequencyReg(GetCurrFreq());
    }
    #ifdef HAS_PA_LNA
        #ifdef EU_LBT
            BeginClearChannelAssessment();
        #endif
    #endif
    missingPackets = 0;  // reset the number of consecutive missing packets
    if (aPacketSeen < 10 )  aPacketSeen++ ;  // increase number of packets up to 10
    if (jumper == 0){
        if(LED_pin != -1) LED_on;
    }    
}

#ifdef USE_WIFI
    void checkStartWIFI()
    { // this is done only for for valid frame other that TLM frame    
        if (RxData[3] & 0x40)  
        { //receive Flag from tx asking to start wifi server
            if (++countUntilWiFi >= 5) // we wait 5 consecutive WIFI request before starting
            {
                #ifdef HAS_PA_LNA
                    SX1280_SetTxRxMode(TXRX_OFF);//stop PA/LNA to reduce current before starting WiFi
                #endif
                SX1280_SetMode(SX1280_MODE_SLEEP);//start sleep mode to reduce SX120 current before starting WiFi
                timer0_detachInterrupt();//timer0 is needed for wifi
                detachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin));
                timer1_detachInterrupt();
                uint32_t Now = millis();
                WIFI_start();
                while(1){
                    WIFI_event();
                    if ((millis() - Now)>= 50) 
                    {
                        Now = millis();
                        if ( LED_pin != -1) LED_toggle;
                    }
                }
            }
        } else { // we do not recived the flg asking to start WIFI
            countUntilWiFi = 0; // reset the counter because we have toexpect 5 consecutive WIFI frame
        }
    }    
#endif


void saveRcFrame() {
    // save the channels values in 
    
    uint16_t c[8];    
    //uint16_t wordTemp;
    // process a valid RC frame (can be a frame with failsafe data)
    c[0]  = (uint16_t)((RxData[4] | RxData[5]  << 8) & 0x07FF);
    c[1]  = (uint16_t)((RxData[5]  >> 3  |  RxData[6] << 5) & 0x07FF);
    c[2]  = (uint16_t)((RxData[6]  >> 6  | RxData[7]  << 2  | RxData[8] << 10) & 0x07FF);
    c[3]  = (uint16_t)((RxData[8]  >> 1 | RxData[9]  << 7) & 0x07FF);
    c[4]  = (uint16_t)((RxData[9]  >> 4 | RxData[10]  << 4) & 0x07FF);
    c[5]  = (uint16_t)((RxData[10]  >> 7 | RxData[11]  << 1  | RxData[12] << 9 ) & 0x07FF);
    c[6]  = (uint16_t)((RxData[12]  >> 2 | RxData[13] << 6) & 0x07FF);
    c[7]  = (uint16_t)((RxData[13] >> 5 | RxData[14] << 3) & 0x07FF);

    uint8_t j = 0;
    if ( (FrameType == CH9_16_PACKET) || (FrameType == FS9_16_PACKET) ) j = 8;
    
    #if defined TX_FAILSAFE
        if ( (FrameType == FS1_8_PACKET) || (FrameType == FS9_16_PACKET) ) 
        { // use failsafe but do not store them in EEPROM 
            setFSfromTx = true;  // this will avoid setting jumper = 1 and so avoid saving failsafe value with the Rx button
            for (uint8_t i = 0; i < 8; i++) {
                MiLoStorage.FS_data[i+j] = c[i];    
                #ifdef DEBUG_FS
                    debugln("FS_data channel %d  = %d" , i+j+1, MiLoStorage.FS_data[i + j]);
                #endif
            }
        }        
    #endif
    if ( (FrameType == CH1_8_PACKET) || (FrameType == CH9_16_PACKET) ) 
    { // we store always the values for Sbus in sbusChannel[] because we need all 16 values somewhere when failsafe is set with the button
        memcpy( &sbusChannel[j], &c[0], 8) ; // copy into SBUS
        #ifdef DEBUG_RC_CHANNEL_DATA
            debug("frame= %d , " , FrameType);
            for (uint8_t i = 0; i < 1; i++) {  // i max set to 1 to test; could be increase to 8
                debug(" %d ; ", sbusChannel[i]);
            }
            debugln(" ");    
        #endif
        // we save them to ServoData when PWM is used; we convert the range  
        #ifdef PWM_SERVO    
            for (uint8_t i = 0; i < 8; i++) {
                ServoData[i + j] = (((c[i]<<2)+c[i])>>3)+860;	//value range 860<->2140 -125%<->+125%
                #ifdef DEBUG_SERVODATA
                    debugln("servo data channel %d = %d", i+j+1 , ServoData[i+j]);
                #endif                
            }
        #endif
        if (jumper)
        {  // when button is pressed, while connected, the Rc channels are stored in EEPROM as failsafe values after blinking  
            if (countFS++ >= MAX_MISSING_PKT)
            { 
                if(LED_pin != -1) (countFS & 0x10) ? LED_off : LED_on;
            }
            if (countFS >= (2 * MAX_MISSING_PKT))
            {
                for (uint8_t i = 0; i < 16; i++)
                {
                    if (MiLoStorage.FS_data[i] != sbusChannel[i]) //only changed values
                        EEPROMWriteInt(address + 4 + 2 * i, MiLoStorage.FS_data[i]);
                }
                noInterrupts();
                EEPROM.commit();
                interrupts();
                jumper = 0;
            }
        } // end of jumper    

    } // end of CH.-._PACKET
} // end saveRcFrame

void handleDio1() {  
    // check the SX1280 irq flags and return true if a frame has been received with CRC OK 
    uint16_t irqStatus = SX1280_GetIrqStatus();
    SX1280_ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    frameReceived = false;
    if (irqStatus & (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT))
    {
        if (irqStatus & SX1280_IRQ_CRC_ERROR) {
            G3PULSE(1);G3PULSE(3); G3PULSE(1);
            #ifdef STATISTIC
                TotalCrcErrors += 1 ;//bad packets that not pass the crc check
            #endif
        }
        else if ( irqStatus & SX1280_IRQ_RX_TX_TIMEOUT) {
            // should not happens because timeout is not used in RX continous mode
        }
        else {
            frameReceived = true ;
            dioISRMicros = microsInDioISR ; // keep the timestamp
        }
        return ;   
    }

}

void decodeSX1280Packet() { // handle a valid frame (RcData or uplink tlm)
    #ifdef USE_WIFI
        if (FrameType  != TLM_PACKET) 
        { // If WIFI is requested in the frame, we start it (after 5 consecutive wifi frames); then we do not exit
            checkStartWIFI();
        }      
    #endif
    #ifdef TELEMETRY 
        if (FrameType == TLM_PACKET) // process uplink tlm frame
        {
            downlinkTlmId = (RxData[0] & 0X30) >> 4 ; // bits 5..4 = downlinkTlmId
            if( ( (RxData[3] & 0XC0) >> 6 ) == uplinkTlmId ) { 
                // we receive a message with the expected sequence, so we process it
                uplinkTlmId++;
                // format the packet (add START + stuffing + CRC) in sportMspData[4]  
                #ifdef DEBUG_UPLINK_TLM_DATA
                    debug("frame= %d , " , FrameType);
                    for (uint8_t i = 4; i < 5; i++) {  // change i and max to print the full data 
                        debug(" %d ; ", RxData[4+i]);
                    }
                    debugln(" ");    
                #endif
                sportMSPstuff(&RxData[4]) ; // process 8 bytes starting from RxData and add them to sportMspData[]
                // update sportMspData[].frame, sportMspData[].len and sportMspTail, sportMspHead and sportMspCount
            }    
        }
    #endif
    if (FrameType != TLM_PACKET && FrameType != BIND_PACKET)
    {     // BIND_PACKET are discarded here because we wait for 5 consecutive frames and then we process them in another place
        sbusAllowed = true; // as we received a valid frame we can allow generating Sbus and PWM ;
        lastReceivedRcFrameMicros = micros(); // used to activate Failsafe when there is no RC packet since X mmsec
        downlinkTlmId = (RxData[0] & 0X30) >> 4 ; // bits 5..4 = downlinkTlmId
        saveRcFrame(); // save data from Rc channels or failsafe (those are not stored in EEPROM but sent at 9sec interval by handset to TX)
                       // if SBUS is defined, data are stored also in sbusChannel[] but frame is not yet generated 
        //SBUS_frame(); // create frame mainly based on sbusChannel[]
    }
}  

//++++++++++++++++++++++++++++++++++++++++   main loop   +++++++++++++++++++++++++++++++++++ 
void loop()
{
    yield();
    lastYieldMicros = micros();
    if (bind_jumper() && jumper == 0// when button is pushed and previously it was not pressed
            #ifdef TX_FAILSAFE
            && setFSfromTx == false  // and no failsafe data have been received from TX
            #endif
            )
        if (countFS == 0) //only at boot reset (Failsafe) no reset of failsafe values while the rx is bound
        {
            countFS = 1;
            jumper = 1;
        }
    if ((millis() - bindingTime) > 20000 && aPacketSeen == 0)
    { //Perform automatically a bind 20 sec after startup if a packet has not yet been received
        timer0_detachInterrupt();//timer0 is needed for wifi
        MiLoRxBind();  // note : we never exit this process: when binding is done, led is blinking and RX must be powerwed off and on again
    }
    G3PULSE(50); // to debug when we enter a new while loop
    while (1) // exit only on timeout or when a valid frame is received
    {
        if (dioOccured)
        { // an interrupt occured on DIO1
            cli();
            dioOccured = false; //reset the flag set in ISR  
            sei();
            handleDio1(); // handle interrupt ( reset inyterrupt, read SX1280 incoming data and set frameReceived to true)     
            if (frameReceived) 
            {             // a frame has been received from the TX with a good CRC (flag has been set in DioISR)
                G3PULSE(5); // to debug when a frame has been received
                if ( isReceivedFrameValid()) { //check if frame is valid
                    G3PULSE(10);// to debug when a frame is valid
                    slotBeginAt = dioISRMicros ;  // resynchronise RX on TX
                    prepareNextSlot(); //  we first prepare next slot (e.g. frequency hop if allowed ) and update some flags/counters
                    decodeSX1280Packet();
                    break; // exit while(1)
                } else {
                    Serial.println("Not valid");
                }
            }
        }
        if ((micros() - slotBeginAt) >= t_outMicros)  // timeout
        {// when timeout occurs (after 1 or FHSS_CHANNELS_NUM * interval; so after 7ms or 476 msec
            if ( isConnected2Tx )// if we where connected and just waited for 1 interval = 7 msec
            {
                slotBeginAt += smoothedInterval; // to avoid cumulative jitter, we just add the theoretical slot interval to the previous theoretical frame begin.  
                handleShortTimeout(); //change antenna, update some counters and perform a freq hop (not when next slot is for dwnlnk)
            }
            else// it means no connection or connection has already been lost (after to many missing packets) 
            {
                G3PULSE(30);// to debug when a long time out occurs
                slotBeginAt += FHSS_CHANNELS_NUM * smoothedInterval + MARGIN_LONG_TIMEOUT;
                handleLongTimeout(); //process LED, failsafe setup and frequency hop
            }
            break;// exit while() when a time out occurs
        }
                
        #if defined( MSW_SERIAL) 
            callSportSwSerial(); // get data from sport that have been stored by sportbuff[] (filled by an interrup)
                                    // accumulate bytes it in sRxData[]
                                   // when frame is full, check it, convert it and 
                                   // put data in a circular buffer (sportData[]) that will be handled by downlink tlm slot.
        #endif
        #ifdef SPORT_TELEMETRY
        if ( sportPollIsrFlag){  // flag is set by an interrupt from a timer 
            handleSportPoll(); // perform a polling (or forward a uplink tlm frame) at regular interval (12msec)
        }
        #endif
        #ifdef SBUS
            // send Sbus frame on Serial once evey xx msec 
            if ( ( micros() - lastSbusMicros) > SBUS_INTERVAL ) {
                lastSbusMicros = micros();
                if (sbusAllowed) 
                {  // at least Rx has been connected and Failsafe is not activated with NO PULSE(sbusAllowed)
                    SBUS_frame(); // fill Sbus frame with channels data from sbusData[]
                    for (uint8_t i = 0; i < TXBUFFER_SIZE; i++) Serial.write(sbus[i]);
                }
            }        
        #endif
        
        if (! isConnected2Tx) { // not needed to yield when connected because it is already done at begining of the loop
                                // it is only when not connected that we can stay in while more that 7000 msec
                                // avoiding yield is good because it can take up to 300 msec on ESP8266 
            if( (micros() - lastYieldMicros) > 20000 ) {
                yield();
                lastYieldMicros= micros();
            }    
        }    
    } // end while(1)

    // when we arrive here, it means that OR a valid frame has been received OR a time out (short ot long) occurs
    
    #if defined(TELEMETRY) // process downlink tlm
        if ( (packetSeq == 1 ) && ( isConnected2Tx) ) 
        { // next slot must be used to send a downlink telemetry packet but only if there is a connection.
          // here we send a downlink tlm frame even if we just miss one or a few frames (but not loss the connection)
            #ifdef HAS_PA_LNA
                #ifdef EU_LBT
                    if (!ChannelIsClear()) 
                        SX1280_SetOutputPower(MinPower);
                    else
                #endif
                SX1280_SetOutputPower(MaxPower);
            #endif
            MiloTlmSent();  // perhaps add some code to better synchronize with Tx slot timing
                          // if we send just after having received a valid frame, it will be about 5msec after TX started sending the previous frame
                          // if we did not get a valid frame, then we had first to wait for the end of the timeout before we reach this point
                          //     So we are about 7.5msec after the TX started sending the missinf frame.   
            
        }    
    #endif
    #ifdef STATISTIC
        LQICalc(0); // count this packet as received (not dropped)
    #endif

} // end main loop




void   SetupTarget()
{
    if (LED_pin != -1) pinMode(LED_pin, OUTPUT);
    if (BIND_pin != -1) pinMode(BIND_pin, INPUT);
    digitalWrite(SX1280_RST_pin,HIGH);
    pinMode(SX1280_RST_pin , OUTPUT);
    pinMode(SX1280_BUSY_pin , INPUT);
    pinMode(SX1280_DIO1_pin , INPUT);
    digitalWrite(SX1280_CSN_pin,HIGH);
    pinMode(SX1280_CSN_pin , OUTPUT); 
    
    digitalWrite(SX1280_CSN_pin, HIGH);
    #ifdef DIVERSITY        
        if ( SX1280_ANTENNA_SELECT_pin != -1 ) pinMode(SX1280_ANTENNA_SELECT_pin , OUTPUT);    
        SX1280_ANT_SEL_on;
    #endif
    //SPI
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(10000000); 
    #ifdef HAS_PA_LNA
        if (SX1280_TXEN_pin != -1) pinMode(SX1280_TXEN_pin , OUTPUT);
        if (SX1280_RXEN_pin != -1) pinMode(SX1280_RXEN_pin , OUTPUT);
    #endif
    EEPROM.begin(EEPROM_SIZE);
}

void MiLoRxBind(void)
{
    #if defined(DEBUG_BIND)
      debugln("Entering MiloRx Bind; waiting for a bind frame from Tx"); 
    #endif
    if(LED_pin != -1) LED_on;
    is_in_binding = true;
    #ifndef TUNE_FREQ
        currFreq = GetBindFreq(); //set frequency first or an error will occur!!!
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
            if ((RxData[0] & 0x07) == BIND_PACKET) { //bind frametype
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
    
    MProtocol_id = (RxData[1] | (RxData[2] << 8) | (RxData[3] << 16) | (RxData[4] << 24));
    #if defined(DEBUG_BIND)
        debugln("txid1 = %d,txid2= %d,rx_num = %d,chanskip = %d", MiLoStorage.txid[0], MiLoStorage.txid[1], MiLoStorage.rx_num, MiLoStorage.chanskip);
        debugln("FreqCorr = %d ", FreqCorrection);
        debugln("FreqCorrRegV = %d ", FreqCorrectionRegValue);
        debugln("MP_id = %d", MProtocol_id);
    #endif
    for (uint8_t i = 0; i < 16; i++)
        MiLoStorage.FS_data[i] = NO_PULSE;
    StoreEEPROMdata(address);
    while (1)
    {
        if(LED_pin != -1) LED_on;
        delay(500);
        if(LED_pin != -1) LED_off;
        delay(500);
    }
} // end MiloRxBind

#ifdef SPORT_TELEMETRY
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
        sportPollIsrFlag = true;
    }

    void  ICACHE_RAM_ATTR3 handleSportPoll() // this is called in main loop if timer for Sport fires
    {
        cli();
        sportPollIsrFlag = false; // reset the ISR flag    
        sei();
        if (sportMspCount)   // We have some uplink tlm data in sportMspData[] to send to sensor
        {
            memcpy((void *)sTxData, &sportMspData[sportMspTail].frame[0], sportMspData[sportMspTail].len);
            sendMSPpacket(sportMspData[sportMspTail].len);  // Start a background process (ISR) to send sTxData[] (with len bytes)  
            sportMspTail = (sportMspTail + 1) & 0X03; // do not exceed 4 values in circular buffer
            sportMspCount--;
        }
        else
            tx_sport_poll();
    }

#endif // end SPORT_TELEMETRY

#ifdef TELEMETRY
/*
        # RX downlink telemetry frame sent separate at a fixed rate of 1:3;frame rate 7ms. Can contain 2 Sport frame 
    0. - bits 7...2 : MSB of TXID1 (6 bits)
       - bits 1...0 : current downlink tlm counter (2 bits); when received TX should send this counter + 1type of link data packet(RSSI/SNR /LQI) (2 bits= 3 values currently) 
    1. - bits 7...2 : MSB of TXID2 (6 bits)
       - bits 1...0 : last upllink tlm counter received (2 bits); 
    2. - bit 7 : reserve
         bits 6..5 : recodified PRIM from sport frame1 (0X30=>0, 0X31=>1,0X32=>2, 0X10=>0)
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
*/
    #define PHID_LINK_QUALITY 0X1E
    #define PHID_NO_DATA      0X1F
    
    #ifdef SPORT_TELEMETRY
    uint8_t convertPrimPhid(uint8_t sportTailWhenAck){ // in return set bit7 = 1 in case of error
        uint8_t convert ;
        switch (sportData[sportTailWhenAck+1]) {   
            case 0X10:
                convert = 3;
                break;
            case 0X30:
                convert = 0;
                break;
            case 0X31:
                convert = 1;
                break;
            case 0X32:
                convert = 2;
                break;
            default:
                convert = 0XFF;
                debugln("error in conversion of PRIM");
        }
        convert = (convert <<5) | (sportData[sportTailWhenAck] & 0X01F); 
        if  ( (sportData[sportTailWhenAck] & 0X01F) > 0X1B ){
            debugln("error in conversion of PHID");
            convert |= 0X80 ; // set bit 7 to 1
        }
        return convert;
    }
    
    uint32_t lastLinkqAckedMicros = 0;
    uint32_t lastLinkqToAckMicros =  0;
    void appendTlmFrame()
    { // fill frame[2...15] with max 2 sets of data 
        // first part is filled with link quality data when timelap since previous exceeds xx msec or if there are less than 2 sport data available
        
        if ( (sportDataLen < 2) || ((micros() - lastLinkqAckedMicros) > 500) )
        { // fill with link quality data
            lastLinkqToAckMicros = micros();
            getRFlinkInfo();
            frame[2] = PHID_LINK_QUALITY;
            if (antenna) frame[4] = MiLoStats.uplink_RSSI_2;//antenna
            else         frame[4] = MiLoStats.uplink_RSSI_1;
            frame[5] = MiLoStats.uplink_SNR;
            frame[6] = MiLoStats.uplink_Link_quality;
            frame[7] = missingPackets;         // note : instead of sending the current value, we should calculate a max over a define timelap.
            frame[8] = getCurrentChannelIdx(); // note: this does not make lot of sense; it is more usefull in frames TX >> RX
            frame[9] = 0;
        } else if (sportDataLen > 0 ) {
            frame[2] = convertPrimPhid(sportTailWhenAck);
            if ( frame[2] & 0X80) { // in case of error discard the data
                frame[2]  = PHID_NO_DATA;
                memset( &frame[4] , 0, 6);   
            } else {
                memcpy( &frame[4] , &sportData[sportTailWhenAck + 2 ] , 6); 
            }
            sportTailWhenAck = (sportTailWhenAck + 8 ) & 0x3F;    
        } else {
            frame[2] = PHID_NO_DATA;
            memset( &frame[4] , 0, 6);    
        } 
        // at this stage, the first part id filled; now we do the second part
        if (sportTailWhenAck != sportHead) { // there is at least one second set of data available
            frame[3] = convertPrimPhid(sportTailWhenAck);
            if ( frame[3] & 0X80) { // in case of error discard the data
                frame[3]  = PHID_NO_DATA;
                memset( &frame[10] , 0, 6);   
            } else {
                memcpy( &frame[10] , &sportData[sportTailWhenAck + 2 ] , 6);
            }
            sportTailWhenAck = (sportTailWhenAck + 8 ) & 0x3F;    
        } else {
            frame[3] = PHID_NO_DATA;
            memset( &frame[10] , 0, 6);    
        } 
    }
    #endif
    void  ICACHE_RAM_ATTR3 MiLoTlm_build_frame()  // just create a downlink tlm frame ; sending is done in MiloTlmSent()
    {     
        frame[0] = (MiLoStorage.txid[0] & 0XFC ) | downlinkTlmId; 
        frame[1] = (MiLoStorage.txid[1] & 0XFC ) | uplinkTlmId;
        if (sportTail != sportTailWhenAck) {  
            if (downlinkTlmId == dwnlnkTlmExpectedId)
            { // previous dwnlnk frame has been ack so we can move sportTail to sportTailWhenAck
                lastLinkqAckedMicros = lastLinkqToAckMicros ; // avoid to generate linkquality data on each frame
                dwnlnkTlmExpectedId = (downlinkTlmId + 1) & 0x03; // calcuate Id that would allow an update for next downlink tlm frame
                sportTail = (sportTail + 8 ) & 0X3F ; // we first move one step forward.
                sportDataLen--;
                if (sportTail != sportTailWhenAck ) { // move one step more
                    sportTail = (sportTail + 8 ) & 0X3F ; // we first move one step forward.
                    sportDataLen--;
                    if (sportTail != sportTailWhenAck ) debugln("Error in handling sportTail");
                }
                sportTail = sportTailWhenAck;
            } else{ // when no Ack, roll back sportTailWhenAck to sportTail
                sportTailWhenAck = sportTail;
                Serial.println("Roll Tail back");
            } 
        }
        #ifdef DEBUG_SPORT_SPORTDATA
            Serial.print("Tail="); Serial.print(sportTail); Serial.print(" ifAck="); Serial.print(sportTailWhenAck); Serial.print(" data=");
            uint8_t idx = sportTail;
            for (uint8_t i=0 ; i < sportDataLen ; i++){
                for (uint8_t j=0 ; j<8; j++){
                    Serial.print(sportData[idx + j],HEX) ; Serial.print(";");
                }
                Serial.print("  /  ");
                idx = (idx+8) & 0X3F; //max 64 bytes in circular buffer
            }
            Serial.println(" ");
        #endif
        appendTlmFrame();   // fill frame[2...15]
        #ifdef DEBUG_DOWNLINK_TLM_FRAME
            if ( ((frame[2]&0X1F) < 0X1E ) || ((frame[3]&0X1F) < 0X1E ) ) { // print only when part 1 or 2 if filled with sportdata. 
                Serial.print("ifAck="); Serial.print(sportTailWhenAck);
                Serial.print(" pack=");
                Serial.print(frame[0]&0XFC,HEX); Serial.print("-"); Serial.print(frame[0]&0X03); Serial.print(";");
                Serial.print(frame[1]&0XFC,HEX); Serial.print("-"); Serial.print(frame[1]&0X03); Serial.print(";");  
                Serial.print((frame[2]&0b01100000)>>5); Serial.print("/"); Serial.print(frame[2]&0X1F,HEX); Serial.print(";");  
                Serial.print((frame[3]&0b01100000)>>5); Serial.print("/"); Serial.print(frame[3]&0X1F,HEX); Serial.print(";");  
                for(uint8_t i = 4;i<NBR_BYTES_IN_PACKET;i++){
                    Serial.print(frame[i],HEX);  Serial.print(";");
                }
                Serial.println("");
            }     
        #endif  
    }

    void  ICACHE_RAM_ATTR3 MiloTlmSent() // create and send a downlink tlm frame to TX
    {
        MiLoTlm_build_frame();
        delayMicroseconds(500);//just in case  // mstrens : this is probably not enough to let TX being in receiving mode at the end of his sending slot
        #ifdef HAS_PA_LNA
            SX1280_SetTxRxMode(TX_EN);//PA enabled
        #endif
        SX1280_WriteBuffer(0x00, frame, PayloadLength); //
        SX1280_SetMode(SX1280_MODE_TX);
    }

    
#endif // end TELEMETRY

#ifdef SPORT_TELEMETRY
    void  sportMSPstuff(uint8_t *p)// add START + stuff uplink tlm data + CRC
    {
        if (sportMspCount > 3) return; // discard if array is full
        uint16_t crc_s = 0;
        uint8_t indx = 0;
        sportMspData[sportMspHead].frame[indx++] = 0x7E; // add START
        sportMspData[sportMspHead].frame[indx++] = p[0]; // add first byt; no stuff not in CRC 
        for (uint8_t i = 1 ; i < 9; i++)
        {
            if (i == 8)
                p[i] = 0xff - crc_s;//this is the crc byte
            if (p[i] == 0x7D || p[i] == 0x7E)
            {
                sportMspData[sportMspHead].frame[indx++] = 0x7D;
                sportMspData[sportMspHead].frame[indx++] = p[i] ^ 0x20;
            }
            else
            {
                sportMspData[sportMspHead].frame[indx++] = p[i];   ;
            }
            crc_s += p[i]; //0-1FF
            crc_s += crc_s >> 8; //0-100
            crc_s &= 0x00ff;
        }
        sportMspData[sportMspHead].len = indx;
        sportMspHead =  (sportMspHead + 1) & 0X03; // advance Head (max 4 values)
        sportMspCount++;
    }

    // The sensor replies to the polling with a frame that starts by START code
    //             contains at least 8 bytes but it can be more due to stuffing
    #ifdef MSW_SERIAL
        void  ICACHE_RAM_ATTR3 callSportSwSerial(){
            sport_index = sportindex;
            if (sport_index >= 8) { // 
                if ((micros() - sportStuffTime) > 500){//If not receiving any new sport data in 500us
                    detachInterrupt(digitalPinToInterrupt(SPORT_pin));//no need to keep interrupt on serial pin after transferring all data    
                    cli();
                    sportindex = 0; //reset the counter (number of received bytes) used in the iterrupt 
                    sei();
                    memcpy((void*)sRxData,(const void*)sportbuff,sport_index);
                    #ifdef DEBUG_INCOMMING_SPORTDATA
                        Serial.print("Incoming Sport=");
                        for (uint8_t i=0 ; i < sport_index ; i++){
                            Serial.print(sRxData[i],HEX) ; Serial.print(";");
                        }
                        Serial.println(" ");
                    #endif
                    ProcessSportData();                
                }
            } 
        }

    #endif // MSW SERIAL

#endif //end SPORT_TELEMETRY

uint8_t bind_jumper(void)
{
    if (BIND_pin != -1) pinMode(BIND_pin, INPUT_PULLUP);
    if (IS_BIND_BUTTON_on) 
    {
        return 1;
    }
    return  0;
}


void ICACHE_RAM_ATTR dioISR()
{
    #ifdef DEBUG_ON_GPIO3
        GPOS =1<<3 ; // mstrens to debug : Make a pulse to measure the time in ISR ; GPOS is faster than digitalWrite()
    #endif
    dioOccured = true ;
    microsInDioISR= micros();
    #ifdef DEBUG_ON_GPIO3
        GPOC= 1<<3;  // mstrens to debug : End of pulse to measure the time in ISR
    #endif    
}    

#ifdef STATISTIC
    void LQICalc(uint8_t ThisPacketDropped){
        static uint8_t DropHistoryIndex ;
        if ( aPacketSeen > 5) {
            uint8_t oldDropBit = DropHistory[DropHistoryIndex];
            DropHistory[DropHistoryIndex] = ThisPacketDropped ;
            if ( ++DropHistoryIndex >= 100 ) DropHistoryIndex = 0 ;
            DropHistoryPercent += ThisPacketDropped ;
            DropHistoryPercent -= oldDropBit ;
                if (DropHistoryPercent <= 100)
                    uplinkLQ = (100 - DropHistoryPercent ) ;
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
