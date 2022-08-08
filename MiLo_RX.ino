/* **************************
	* By Midelic on RCGroups *
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

	
#define ICACHE_RAM_ATTR IRAM_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))

#include <SPI.h>
#include "_config.h"
#include "pins.h"
#include "iface_sx1280.h"
#include "FHSS.h"
#include "SX1280.h"

#ifdef SW_SERIAL
	#include <SoftwareSerial.h>
	SoftwareSerial swSer;
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
uint8_t  address = 0;//EEPROM start adress
bool setFSfromRx = false;
uint16_t countFS = 0;


//Channels
#define NOPULSE  0
#define HOLD  2047
#define MAX_MISSING_PKT 100

volatile uint16_t ServoData[16];
volatile int32_t missingPackets = 0;	 
bool packet = false;
uint8_t jumper = 0;
uint16_t c[8];
uint32_t t_out = 50;
uint32_t t_tune = 500;
int16_t wordTemp;
volatile uint8_t all_off = 0;
uint32_t packetTimer;
volatile bool frameReceived = false;
uint16_t LED_count = 0;
uint8_t RxData[15];
uint32_t bindingTime = 0;
uint32_t MProtocol_id = 0;
uint8_t PayloadLength = 15;
uint8_t FrameType = 0;
//Debug
//uint16_t BackgroundTime;
uint32_t debugTimer;
char debug_buf[64];
bool downlinkstart = false;
uint8_t packet_count = 0;
enum {
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

STORAGE_MILO_DATA *MiLoStrgPtr ,MiLoStorage;


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
uint8_t oldDropBit = 0;

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
	uint8_t frame[15];
	bool a_pass = false;
	uint8_t pass = 0;
	uint8_t antenna = 0;
	uint8_t telemetryRX = 0;
	uint8_t TelemetryExpectedId;
	uint8_t TelemetryId;
	uint8_t UplinkTlmId = 0;
	bool skipUntilStart = false;
	volatile bool sportMSPframe = false;
	uint8_t sportMSPdatastuff[16];
	uint8_t sportMSPdata[16];
	volatile uint8_t idxs = 0;
	uint8_t smartPortRxBytes = 0;
	uint8_t ReceivedSportData[11];
#endif

#ifdef SPORT_TELEMETRY
	#include "Sport_serial.h"
#endif

#define NOP() __asm__ __volatile__("nop")

void ICACHE_RAM_ATTR dioISR();
void SetupTarget(void);
void ICACHE_RAM_ATTR callSportSerial(void);
void  ICACHE_RAM_ATTR ISR_TIMER4_COMPA(void);
void  ICACHE_RAM_ATTR SX1280_TXnb(void);
void  ICACHE_RAM_ATTR MiLoTlm_build_frame(); 
uint8_t  ICACHE_RAM_ATTR MiLoTlm_append_sport_data(uint8_t *buf);
uint8_t ICACHE_RAM_ATTR DataLink(uint8_t pas);

//MILO-SX1280 RF parameters
uint8_t currOpmode = SX1280_MODE_SLEEP;
bool IQinverted = false;
uint32_t currFreq = 0;
uint8_t LoRaBandwidth;
uint32_t FreqCorrection;
uint32_t FreqCorrectionRegValue;
uint16_t timeout = 0xFFFF;

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
	uint8_t frame_rate;                   
	int32_t RXsensitivity;                // expected RF sensitivity based on
	uint32_t TOA;                         // time on air in microseconds
	uint32_t DisconnectTimeoutMs;         // Time without a packet before receiver goes to disconnected (ms) - from ExpressLRS
	uint32_t RxLockTimeoutMs;             // Max time to go from tentative -> connected state on receiver (ms) - from ExpressLRS
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

MiLo_mod_settings_s MiLo_AirRateConfig[RATE_MAX] = { 
	{0, RADIO_TYPE_SX128x_LORA, RATE_LORA_150HZ,  SX1280_LORA_BW_0800,SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_7, 7000, TLM_RATIO_1_3,12, 15 },
{1, RADIO_TYPE_SX128x_LORA, RATE_LORA_100HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_6, 9000, TLM_RATIO_1_3,12, 15}};



MiLo_rf_pref_params_s MiLo_AirRateRFperf[RATE_MAX] = {
	{0, RATE_LORA_150HZ,  -108,  5060, 3500, 2500},
{1, RATE_LORA_100HZ,  -112,  7605, 3500, 2500}};



void  ICACHE_RAM_ATTR MiLo_SetRFLinkRate(uint8_t index) // Set speed of RF link (hz) index values 
{
	
	MiLo_mod_settings_s *const ModParams = &MiLo_AirRateConfig[index];
	MiLo_rf_pref_params_s *const RFperf = &MiLo_AirRateRFperf[index];
	
	bool invertIQ = 0x01;
	if ((ModParams == MiLo_currAirRate_Modparams)
		&& (RFperf == MiLo_currAirRate_RFperfParams)
	&& (invertIQ == IQinverted))
	return;
	uint32_t interval = ModParams->interval;
	//uint32_t interval = 0XFFFF;//use micros() instead
	SX1280_Config(ModParams->bw, ModParams->sf, ModParams->cr, GetCurrFreq(),
	ModParams->PreambleLen, invertIQ, ModParams->PayloadLength, interval);
	
	MiLo_currAirRate_Modparams = ModParams;
	MiLo_currAirRate_RFperfParams = RFperf;
}
//end MILO-RF parameters


void setup()
{		
	SetupTarget();
	delay(10);//wait for stabilization
	
	#if defined(DEBUG) || defined(DEBUG_BIND) || defined (DEBUG_EEPROM) || defined (DEBUG_MSP) || defined (DEBUG_LOOP_TIMING)
		#undef SBUS		
		Serial.begin(230400,SERIAL_8N1,SERIAL_TX_ONLY);
		#define debugln(msg, ...)  { sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}	
	#endif
		
	#ifdef SBUS
		init_SBUS();
	#endif
	#if defined SPORT_TELEMETRY
		initSportUart();//SW serial
		SportHead = SportTail = 0;
	#endif	

	Fhss_Init();	
	MiLoRxBinding(0);	
	Fhss_generate(MProtocol_id);
	currFreq = GetCurrFreq(); //set frequency first or an error will occur!!!  
	bool init_success = SX1280_Begin();	
	if (!init_success)
	{
		return;
	}
	else
	{
		all_off = 1;	//no pulse		
		//MiLo_SetRFLinkRate(RATE_100HZ));
		MiLo_SetRFLinkRate(RATE_150HZ);
		SX1280_SetFrequencyReg(currFreq);	
		PayloadLength = MiLo_currAirRate_Modparams->PayloadLength;
		POWER_init();
		#ifdef HAS_PA_LNA
			SX1280_SetTxRxMode(RX_EN);//LNA enable
		#endif
		SX1280_SetMode(SX1280_MODE_RX);	
		frameReceived = false;			 
		bindingTime = millis() ;
		#ifdef SPORT_TELEMETRY
			if(!jumper)
			ConfigTimer();
		#endif
		
	}
}


void loop()
{
	static uint32_t interval = MiLo_currAirRate_Modparams->interval;
	static uint8_t TLMinterval = MiLo_currAirRate_Modparams->TLMinterval;
	static bool packetCount = false;	
	static uint8_t countUntilWiFi = 0;
	static uint8_t sportCount = 0;
	if(bind_jumper()&&jumper==0
		#ifdef TX_FAILSAFE 
			&& setFSfromTx == false
		#endif
	)
	
	if(countFS == 0)//only at boot reset (Failsafe) no reset while the rx is bound
	{	
		countFS = 1;
		jumper = 1;
	}	
	
		#ifdef USE_WIFI	
			if((millis() - bindingTime) >30000 && aPacketSeen < 5){//only at start after 30 sec binding process
				MiLoRxBinding(1);
			}
		#endif
	
	
	if(missingPackets > MAX_MISSING_PKT)
	{		
		#if defined(FAILSAFE)	
			
			for(uint8_t i = 0;i < 16;i++)
			{
				uint16_t word;
				word = MiLoStorage.FS_data[i];
				if(word == NO_PULSE)//no pulse
				{
					ServoData[i] = 0;
					#ifdef PARALLEL_SERVO
						Servo_sorted_data[i] = 0;
					#endif
					all_off = 1;
				}
				else 
				if(word == HOLD)//2047 equivalent is hold value so no update data.
				{
					#ifdef PARALLEL_SERVO
						Servo_sorted_data[i] = (ServoData[i]<<1);
					#endif					   
				}
				else {			   
					ServoData[i] = word;
					#ifdef PARALLEL_SERVO
						Servo_sorted_data[i] = (word<<1);
					#endif
				}			   
				
				#if defined(SBUS)		
					channel[i] = ServoData[i];
				#endif			   
			}
			#if defined SBUS
				SBUS_frame();				
			#endif
		#endif		
	}
	
	packetTimer = micros(); 
	while(1)
	{
		
		if ((micros() - packetTimer) >= ((t_out *interval) +t_tune))
		{
			if(t_out == 1)
			{
				#if defined DIVERSITY						
					if(missingPackets >= 3){				
						if(a_pass)
						{
							ANT_SEL_off;
							antenna = 0;//ant 1
						}
						else{					
							ANT_SEL_on;
							antenna = 1;//ant2
						}
						a_pass = !a_pass;
					}			
				#endif				
				
				if (missingPackets > MAX_MISSING_PKT)//lost connection
				{
					t_out = 50;
					countFS = 0;
					#ifdef TELEMETRY
						telemetryRX = 0;
					#endif
					uplinkLQ = 0;
					aPacketSeen = 0;
					dwnlnkstart = false;
					packet_count = 0;
				}
				if(jumper)
				countFS ++;
				
				#ifdef STATISTIC
					
					if ( aPacketSeen > 5 )//count dropped packets when the receiver is receiveing normally packets
					{
						DroppedPacketCount++;			
						ThisPacketDropped = 1;
						if(packetCount)
						{
							ThisPacketDropped = 0;
							packetCount = false;
						}
					}
					
				#endif
				missingPackets++;
				if(missingPackets >= 2)
				t_tune = 0;
				if(dwnlnkstart == true)
				packet_count = (packet_count + 1)%3;
				nextChannel(1);			
				SX1280_SetFrequencyReg(GetCurrFreq());			
			}
			else
			{
				if(jumper == 0){			
					LED_count++ & 0x02? LED_on :LED_off;					
				}
				else{//when button is pressed 
					
					uint8_t n = 10;
					while(n--){//fast blinking until resetting  FS data from RX	
						LED_toggle;
						delay(100);	//blink LED	  
					}
					for(uint8_t i = 0 ;i < 16;i++){
						if(MiLoStorage.FS_data[i] != 0)
						{
							MiLoStorage.FS_data[i] = 0;//reset the FS values while RX is wating to connect
							setFSfromRx = true;
						}
						
					}					
					if(setFSfromRx)
					{
							for(uint32_t i = 0;i < 8;i++){
								EEPROMWriteInt(address+4+2*i,MiLoStorage.FS_data[i]);	
							}	
							EEPROM.commit();						
						setFSfromRx = false;
					}
					jumper = 0;  
					
				}
				nextChannel(3);
				SX1280_SetFrequencyReg(GetCurrFreq());				
			}
			break;	
		}
		
		if(frameReceived == true)
		{
			uint8_t const FIFOaddr = SX1280_GetRxBufferAddr();
			SX1280_ReadBuffer(FIFOaddr,RxData, PayloadLength);	
			SX1280_GetLastPacketStats();								
			if((RxData[1] == MiLoStorage.txid[0])&&RxData[2] == MiLoStorage.txid[1])// Only if correct txid will pass
			{
				nextChannel(1);
				SX1280_SetFrequencyReg(GetCurrFreq());	
				frameReceived = false;
				missingPackets = 0;
				t_out = 1;
				t_tune = 500;
				FrameType = (RxData[0]&0x07);
				#ifdef LOOP_TIME_AVG
					//Serial.println(CalcAveragePacketTime());
				#endif
				if(FrameType != TLM_PACKET)
				{//only when no uplink telemetry				
				if(RxData[3] & 0x3F) != MiLoStorage.rx_num)
				break;//if other receiver with different modelID				
					if(aPacketSeen > 5)//when received some packets
					if (RxData[3] & 0x40){//receive Flag from tx to start wifi server
						if(++countUntilWiFi==2)
						{
						countUntilWiFi = 0;	
						timer0_detachInterrupt();//timer0 is needed for wifi
						startWifiManager();
						}
					}
				}
				
				#ifdef TELEMETRY
					if(FrameType == TLM_PACKET)
					{
						if ((RxData[3]&0x0F)> 0)
						{//Frame type uplink telemetry and no. of sport bytes >0							
							sportCount = (RxData[3]&0x0F);
							UplinkTlmId = (RxData[3]>>4);
							for (uint8_t i = 0;i <= sportCount;i++)
							ReceivedSportData[i] = RxData[i+3];//transfer all sport telemetry data in a buffer including No. of bytes in sport telemetry (uplink frame)		
						}
					}
				#endif
				
					if (++aPacketSeen >10 )//number packets received
					{
						aPacketSeen = 10 ;
					}

				#if defined(TELEMETRY)
					if(FrameType != TLM_PACKET) //sync telemetry 
					{
						if(RxData[3]>>7){
							telemetryRX = 1;//next is downlink telemetry
							packet_count = 1;
						}
					}
					else{//TLM PACKET
						telemetryRX = 1;//next is downlink telemetry
						dwnlnkstart = true;
						packet_count = 1;
					}
				#endif	
				packet = true;//flag ,packet ready to decode
				
				if(jumper==0)
				LED_on;
				
				all_off = 0;
				break;
			}
		}				
	}
	
	
	#if defined(TELEMETRY)	
		if(telemetryRX || packet_count == 1)
		{
			SX1280_TXnb();
			telemetryRX = 0;
			#ifdef STATISTIC
				if ( aPacketSeen > 5)
				{
					if(DroppedPacketCount){
						DroppedPacketCount --;
						packetCount = true;
					}
				}
			#endif
		}
		else
	#endif
	{
		if(packet ==true || missingPackets > 0){
		#ifdef HAS_PA_LNA
			SX1280_SetTxRxMode(RX_EN);// do first to allow LNA stabilise
		#endif
		SX1280_SetMode(SX1280_MODE_RX);
		}	
	}
	
	if (packet) {	
		if(dwnlnkstart == true)
	        packet_count = (packet_count + 1)%3;
		FrameType = (RxData[0]&0x07);
		
		if(FrameType != TLM_PACKET)
		{
			c[0]  = (uint16_t)(RxData[4] | ((RxData[5]  << 8) & 0x07FF));
			c[1]  = (uint16_t)((RxData[5]  >> 3 ) | (( RxData[6] << 5) & 0x07FF));
			c[2]  = (uint16_t)((RxData[6]  >> 6 ) | (RxData[7]  << 2 ) | ((RxData[8] << 10) & 0x07FF));
			c[3]  = (uint16_t)((RxData[8]  >> 1) | ((RxData[9]  << 7) & 0x07FF));
			c[4]  = (uint16_t)((RxData[9]  >> 4) | ((RxData[10]  << 4) & 0x07FF));
			c[5]  = (uint16_t)((RxData[10]  >> 7) | (RxData[11]  << 1)  | ((RxData[12] << 9 )& 0x07FF));
			c[6]  = (uint16_t)((RxData[12]  >> 2) | ((RxData[13] << 6) & 0x07FF));
			c[7]  = (uint16_t)((RxData[13] >> 5) | ((RxData[14] << 3) & 0x07FF));
						
			uint8_t j = 0;
			#if defined TX_FAILSAFE
			fs_started = false;
		    static uint8_t chan = 7;			
			#endif 	
			switch(FrameType)
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
				if(fs_started)
				chan = (chan+1)%8;
			#endif
			for(uint8_t i = 0;i < 8;i++)
			{
				
				wordTemp = c[i];	
				
				#if defined TX_FAILSAFE				
					if(fs_started && i == chan)
					{
						
						if( c[i] != NO_PULSE && c[i] != HOLD )
						{
							MiLoStorage.FS_data[chan+j] = word_temp;//custom FS
						}
						else if(c[i] == HOLD)//HOLD
						{					
							MiLoStorage.FS_data[chan+j] = 2047;					
						}
						else if(c[i] == NO_PULSE )//NO PULSE 
						{						
							MiLoStorage.FS_data[chan+j] = 0;
						}							
						//FS from TX is not saved in EEPROM!
						#ifdef DEBUG_FS
							if(fs_started){
								debugln("FS_data = " ,MiLoStorage.FS_data[chan+j]);
							}
						#endif			
					}													
					else
				#endif			
				if (wordTemp > 700 && wordTemp < 2300){
					ServoData[i+j] = wordTemp;
					#ifdef DEBUG_DATA
						debugln(" ServoData = %d", ServoData[i]);
					#endif
					#if defined SBUS						
						channel[i+j] = wordTemp;
					#endif
                   #ifdef PARALLEL_SERVO					
					ServoSortedData[i]= (wordTemp<<1);
					#endif
				}
				
			}
		
			if(jumper				
				#ifdef TX_FAILSAFE
					&& fs_started == false
				#endif
			)
			{
               setFSfromRx = true;	
			
				if(countFS++ >= MAX_MISSING_PKT)
				{
					(countFS & 0x10)?LED_off:LED_on;
					
				}
				if (countFS >= (2*MAX_MISSING_PKT))
				{
					if(setFSfromRx)
					{
						
							for(uint8_t i = 0;i < 16;i++){
								if(MiLoStorage.FS_data[i] != ServoData[i])//only changed values
								EEPROMWriteInt(address+4+2*i,MiLoStorage.FS_data[i]);						
							}
							EEPROM.commit();
						
						setFSfromRx = false;

					}
					jumper = 0;
				}
			}
		}
		#ifdef SPORT_TELEMETRY
			if (sportCount > 0){
				for (uint8_t i = 1;i <= sportCount;i++)
				smartPortDataReceive(ReceivedSportData[i]);
				sportCount = 0;
			}
		#endif
		packet = false;
		
		#ifdef STATISTIC
			// Received a packet, that's the definition of LQ
			oldDropBit = DropHistory[DropHistoryIndex];
			DropHistory[DropHistoryIndex] = ThisPacketDropped ;	
			if ( ++DropHistoryIndex >= 100 )
			{		
				DropHistoryIndex = 0 ;
			}
			DropHistoryPercent += ThisPacketDropped ;	
			DropHistoryPercent -= oldDropBit ;
			ThisPacketDropped = 0 ;
			if ( ++DropHistorySend >= 50 )//~450ms
			{
				if(DropHistoryPercent < 100){
					uplinkLQ = (100 - DropHistoryPercent ) ;		
				}		
				DropHistorySend = 0 ;
			}
		#endif	   
		#if defined(SBUS) 
			SBUS_frame();
		#endif 	   
	}
	
	#if defined  SBUS	
		if((millis() - sbus_timer) > 14){//sent out sbus on TLM packet every 7000*2 us
			sbus_timer = millis();
			if (all_off == 0)
			{
				for(uint8_t i = 0; i< TXBUFFER_SIZE;i++){
					#ifdef SBUS_INTERRUPT
						Serial_write(sbus[i]);
						#else
						Serial.write(sbus[i]);
					#endif
				}			
			}

	}
	#endif
	
	#ifdef SW_SERIAL
		callSportSerial();
	#endif
	
	#ifdef SPORT_TELEMETRY
	    ProcessSportData();
	#endif
}



void   SetupTarget()
{	
	pinMode(LED_pin, OUTPUT);
	pinMode(BIND_pin, INPUT);
	pinMode(SX1280_RST_pin ,OUTPUT);
	pinMode(SX1280_BUSY_pin ,INPUT);
	pinMode(SX1280_DIO1_pin ,INPUT);
	pinMode(SX1280_CSN_pin ,OUTPUT);//
	digitalWrite(SX1280_CSN_pin,HIGH);
	//SPI
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	SPI.setFrequency(10000000);
	#ifdef HAS_PA_LNA
	//if used SE2431
	//pinMode(SX1280_TXEN_pin ,OUTPUT);//CTX
	//pinMode(SX1280_RXEN_pin ,OUTPUT);//CPS
	//pinMode(SX1280_CSD_pin ,OUTPUT);//CSD
	//SX1280_CSD_on;// this is on all the time except in sleep mode of SE2431
	//	
	#endif
	#if defined DIVERSITY
		pinMode(ANT_SEL_pin ,OUTPUT);
		ANT_SEL_on;
	#endif
	EEPROM.begin(EEPROM_SIZE);
}


void  MiLoRxBinding(uint8_t bind){
	while(1){
		
		if (bind ==0){
			ReadEEPROMdata(address);//if flashing again new firmware need rebinding
			#if defined(DEBUG_EEPROM)
				delay(1000);
				debugln("txid1 = %d,txid2 = %d,rx_num = %d,chanskip = %d", MiLoStorage.txid[0],MiLoStorage.txid[1],MiLoStorage.rx_num,MiLoStorage.chanskip);
				Serial.print("MP_id ="	);
				Serial.println(MProtocol_id);
				for(uint8_t i = 0 ;i < 16;i++)
				{
					Serial.println(MiLoStorage.FS_data[i]);
				}
			#endif
			#ifdef HC_BIND
				MProtocol_id = 7059696;
				MiLoStorage.rx_num = 0;
				MiLoStorage.txid[0] = 240;
				MiLoStorage.txid[1] = 184;
			#endif
			is_in_binding = false;
			break;
		}
		else	
		{//binding time
			digitalWrite(LED_pin,HIGH);			
			MiLoRxBind();			
		}
		yield();//shut-up WDT
	}
}


void MiLoRxBind(void)
{
	is_in_binding = true;
	#ifndef TUNE_FREQ
		currFreq = GetInitialFreq(); //set frequency first or an error will occur!!!
		SX1280_Begin();//config
		POWER_init();
		MiLo_SetRFLinkRate(RATE_BINDING);	
	#endif
	SX1280_SetFrequencyReg(currFreq);
	#ifdef HAS_PA_LNA
		SX1280_SetTxRxMode(RX_EN);// do first to enable LNA 
	#endif
	SX1280_SetMode(SX1280_MODE_RX);
	
	while (1) 
	{
		if(frameReceived)
		{
			frameReceived = false;
			uint8_t const FIFOaddr = SX1280_GetRxBufferAddr();
			SX1280_ReadBuffer(FIFOaddr,RxData, PayloadLength);		
			if ((RxData[0]&0x07)==0)
			{//bind frametype
				MiLoStorage.txid[0] = RxData[1] ;			
				MiLoStorage.txid[1] = RxData[2];  
				MiLoStorage.rx_num = RxData[5];
				MiLoStorage.chanskip = RxData[6];
				LoRaBandwidth = LORA_BW_0800; 
				FreqCorrection = SX1280_GetFrequencyError();// get frequency offset in HZ
				// =(83ppm =199200Hz) at 2.4 GHZ
				FreqCorrection /= 1.0E9;
				FreqCorrectionRegValue = SX1280_FREQ_GHZ_TO_REG((double)FreqCorrection/1.0E9);			
				break;
			}
		}
		yield();//shut-up WDT
	}
	MProtocol_id = (RxData[1]|(RxData[2]<<8)|(RxData[3]<<16)|(RxData[4]<<24));
	#if defined(DEBUG_BIND)
		debugln("txid1 = %d,txid2= %d,rx_num = %d,chanskip = %d", MiLoStorage.txid[0],MiLoStorage.txid[1],MiLoStorage.rx_num,MiLoStorage.chanskip);
		debugln("Rx_data[2] = %d ,Rx_data[3] = %d ",Rx_data[2],Rx_data[3]);
		debugln("FreqCorr = %d ",FreqCorrection);						
		debugln("MP_id = %d",MProtocol_id);
	#endif
	for(uint8_t i = 0;i<16;i++){
		MiLoStorage.FS_data[i] = NOPULSE;
	}	
	
	StoreEEPROMdata(address);			
	while(1){
		digitalWrite(LED_pin,HIGH);
		delay(500);
		digitalWrite(LED_pin,LOW);
		delay(500);
	}			
}


#ifdef SPORT_TELEMETRY
	
	
	
	uint8_t  ICACHE_RAM_ATTR MiLoTlm_append_sport_data(uint8_t *buf)
	{
		uint16_t next;
		uint8_t index = 0;
		
		
		if(TelemetryId == TelemetryExpectedId)    
		idxOK = SportTail;//update read pointer to last ack'ed packet
		else
		SportTail = idxOK;
		
		TelemetryExpectedId = (TelemetryId + 1) & 0x1F;	
		
		while (index < 10 )
		{//max 10 bytes in a frame
			if(SportTail == SportHead)//if no sport data ,no send, buffer empty
			{
				break;
			}
			buf[index] = SportData[SportTail];
			
			next = SportTail + 1;		
			if(next >= MAX_SMARTPORT_BUFFER_SIZE)
			next = 0;	
			
			SportTail = next;
			
			index +=1;
		}
		
		return index;
	}
	
	void  ConfigTimer()
	{		
			noInterrupts();
			timer0_isr_init();
			timer0_attachInterrupt(ISR_TIMER4_COMPA);
			timer0_write(ESP.getCycleCount() +(12000*80));//12ms*(F_CPU/1000000)
			interrupts();	 	   	  		
	}
	
	void  ICACHE_RAM_ATTR ISR_TIMER4_COMPA()
	{		
			timer0_write(ESP.getCycleCount() +(12000*80));//12ms

		if(sportMSPframe)
		{
			memcpy(TxData, sportMSPdatastuff, idxs);
			sendMSPpacket();
			sportMSPframe = 0;
		}
		else
		tx_sport_poll();
		
		if(missingPackets > MAX_MISSING_PKT){//only when connection lost every 12ms		
		if(all_off == 0);
				for(uint8_t i = 0; i< TXBUFFER_SIZE;i++){
						Serial.write(sbus[i]);
			}
		}
		
	}
	
#endif

#ifdef TELEMETRY
	void  ICACHE_RAM_ATTR MiLoTlm_build_frame() 
	{
		uint8_t nbr_bytesIn;
		TelemetryId = (RxData[0]>>3)&0x1F;
		
		frame[0] = MiLoStorage.txid[0]; ;
		frame[1] = MiLoStorage.txid[1];
		frame[2] = DataLink(pass);
		frame[3] = TelemetryId|(pass << 5);	
		nbr_bytesIn = MiLoTlm_append_sport_data(&frame[5]);		
		frame[4] = nbr_bytesIn|((UplinkTlmId&0x0F)<<4);
		pass = (pass+ 1)%3;	
	}
	
	
	void  ICACHE_RAM_ATTR SX1280_TXnb()
	{
		MiLoTlm_build_frame();
		#ifdef HAS_PA_LNA
		    SX1280_setPower(MaxPower);
			SX1280_SetTxRxMode(TX_EN);//PA enabled
		#endif	
		SX1280_WriteBuffer(0x00, frame, PayloadLength); //
		SX1280_SetMode(SX1280_MODE_TX);
	}
	
	uint8_t  ICACHE_RAM_ATTR DataLink(uint8_t _pass)
	{
		static uint8_t link = 0 ;
		getRFlinkInfo();	
		switch (_pass){
			case 0:
			if(antenna)
			link = MiLoStats.uplink_RSSI_2;//antenna
			else
			link = MiLoStats.uplink_RSSI_1;			
			break;
			case 1:
			link = MiLoStats.uplink_Link_quality;
			break;
			case 2:
			link = MiLoStats.uplink_SNR;
			break;
		}
		return link;
	}
	
#endif


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
			
			if(p[i] == 0x7D||p[i] == 0x7E){
				sportMSPdatastuff[indx++] = 0x7D;
				sportMSPdatastuff[indx++] = p[i]^0x20;
			}
			else
			{
				sportMSPdatastuff[indx++] = p[i];	;
			}	
			crc_s += p[i]; //0-1FF
			crc_s += crc_s >> 8; //0-100
			crc_s &= 0x00ff;
		}
		idxs = indx;
	}
	
	
	//Sports received from TX(MSP)
	void  smartPortDataReceive(uint8_t c){ 
		
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
		
		if (smartPortRxBytes == 0) {
			
			if (c) {
				rxBuffer[smartPortRxBytes++] = c;
			}
			else {
				skipUntilStart = true;
			}
		}
		else {
			if (c == 0x7D) {
				byteStuffing = true;
				return;
			}
			
			if (byteStuffing) {
				c ^= 0x20;
				byteStuffing = false;
			}
			
			rxBuffer[smartPortRxBytes++] = c;
			
			if (smartPortRxBytes >= 8)
			{	
				#ifdef DEBUG_MSP
					//debugln("sportMSPdata = %d,smartPortRxBytes= %d" sportMSPdata[smartPortRxBytes],smartPortRxBytes);
				#endif
				#ifdef HAS_LUA
					//if(checkRxconfigfromTX()){}//future dev.this are configurations from TX to RX(LUA/basic scripts.)
					//else
				#endif
				{
					sportMSPstuff(sportMSPdata);
					sportMSPframe = true;
				}
				smartPortRxBytes = 0;
				skipUntilStart = true;				
			} 
		}
	}
		
	#ifdef SW_SERIAL
		void  ICACHE_RAM_ATTR callSportSerial(){
			while(swSer.available() > 0){
				if (sport_index < 16)
				sRxData[sport_index++] = swSer.read();
			}	
		}			
	#endif
#endif

uint8_t bind_jumper(void){
	pinMode(BIND_pin, INPUT_PULLUP);
	if ( digitalRead(BIND_pin) == LOW) {
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
	
	else
	if (irqStatus & (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT))
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
		if (fail == SX1280_RX_OK)
		{		
			frameReceived  = true ;
		}	
	}
	else{
	#ifdef STATISTIC
		TotalCrcErrors += 1 ;//bad packets that not pass the crc check
	#endif
	}
}




#ifdef TUNE_FREQ
	void  Sx1280_FreqTunning(){
		currFreq = GetCurrFreq(); //set frequency first or an error will occur!!! this the reg value not actual freq
		SX1280_Begin();//config
		MiLo_SetRFLinkRate(RATE_BINDING);
		//SX1280_setPower(MinPower);
		POWER_init();
		#ifdef HAS_PA_LNA
			SX1280_SetTxRxMode(RX_EN);// do first to enable LNA
		#endif
		Sx1280_SetMode(SX1280_MODE_RX);
		uint32_t  t_tune = millis();
		FreqCorrection = 0;
		int32_t freqTemp = 0; 
		while(1)
		{	 
			if((millis() - t_tune) > 50){
				t_tune = millis();
				if(FreqCorrection)
				SX1280_SetFrequencyReg(currFreq - FreqCorrectionRegValue);
				else
				SX1280_SetFrequencyReg(currFreq + FreqCorrectionRegValue);
				SX1280_SetMode(SX1280_MODE_RX);
			}
			if(frameReceived){
				if(RxData[0]&0x07) == BIND_PACKET){
				FreqCorrection = SX1280_GetFrequencyError();// get frequency offset in HZ
				
				if(FreqCorrection < 0){				 
					freqTemp = - FreqCorrection;
				}
				FreqCorrectionRegValue = SX1280_FREQ_GHZ_TO_REG((double)freqTemp/1.0E9);
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







