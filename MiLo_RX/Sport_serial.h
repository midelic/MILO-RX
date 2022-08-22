
/* **************************
	* By Midelic on RCGroups *
	**************************
	This project is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	
	MiLo receiver code is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/

//SPORT routines
#define MAX_SMARTPORT_BUFFER_SIZE  64
#define MAX_SERIAL_BYTES  16
//
#define START_STOP     0x7E
#define BYTE_STUFF      0x7D
#define STUFF_MASK      0x20
//
#define ADC1_ID            0xf102//A1
#define ADC2_ID            0xf103//A2

//ADC3 uses an AppId of 0x0900~0x090f 
// ADC4 uses an AppId of 0x0910~0x091f
// Both send a 32-bit unsigned value that represents a voltage from 0 to 3.3V, in steps of 1/100 volt.

#define A3_FIRST_ID      0x0900
#define A3_LAST_ID       0x090f
#define A3_ID_8			   0x90
#define A4_FIRST_ID      0x0910
#define A4_LAST_ID       0x091f


const uint8_t sport_ID[] = {
    0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45,
	0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB,
	0xAC, 0x0D, 0x8E, 0x2F, 0xD0, 0x71,
	0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7,
    0x98, 0x39, 0xBA, 0x1B } ;



uint8_t sport_rx_index[28] ;
volatile  uint8_t sport_index=0;
uint8_t phase ;
uint8_t pindex;	
uint8_t ukindex ;//unknown index
uint8_t kindex ;//known
volatile uint8_t TxData[MAX_SERIAL_BYTES];
uint8_t sRxData[MAX_SERIAL_BYTES];
uint8_t SportIndexPolling;
uint8_t sport_count;
uint8_t SportData[MAX_SMARTPORT_BUFFER_SIZE];
uint8_t SportHead;
uint8_t SportTail;
uint8_t idxOK;

uint8_t  ICACHE_RAM_ATTR nextID()
{
	uint8_t i ;
	uint8_t poll_idx ; 
	if (phase)	// poll known
	{
		poll_idx = 99 ;
		for ( i = 0 ; i < 28 ; i++ )
		{
			if ( sport_rx_index[kindex] )
			{
				poll_idx = kindex ;
			}
			kindex++ ;
			if ( kindex >= 28 )
			{
				kindex = 0 ;
				phase = 0 ;
				break ;
			}
			if ( poll_idx != 99 )
			{
				break ;
			}
		}
		if ( poll_idx != 99 )
		{
			return poll_idx ;
		}
	}
	
	if ( phase == 0 )
	{
		for ( i = 0 ; i < 28 ; i++ )
		{
			if ( sport_rx_index[ukindex] == 0 )
			{
				poll_idx = ukindex ;
				phase = 1 ;
			}
			if (++ukindex > 27 )
			{
				ukindex = 0 ;
			}
			if ( poll_idx != 99 )
			{
				return poll_idx ;
			}
		}
		if ( poll_idx == 99 )
		{
			phase = 1 ;
			return 0 ;
		}
	}
	return poll_idx ;
}

void  ICACHE_RAM_ATTR tx_sport_poll()
{
    sport_count = 2;
	sport_index = 0;
	pindex = nextID();
	TxData[0] = START_STOP;	
	TxData[1] = sport_ID[pindex];
	#ifdef SW_SERIAL
		swSer.flush();
		swSer.enableTx(true); //for tx
		swSer.write((uint8_t *)TxData,(size_t)sport_count);
		swSer.enableTx(false);//for rx
	#endif
}


void  ICACHE_RAM_ATTR sendMSPpacket()
{
	sport_index = 0 ;
	sport_count = idxs;
	idxs = 0;
	#ifdef SW_SERIAL
		swSer.flush();
		swSer.enableTx(true); //for tx
		swSer.write((uint8_t *)TxData, (size_t) sport_count);
		swSer.enableTx(false);//for rx
	#endif
}

void initSportUart( )
{
	#ifdef SW_SERIAL
		swSer.begin(57600, SWSERIAL_8N1,3, 3,true, 256);//inverted
	#endif
}


uint8_t ICACHE_RAM_ATTR CheckSportData()
{
	volatile uint8_t *packet = sRxData ;
	uint16_t crc = 0 ;
	//if ( sport_index < 8 )//sport index = 8 with crc
	//{
	//	return 0 ;
	//}
	for ( uint8_t i = 0 ; i< 8 ; i++ )//no crc
	{
		crc += packet[i]; //0-1FF
		crc += crc >> 8; //0-100
		crc &= 0x00ff;
	}
	return (crc == 0x00ff) ;
}


uint8_t ICACHE_RAM_ATTR unstuff()
{
	uint8_t i ;
	uint8_t j ;	
	j = 0 ;
	if ( sport_index < 8 )
	{
		return 0 ;
	}
	
	for ( i = 0 ; i < sport_index ; i++ )
	{
		if ( sRxData[i] == BYTE_STUFF )//0x7D bytes already stuffed.
		{
			i += 1 ;
			sRxData[j] = sRxData[i] ^ STUFF_MASK ; 
		}
		else
		{
			sRxData[j] = sRxData[i] ;
		}
		j += 1 ;
	}
	return j ;
}


void  ICACHE_RAM_ATTR StoreSportDataByte(uint8_t value)
{
	uint16_t next = SportHead + 1;
	
	if(next >= MAX_SMARTPORT_BUFFER_SIZE){
		next = 0;
	}
	
	if (next != idxOK)
	{
		SportData[SportHead] = value;
		SportHead = next;
	}
}

void ICACHE_RAM_ATTR StuffSportBytes(uint8_t a)
{
	if(a ==START_STOP||a ==BYTE_STUFF){
		StoreSportDataByte(BYTE_STUFF);
		a = (a)^ STUFF_MASK;
	}
    StoreSportDataByte(a); 
}

void ICACHE_RAM_ATTR sport_send(uint16_t id, uint32_t v, uint8_t prim)//9bytes
{
	StoreSportDataByte(START_STOP);
	StoreSportDataByte(0x1A);
	StoreSportDataByte(prim) ;//0x10;0x32(MSP)
	StuffSportBytes(id & 0xFF);
	StuffSportBytes((id >> 8)&0xFF);	 
	StuffSportBytes(v & 0xFF);	 	
	StuffSportBytes((v >> 8) & 0xFF);
	StuffSportBytes((v >> 16) & 0xFF);
	StuffSportBytes((v >> 24) & 0xFF);
}

void ICACHE_RAM_ATTR ProcessSportData()
{
	sport_index = unstuff();
	
	if(sport_index >= 8)
	{
		//SPORT frame - 10 bytes
		//0x7E, PHID,PRIM,ID1,ID2,VAL1,VAL2,VAL3,VAL4, CRC  
		if(CheckSportData())//crc ok
		{
			StoreSportDataByte(START_STOP);	
			StoreSportDataByte(TxData[1]);//PHID
			StoreSportDataByte(sRxData[0]);//prim
			for(uint8_t i = 1; i< (sport_index - 1);i++)
			{
				StuffSportBytes(sRxData[i]);	
			}
			uint8_t phId ;
			phId = TxData[1] & 0x1F ;
			if ( phId < 28 )
			{
				sport_rx_index[phId] = 1;
			}
		}
		sport_index = 0 ;		
	}	
}
