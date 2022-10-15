
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
#define START_STOP      0x7E
#define BYTE_STUFF      0x7D
#define STUFF_MASK      0x20
//
#define ADC1_ID         0xf102//A1
#define ADC2_ID         0xf103//A2

//ADC3 uses an AppId of 0x0900~0x090f 
// ADC4 uses an AppId of 0x0910~0x091f
// Both send a 32-bit unsigned value that represents a voltage from 0 to 3.3V, in steps of 1/100 volt.

#define A3_FIRST_ID      0x0900
#define A3_LAST_ID       0x090f
#define A3_ID_8          0x90
#define A4_FIRST_ID      0x0910
#define A4_LAST_ID       0x091f

const uint8_t sport_ID[] = {
    0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45,
    0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB,
    0xAC, 0x0D, 0x8E, 0x2F, 0xD0, 0x71,
    0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7,
    0x98, 0x39, 0xBA, 0x1B } ;

uint8_t sport_rx_index[28] ;
volatile  uint8_t sport_index = 0;
volatile uint8_t sportindex = 0;
uint8_t phase ;
uint8_t pindex; 
uint8_t ukindex ;   //unknown index
uint8_t kindex ;   //known
volatile uint8_t sTxData[MAX_SERIAL_BYTES];
uint8_t sRxData[MAX_SERIAL_BYTES];
volatile uint8_t sportbuff[MAX_SERIAL_BYTES];
uint8_t SportIndexPolling;
uint8_t sport_count;
uint8_t SportData[MAX_SMARTPORT_BUFFER_SIZE];
uint8_t SportHead;
uint8_t SportTail;
uint8_t idxOK;
volatile uint32_t sportStuffTime = 0;  //timing extra stuffing bytes
#ifdef MSW_SERIAL
    #define  IDLE            0
    #define  TXPENDING       1 
    #define  TRANSMIT        2     
    #define  STOP_BIT        3          
    #define  RECEIVE         4
    #define  WAITING         5

    #define CLEAR_TX_BIT  digitalWrite(SX1280_SPORT_TX_pin,LOW)
    #define SET_TX_BIT  digitalWrite(SX1280_SPORT_TX_pin,HIGH)
    #define BIT_TIME  1105    //(1389 - 284)//minus cycles spent on calling ISR
    #define HLF_BIT_TIME  560

    uint8_t sportTXbit;   //sportTX bit counter.
    uint8_t sportRXbit;   //sportRX bit counter.
    uint8_t txcount;   // tx interrupt byte counter
    uint8_t sportTX;   // data to be transmitted.
    uint8_t sportRX;   //data to be received
    uint8_t state;    //serial state
    void ICACHE_RAM_ATTR SerialBitISR(void);
    void ICACHE_RAM_ATTR SerialPinISR(void);
    void ICACHE_RAM_ATTR enable_interrupt_serial_pin(){attachInterrupt(digitalPinToInterrupt(SX1280_SPORT_RX_pin), SerialPinISR, CHANGE);}
    void ICACHE_RAM_ATTR disable_interrupt_serial_pin() { detachInterrupt(digitalPinToInterrupt(SX1280_SPORT_RX_pin));} 
#endif

uint8_t  ICACHE_RAM_ATTR nextID()
{
    uint8_t i ;
    uint8_t poll_idx ; 
    if (phase)  // poll known
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
    sportindex = 0;
    pindex = nextID();
    sTxData[0] = START_STOP;
    sTxData[1] = sport_ID[pindex];
    #ifdef MSW_SERIAL   
        disable_interrupt_serial_pin();
        pinMode(SX1280_SPORT_TX_pin,OUTPUT);
        CLEAR_TX_BIT;
        state = TXPENDING;
        timer0_attachInterrupt(SerialBitISR); 
        timer0_write(ESP.getCycleCount() + 2*BIT_TIME);
    #elif defined SW_SERIAL
        swSer.flush();
        swSer.enableTx(true); //for tx
        swSer.write((uint8_t *)sTxData,(size_t)sport_count);
        swSer.enableTx(false);//for rx
    #endif
}

void  ICACHE_RAM_ATTR sendMSPpacket()
{
    sportindex = 0 ;
    sport_count = idxs;
    idxs = 0;
    #ifdef MSW_SERIAL       
        disable_interrupt_serial_pin();
        pinMode(SX1280_SPORT_TX_pin,OUTPUT);
        CLEAR_TX_BIT;
        state = TXPENDING;
        timer0_attachInterrupt(SerialBitISR); 
        timer0_write(ESP.getCycleCount() + 2*BIT_TIME);
    #elif defined SW_SERIAL
        swSer.flush();
        swSer.enableTx(true); //for tx
        swSer.write((uint8_t *)sTxData,(size_t)sport_count);
        swSer.enableTx(false);//for rx
    #endif  
}

#ifdef MSW_SERIAL
    void ICACHE_RAM_ATTR SerialPinISR()
    {   
        if(digitalRead(SX1280_SPORT_RX_pin)==HIGH)   // Pin is high,inverted signal
        {
            disable_interrupt_serial_pin();   //disable pin change interrupt         
            timer0_attachInterrupt(SerialBitISR);
            timer0_write(ESP.getCycleCount()+ BIT_TIME + HLF_BIT_TIME );//return one and an 1/2 period into the future.
            sportRXbit = 0;   //Clear received bit counter.
            sportRX = 0;
            state = RECEIVE ;   // Change state
        }
    }

    void ICACHE_RAM_ATTR SerialBitISR()
    {
        switch (state)
        {   
            //Transmit Sport Byte.
            case TRANSMIT :   // Output the TX buffer.
                if( sportTXbit < 8 )
                {
                    if( sportTX & 0x01 )                       
                        CLEAR_TX_BIT;     
                    else        
                        SET_TX_BIT;
                    sportTX >>= 1 ;  
                    sportTXbit += 1 ;      
                }
                else   //Send stop bit.
                {
                    CLEAR_TX_BIT; 
                    state = STOP_BIT;
                }
                timer0_write(ESP.getCycleCount() + BIT_TIME);
                break;
            case STOP_BIT :  
                if(++txcount < sport_count){
                    SET_TX_BIT;
                    timer0_write(ESP.getCycleCount() + BIT_TIME);           
                    sportTX = sTxData[txcount];     
                    sportTXbit = 0 ;
                    state = TRANSMIT ;
                }
                else
                {
                    pinMode(SX1280_SPORT_TX_pin,INPUT_PULLUP);   //RX serial on ESP8266 has pullup
                    timer0_write(ESP.getCycleCount() + 15*BIT_TIME);//after256us go to listening mode
                    state = WAITING;
                }
                break;
            case RECEIVE :
                timer0_write(ESP.getCycleCount() + BIT_TIME);
                {   
                    uint8_t data ;              
                    data = sportRXbit ;
                    if( data < 8 )
                    {
                        sportRXbit = data + 1 ;
                        data = sportRX ;
                        data >>= 1 ;   //Shift due to receiving LSB first.
                        if(digitalRead(SX1280_SPORT_RX_pin)==0)
                        {
                            data |= 0x80 ; 
                        }
                        sportRX = data ;
                    }
                    else      //receiving one byte completed
                    {
                        if (sportindex < 16)
                        {               
                            sportbuff[sportindex++] = sportRX ;    //store received bytes in a buffer
                            if (sportindex >= 8){
                                sportStuffTime = micros();
                            }                       
                        }                       
                        timer0_detachInterrupt();//stop timer interrupt
                        state = IDLE ;   //change state to idle 
                        enable_interrupt_serial_pin();   //switch to RX serial receive.             
                    }
                }           
                break; 
            case TXPENDING :
                SET_TX_BIT;
                timer0_write(ESP.getCycleCount() + BIT_TIME);
                sportTXbit = 0 ;
                txcount = 0 ;
                sportTX = sTxData[0];
                state = TRANSMIT;
                break;
            case WAITING :  
                timer0_detachInterrupt();//stop timer interrupt
                state = IDLE ;   //change state to idle 
                enable_interrupt_serial_pin();   //switch to RX serial receive.
                break;
            case IDLE:
                break;
        }
    }
#endif // end MSW_SERIAL

void initSportUart( )
{
#ifdef MSW_SERIAL
    timer0_isr_init();
    #elif defined SW_SERIAL
    swSer.begin(57600, SWSERIAL_8N1,3, 3,true, 256);//inverted
#endif
}

uint8_t ICACHE_RAM_ATTR CheckSportData(uint8_t *packet)
{
    uint16_t crc = 0 ;
    for ( uint8_t i = 0 ; i< 8 ; i++ )   //no crc
    {
        crc += packet[i];   //0-1FF
        crc += crc >> 8;   //0-100
        crc &= 0x00ff;
    }
    return (crc == 0x00ff) ;
}


uint8_t ICACHE_RAM_ATTR unstuff()
{
    uint8_t i ;
    uint8_t j ; 
    j = 0 ;

    for ( i = 0 ; i < sport_index ; i++ )
    {
        if ( sRxData[i] == BYTE_STUFF )   //0x7D bytes already stuffed.
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
    uint16_t next = (SportHead + 1)%0x3F;   
    if (next != idxOK)
    {
        SportData[SportHead] = value;
        SportHead = next;
    }
}

void ICACHE_RAM_ATTR StuffSportBytes(uint8_t a)
{
    if(a ==START_STOP||a ==BYTE_STUFF){//0x7E or 0x7D
        StoreSportDataByte(BYTE_STUFF);
        a = (a)^ STUFF_MASK;
    }
    StoreSportDataByte(a); 
}

void ICACHE_RAM_ATTR sport_send(uint16_t id, uint32_t v, uint8_t prim)//9bytes
{
    StoreSportDataByte(START_STOP);
    StoreSportDataByte(0x1A);
    StoreSportDataByte(prim) ;   //0x10;0x32(MSP)
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
        if(CheckSportData(&sRxData[0]))//crc ok
        {
            StoreSportDataByte(START_STOP);  //0x7E 
            StoreSportDataByte(sTxData[1]);   //PHID
            StoreSportDataByte(sRxData[0]);   //prim
            for(uint8_t i = 1; i< (sport_index - 1);i++)
            {
                StuffSportBytes(sRxData[i]);                
            }
            uint8_t phId ;
            phId = sTxData[1] & 0x1F ;
            if ( phId < 28 )
            {
                sport_rx_index[phId] = 1;
            }
        }
        sport_index = 0 ;   //discard   
    }   
}
