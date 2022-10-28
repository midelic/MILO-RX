
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
volatile uint8_t sportindex = 0;  // number of bytes in sRxData
uint8_t phase ;
uint8_t pindex; 
uint8_t ukindex ;   //unknown index
uint8_t kindex ;   //known
volatile uint8_t sTxData[MAX_SERIAL_BYTES]; // bytes ready to be sent to the sensor via sport (also used to send the 2 sport polling byte)
uint8_t sRxData[MAX_SERIAL_BYTES];          // circular buffer that contains the bytes to be sent to handset (already reformatted)
volatile uint8_t sportbuff[MAX_SERIAL_BYTES];
//uint8_t SportIndexPolling;
uint8_t sport_count;   // number of byte to send to the sensor

// circular buffer with data formatted to be sent to SX1280 (8 bytes per set of data)
uint8_t sportData[MAX_SMARTPORT_BUFFER_SIZE];
uint8_t sportHead =0; // position where next byte could be written in the buffer
uint8_t sportTail =0; // position of first byte to be read
uint8_t idxOK = 0;    // position where to roll back if Tx does not get the last dwnlnk frame
uint8_t sportDataLen = 0 ; // number of entries in the buffer (from 0 up to 8)
uint8_t cleanSportData[8]; // one set of Sport data in the format to be used by SX1280 (and in sportData[]) 

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

void generateDummySportDataFromSensor();

uint8_t  ICACHE_RAM_ATTR nextID()   // find the next Sport ID to be used for polling
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

void  ICACHE_RAM_ATTR tx_sport_poll()  // send the polling code
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

void  ICACHE_RAM_ATTR sendMSPpacket()  // send an uplink frame to the sensor
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
    void ICACHE_RAM_ATTR SerialPinISR()   // called when a pin changed (and we where waiting to receive a byte)
    {   
        if(digitalRead(SX1280_SPORT_RX_pin)==HIGH)   // Pin is high,inverted signal
        {  // When level goes up, it means we got a start bit
            disable_interrupt_serial_pin();   //disable pin change interrupt         
            timer0_attachInterrupt(SerialBitISR);
            timer0_write(ESP.getCycleCount()+ BIT_TIME + HLF_BIT_TIME );//return one and an 1/2 period into the future.
            sportRXbit = 0;   //Clear received bit counter.
            sportRX = 0;
            state = RECEIVE ;   // Change state
        }
    }

    void ICACHE_RAM_ATTR SerialBitISR()  // interrupt called while sending bytes or after getting a start bit (for receive)
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

uint8_t ICACHE_RAM_ATTR CheckSportData(uint8_t *packet) // calculate CRC on the first 8 bytes in a buffer
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


uint8_t ICACHE_RAM_ATTR unstuff()   // Remove stuffing in a buffer sRxData (filled by callSportData); return the (reduced) number of bytes
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

/*
void  ICACHE_RAM_ATTR StoreSportDataByte(uint8_t value)  // fill circular buffer sportData[len=Ox3F] with data from a sensor
{                                                        // this buffer will be used to transmit to TX   
    uint16_t next = (sportHead + 1) & 0x3F;   
    if (next != idxOK)
    {
        sportData[sportHead] = value;
        sportHead = next;
    }
}

void ICACHE_RAM_ATTR StuffSportBytes(uint8_t a)  // store a stuffed byte 
{
    if(a ==START_STOP||a ==BYTE_STUFF){//0x7E or 0x7D
        StoreSportDataByte(BYTE_STUFF);
        a = (a)^ STUFF_MASK;
    }
    StoreSportDataByte(a); 
}
*/

/* not used currently
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
*/

uint8_t checkSimilarSport(){
    uint8_t tail = sportTail;
    // look in circular buffer from the first occurence (if any)
    // return the position of a found set of data
    //        or the size of the circular buffer if not found.
    while ( tail != sportHead){
        if ( ( cleanSportData[0] == sportData[tail]) && ( cleanSportData[1] == sportData[tail+1] ) &&
            ( cleanSportData[2] == sportData[tail+2]) && ( cleanSportData[3] == sportData[tail+3] ) ){
                return tail;
            }
        tail += 8;
        if (tail >= MAX_SMARTPORT_BUFFER_SIZE) tail = 0;    
    }
    return MAX_SMARTPORT_BUFFER_SIZE;
}
void ICACHE_RAM_ATTR ProcessSportData()  // handle a frame received from the sensor (stored in SRxData)
{             // START byte has already been removed in the frame bing processed
              // frame contains at least 8 bytes but can be more (due to stuffing)
              // first remove stuff and check length and CRC. 
              // if OK, append a new (adapted) message to a circular buffer sportData[] (used with sportTail,  sportHead, IdxOK) 
    
    sport_index = unstuff(); // first remove stuff from sRxData[] having sport_index bytes
            // frame then contains now PRIM, ID1, ID2, VAL1, VAL2, VAL3, VAL4, CRC (= 8 bytes normally)
    if(sport_index >= 8)
    {
        if(CheckSportData(&sRxData[0]))//crc ok
        {
            // create a frame in the format ready to be sent
            cleanSportData[0] = sTxData[1];   // add PHID = last polling byte having been used
            memcpy( &cleanSportData[1], &sRxData[0], 7); // do not copy CRC
            // cleanSportData contains 8 bytes : PHID, PRIM, ID1, ID2, VAL1, VAL2, VAL3, VAL4 
            // check if similar (first 4 bytes) frame already exist in sportBuffer[]
            uint8_t similarSportIdx = checkSimilarSport(); // return the index of first byte of a similar frame
            if (similarSportIdx < MAX_SMARTPORT_BUFFER_SIZE) { // a similar has been found
                memcpy( &sportData[similarSportIdx+4],&cleanSportData[4],4) ; // update the last 4 bytes
            } else if ( sportDataLen < 8){   // when buffer is not full, add the data
                memcpy( &sportData[sportHead] , &cleanSportData[0], 8);
                sportHead = (sportHead + 8 ) & 0X3F ; 
                sportDataLen++;
            } else {
                // discard the incoming frame
            }
            /*
            //New SPORT frame will be generated and will have "logically" 10 bytes 
            //0x7E, PHID,PRIM,ID1,ID2,VAL1,VAL2,VAL3,VAL4, CRC  
            // but some bytes are stuffed and so there can be more than 10 bytes in the new buffer
            StoreSportDataByte(START_STOP);  //0x7E 
            StoreSportDataByte(sTxData[1]);   //PHID = last polling byte having been used
            StoreSportDataByte(sRxData[0]);   //PRIM
            for(uint8_t i = 1; i< (sport_index - 1);i++)
            {   // note: we do not push to sportData the original CRC
                StuffSportBytes(sRxData[i]);
            }
            */
            uint8_t phId ;
            phId = sTxData[1] & 0x1F ;
            if ( phId < 28 )
            {
                sport_rx_index[phId] = 1;
            }
        }
    }
    sport_index = 0 ;   //discard
}


#ifdef DEBUG_SIM_SPORT_SENSOR
    uint8_t debugSportDataReadyToSend[20] = {
        //0x7E, PHID,PRIM,ID1,ID2,VAL1,VAL2,VAL3,VAL4, CRC
        // #define VARIO_FIRST_ID          0x0110
        // a real frame from sensor has a START byte and no END byte but this START has already been removed in callSerial
        // so it is not present at the beginning of those debug data.
        // a 0X7E is added manually to the debug data to mark the end of the dummy buffer; this 0X7E is discarded during this process 
        0x10 , 0X05,  0X00, 0XC2, 0, 0, 0, 0X28, 0X7E,
    };

    uint32_t lastSportGeneratedMillis = 0;
    #define DEBUG_SPORT_INTERVAL 500  // interval between 2 dummy frames (must be at least about 20 to let SX1280 sent the frame)

    void generateDummySportDataFromSensor(){
        if ( ( millis() - lastSportGeneratedMillis ) > DEBUG_SPORT_INTERVAL ) {
            lastSportGeneratedMillis = millis();
            // copy data in sRxData[] 
            uint8_t i = 0;
            while ( ( i < 20) && ( debugSportDataReadyToSend[i] != 0X7E) ) {
                sRxData[i] = debugSportDataReadyToSend[i] ; // store up to next START  (not included)
                i++;       
            }
            sport_index = i;
            sTxData[1] = 0X83;  // simulate that sensor replies to polling ID 0X83
            ProcessSportData() ; // check the data and push them into the circular buffer sportData[] if OK
            // at this stage sportData should contains 1 or several time the values where 0X83 is a dummy PHID
            // 0X83, 0x10 , 0X05,  0X00, 0XC2, 0, 0, 0, 0X28,          
        }
    }
#endif
