
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

uint8_t sport_rx_index[28] ; // list of PHID of the sensors that replied to polling request( should be polled more often)
uint8_t phase = 0 ;              // used in research of next pooling ID to be used (PHID)
uint8_t ukindex =0;   //unknown index
uint8_t kindex =0;   //known

uint32_t nextSerialBitCycle; // next timestamp (in cycles) when we have to output/read next bit in software serial

volatile uint8_t sTxData[MAX_SERIAL_BYTES]; // bytes ready to be sent to the sensor via sport (also used to send the 2 sport polling byte)
uint8_t sportTxCount;   // number of bytes (in sTxData) to send to the sensor via sport (for polling of uplink tlm)

volatile uint8_t sportbuff[MAX_SERIAL_BYTES];
volatile uint8_t sportindex = 0;  // number of bytes in sportbuff[]

uint8_t sRxData[MAX_SERIAL_BYTES];          // circular buffer that contains the bytes received from sport (first unformatted and then reformatted)
uint8_t sport_index = 0;          // copy of sportindex (freeze); is used in ProcessSportData to know the number of bytes in sRxData


// circular buffer accumulates data received from sport sensor
// data are in groups of 8 bytes : PHID, PRIM, ID1, ID2, VAL1, VAL2, VAL3, VAL4
uint8_t cleanSportData[8]; // one set of Sport data in the format to be used by SX1280 (and in sportData[]) so without START, stuffing, CRC 
uint8_t sportData[MAX_SMARTPORT_BUFFER_SIZE]; // 64 is a multiple of 8 so we can store 8 dataset (each of 8 bytes); we need to know the lenght (in sportDataLen)
uint8_t sportHead =0; // position where next byte could be written in the buffer
uint8_t sportTailWhenAck =0; // position of first byte to be read when TX ACK previous dwnlnk packet
uint8_t sportTail = 0;  // position of first byte to be read (to use when Tx does not ACK previous dwnlnk packet
uint8_t sportDataLen = 0 ; // number of entries in the buffer (from 0 up to 8)

volatile uint32_t sportStuffTime = 0;  //timing extra stuffing bytes

#ifdef MSW_SERIAL
    #define  IDLE            0
    #define  TXPENDING       1 
    #define  TRANSMIT        2     
    #define  STOP_BIT        3          
    #define  RECEIVE         4
    #define  WAITING         5

    #define CLEAR_TX_BIT  GPOC=1<<SX1280_SPORT_pin  // = digitalWrite(SX1280_SPORT_pin,LOW) but about 10X faster
    #define SET_TX_BIT  GPOS=1<<SX1280_SPORT_pin    // = digitalWrite(SX1280_SPORT_pin,HIGH) but about 10X faster
    #define BIT_TIME  1389    //(1389 - 284)//minus cycles spent on calling ISR
    #define BIT_TIME_GET_FIRST_BIT  1100 // value based on tests with DEBUG_ON_GPIO1 in order that read bit occurs at the right time

    uint8_t sportTXbit;   //sportTX bit counter.
    uint8_t sportRXbit;   //sportRX bit counter.
    uint8_t txcount;   // tx interrupt byte counter
    uint8_t sportTX;   // data to be transmitted.
    uint8_t sportRX;   //data to be received
    volatile uint8_t state;    //serial state
    void ICACHE_RAM_ATTR SerialBitISR(void);
    void ICACHE_RAM_ATTR SerialPinISR(void);
    //oid ICACHE_RAM_ATTR enable_interrupt_serial_pin(){attachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin), SerialPinISR, RISING);}
    //void ICACHE_RAM_ATTR disable_interrupt_serial_pin() { detachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin));} 
#endif

void generateDummySportDataFromSensor();

uint8_t  ICACHE_RAM_ATTR nextID()   // find the next Sport ID to be used for polling
{
    uint8_t i ;
    uint8_t poll_idx = 99 ; 
    if (phase)  // poll known
    {
        for ( i = 0 ; i < 28 ; i++ )  // search max 28 X 
        {
            if ( sport_rx_index[kindex] ) { poll_idx = kindex ; }
            if ( ++kindex >= 28 )
            {
                kindex = 0 ;
                phase = 0 ;
                break;
            }
            if ( poll_idx != 99 ) { return poll_idx ; }
        }
    }
    // if a known has not been found or if we where looking for an unknown because last known was 28th  
    for ( i = 0 ; i < 28 ; i++ ) // search max 28 X
    {
        if ( sport_rx_index[ukindex] == 0 )
        {
            poll_idx = ukindex ; // Use the unknown index 
            phase = 1 ; // next time we will search a known PHID
        }
        if (++ukindex >= 28 ) { ukindex = 0 ; } // prepare next search
        if ( poll_idx != 99 ) { return poll_idx ; } // if found, use it.
    }
    // At this stage no we did not found an unsued channel
    phase = 1 ;
    return 0 ;
}

void  ICACHE_RAM_ATTR tx_sport_poll()  // send the polling code
{
    uint8_t pindex; 
    sportTxCount = 2;
    pindex = nextID();
    #ifdef DEBUG_SEND_POLLING
        debugln("Send polling %d", pindex);
    #endif    
    sTxData[0] = START_STOP;
    sTxData[1] = sport_ID[pindex];
    #ifdef MSW_SERIAL   
        noInterrupts();
        sportindex = 0;
        detachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin));
        timer0_detachInterrupt(); // mstrens not sure it is needed
        pinMode(SX1280_SPORT_pin,OUTPUT);
        CLEAR_TX_BIT;
        state = TXPENDING;
        timer0_attachInterrupt(SerialBitISR); 
        timer0_write(ESP.getCycleCount() + 2*BIT_TIME);
        interrupts(); 
    #endif
    #ifdef DEBUG_SIM_SPORT_SENSOR
        generateDummySportDataFromSensor(); // simulate immediately a reply from sensor adding data to sRxData and processing them
    #endif
}

void  ICACHE_RAM_ATTR sendMSPpacket()  // send an uplink frame to the sensor
{
    sportTxCount = idxs;
    idxs = 0;
    #ifdef DEBUG_SEND_POLLING
        debugln("Send msp packet");
    #endif    
    
    #ifdef MSW_SERIAL       
        noInterrupts();
        sportindex = 0 ;
        detachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin));
        timer0_detachInterrupt(); // mstrens not sure it is needed
        pinMode(SX1280_SPORT_pin,OUTPUT);
        CLEAR_TX_BIT;
        state = TXPENDING;
        timer0_attachInterrupt(SerialBitISR); 
        timer0_write(ESP.getCycleCount() + 2*BIT_TIME);
        interrupts();
    #endif  
}

#ifdef MSW_SERIAL
    void ICACHE_RAM_ATTR SerialPinISR()   // called when a pin rise = start bit with inverted serial (we where waiting to receive a byte)
    {   
        //if(digitalRead(SX1280_SPORT_pin)==HIGH)   // Pin is high,inverted signal
        //{  // When level goes up, it means we got a start bit
            detachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin));   //disable pin change interrupt         
            sportRXbit = 0;   //Clear received bit counter.
            sportRX = 0;      // accumulate the bit received waiting for a full byte
            state = RECEIVE ;   // Change state
            //G1PULSE(5);
            timer0_attachInterrupt(SerialBitISR);
            nextSerialBitCycle = ESP.getCycleCount() + BIT_TIME_GET_FIRST_BIT;
            timer0_write(nextSerialBitCycle);  
        //}
    }
    
    
    void ICACHE_RAM_ATTR SerialBitISR()  // interrupt called while sending bytes or after getting a start bit (for receive)
    {
        switch (state)
        {   
            case RECEIVE :
                nextSerialBitCycle += BIT_TIME;
                timer0_write(nextSerialBitCycle);
                {   
                    uint8_t data ;              
                    data = sportRXbit ;
                    if( data < 8 )
                    {
                        sportRXbit = data + 1 ;
                        data = sportRX ;
                        data >>= 1 ;   //Shift due to receiving LSB first.
                        G1ON;
                        if(digitalRead(SX1280_SPORT_pin)==0)
                        {
                            data |= 0x80 ; 
                        }
                        // could become which is probably faster:
                        //data |= (((*GPIO_IN & 0B1000) >> 3 ) ^ 0B1); 
                        G1OFF;
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
                        attachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin), SerialPinISR, RISING);   //switch to RX serial receive.             
                    }
                }           
                break; 
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
                nextSerialBitCycle += BIT_TIME;
                timer0_write(nextSerialBitCycle);
                break;
            case STOP_BIT :          // Stop bit has been generated
                if(++txcount < sportTxCount){
                    SET_TX_BIT;       // new start bit
                    nextSerialBitCycle += BIT_TIME;
                    timer0_write(nextSerialBitCycle);
                    sportTX = sTxData[txcount];     
                    sportTXbit = 0 ;
                    state = TRANSMIT ;
                }
                else
                {
                    pinMode(SX1280_SPORT_pin,INPUT);   //RX serial on ESP8266 has pullup but Sport is inverted and it can't be used
                    timer0_detachInterrupt();//stop timer interrupt
                    state = IDLE ;   //change state to idle 
                    attachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin), SerialPinISR, RISING);   //switch to RX serial receive.
                //
                //    timer0_write(ESP.getCycleCount() + BIT_TIME);
                //    state = WAITING;
                }
                break;
            case TXPENDING :
                SET_TX_BIT;                                  // output Start bit
                nextSerialBitCycle= ESP.getCycleCount() + BIT_TIME;
                timer0_write(nextSerialBitCycle);
                sportTXbit = 0 ;
                txcount = 0 ;
                sportTX = sTxData[0];
                state = TRANSMIT;
                break;
            //case WAITING :  
            //    timer0_detachInterrupt();//stop timer interrupt
            //    state = IDLE ;   //change state to idle 
            //    attachInterrupt(digitalPinToInterrupt(SX1280_SPORT_pin), SerialPinISR, RISING);   //switch to RX serial receive.
            //    break;
            case IDLE:
                break;
        }
    }
#endif // end MSW_SERIAL

void initSportUart()
{
#ifdef MSW_SERIAL
    timer0_isr_init();
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
    // note: this function is not 100% correct.
    // imagine folowing scenario: 
    // sportData[] contains already a set of data that have been sent to Tx in the second part (the first part being also a sensor dataset and not a link quality set)
    // TX has not yet acknowledge
    // a new set of data arrives from sensor with the same 4 first bytes
    // we will append the new set of data
    // we now have to create a new downlink frame but Tx has not give the ACK so we roll back SportTailIfAck to sportTail
    // suppose now that it is time to fill first part of packet with link quality data and so only first set from circular buffer will be added in second part.
    // so our second set will not be part of the packet (and we have 2 identical KEY in the circular buffer).
    // if new data arrives from sensor with identical key, only the first one will be updated.
    // so at the end we could send the oldiest data after having sent freshier data.  
    uint8_t tail = sportTailWhenAck; // we start looking from this to avoid updating a frame that have already been sent but not yet confirmed
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
void ICACHE_RAM_ATTR ProcessSportData()  // handle a frame received from the sensor (stored in sRxData)
{             // sRxData[] contains at least 8 bytes but can be more (due to stuffing) PRIM, ID1, ID2, VAL1, VAL2, VAL3, VAL4, CRC
              // first remove stuff and check length and CRC. 
              // if OK, append a new (adapted) message to a circular buffer sportData[] (used with sportTail,  sportHead sportTailWhenAck) 
    sport_index = unstuff(); // first remove stuff from sRxData[] having sport_index bytes
            // sRxData[] then contains now PRIM, ID1, ID2, VAL1, VAL2, VAL3, VAL4, CRC (= 8 bytes normally)
    if(sport_index >= 8) 
    {  //the received frame contains at least 8 bytes 
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
            uint8_t phId ;
            phId = sTxData[1] & 0x1F ;
            if ( phId < 28 )
            {
                if (sport_rx_index[phId] == 0) { // if new sensor, then mark it
                    sport_rx_index[phId] = 1; // mark that a new sensor has replied
                    if ( kindex < phId) {
                        // when kindex is < PHID and there is no KNOWN id before new PHID,
                        // then we have to change kindex to avoid sending the same PHID in the pooling request 
                        while (( kindex < phId) && (sport_rx_index[kindex] == 0)){ // advance kindex to next used avoid sending the same PHID several times in sequence
                            kindex++;
                        }
                        if (kindex == phId) kindex++; // advance 1 more to avoid new phid
                    }
                    //Serial.print("PHID=");Serial.print(phId);Serial.print(" upd kindex=");Serial.println(kindex);
                    //Serial.print("hase=");Serial.print(phase);Serial.print(" ukindex=");Serial.println(ukindex); 
                    //debugln("PHID=%d  kindex=%d",phId, kindex);
                }    
                
            }
        }
    }
    sport_index = 0 ;   //discard
}


#ifdef DEBUG_SIM_SPORT_SENSOR
    
    uint32_t lastSportGeneratedMillis = 0;
    #define DEBUG_SPORT_INTERVAL 500  // interval between 2 dummy frames (must be at least about 20 to let SX1280 sent the frame)
    uint32_t nextDummyValue = 0;  // value to put in the sport dummy frame
    uint8_t nextID2 = 0;          // id2 of the field to be put in the sport dummy frame
    
    void generateDummySportDataFromSensor(){
        if ( ( millis() - lastSportGeneratedMillis ) > DEBUG_SPORT_INTERVAL ) {
            lastSportGeneratedMillis = millis();        
            uint16_t crc ;
            uint8_t tempBuffer[8];
            tempBuffer[0] = 0X10 ; // type of packet : data
            tempBuffer[1] = 0X05    ; 
            tempBuffer[2] = nextID2 & 0X0F ; 
            tempBuffer[3] = nextDummyValue >> 0 ; // value 
            tempBuffer[4] = nextDummyValue >> 8 ;  
            tempBuffer[5] = nextDummyValue >> 16 ;  
            tempBuffer[6] = nextDummyValue >> 24; // value
            
            crc = tempBuffer[0] ;
            for (uint8_t i = 1; i<=6;i++){
                crc +=  tempBuffer[i]; //0-1FF
                crc += crc >> 8 ; //0-100
                crc &= 0x00ff ;
            }
            tempBuffer[7] = 0xFF-crc ;  // CRC in buffer
            // copy and convert bytes
            // Byte in frame has value 0x7E is changed into 2 bytes: 0x7D 0x5E
            // Byte in frame has value 0x7D is changed into 2 bytes: 0x7D 0x5D
            sport_index = 0;
            for (uint8_t i = 0 ; i<8 ; i++){
                if (tempBuffer[i] == 0x7E) {
                    sRxData[sport_index++] = 0x7D;
                    sRxData[sport_index++] = 0x5E;
                } else if (tempBuffer[i] == 0x7D) {
                    sRxData[sport_index++] = 0x7D;
                    sRxData[sport_index++] = 0x5D;
                } else {
                sRxData[sport_index++]= tempBuffer[i];
                }
            }
            nextDummyValue++;                      // increase the value
            nextID2++;                             // increase the ID2
            #ifdef DEBUG_SPORT_SIM_GENERATION
                Serial.print("sRxdata=");
                for(uint8_t i = 0; i < sport_index; i++){
                    Serial.print(sRxData[i],HEX); Serial.print(" ; "); 
                }
                Serial.println(" ");
            #endif
            // here we have a frame (without START but with CRC and stuffed) in sRxData[] ready for processing
            ProcessSportData() ; // check the data and push them into the circular buffer sportData[] if OK
        }
    }
#endif
