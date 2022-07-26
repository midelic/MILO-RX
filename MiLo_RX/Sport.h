
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

volatile uint8_t sTxData[MAX_SERIAL_BYTES]; // bytes formatted to be sent to the sensor via sport (also used to send the 2 sport polling bytes)
volatile uint8_t sportTxCount;   // number of bytes (in sTxData) to send to the sensor via sport (for polling of uplink tlm)

volatile uint8_t sportbuff[MAX_SERIAL_BYTES];  // buffer where the bytes received from sport sensor are initially received 
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

void generateDummySportDataFromSensor();

#ifdef MSW_SERIAL
    #include "Sport_serial.h"
#endif
#ifdef RP2040_PLATFORM
    #include "Sport_RP2040.h"
#endif

uint8_t ICACHE_RAM_ATTR3 nextID()   // find the next Sport ID to be used for polling
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

void  ICACHE_RAM_ATTR3 tx_sport_poll()  // send the polling code
{
    uint8_t pindex; 
    sportTxCount = 2;
    pindex = nextID();
    #ifdef DEBUG_SEND_POLLING
        debugln("Send polling %d", pindex);
    #endif    
    sTxData[0] = START_STOP;
    sTxData[1] = sport_ID[pindex];
    sendSTxData();
    #ifdef DEBUG_SIM_SPORT_SENSOR
        generateDummySportDataFromSensor(); // simulate immediately a reply from sensor adding data to sRxData and processing them
    #endif
}

void  ICACHE_RAM_ATTR3 sendMSPpacket(uint8_t nbrBytes)  // send an uplink frame to the sensor
{
    sportTxCount = nbrBytes; // sportTxCount is decreased in the ISR when bytes are sent
    #ifdef DEBUG_SEND_POLLING
        debugln("Send msp packet");
    #endif    
    sendSTxData();
}


uint8_t ICACHE_RAM_ATTR3 CheckSportData(uint8_t *packet) // calculate CRC on the first 8 bytes in a buffer
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


uint8_t ICACHE_RAM_ATTR3 unstuff()   // Remove stuffing in a buffer sRxData (filled by callSportData); return the (reduced) number of bytes
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


uint8_t checkSimilarSport(){ 
    uint8_t tail = sportTailWhenAck; // we start looking from this to avoid updating a frame that have already been sent but not yet confirmed.
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
void ICACHE_RAM_ATTR3 ProcessSportData()  // handle a frame received from the sensor (stored in sRxData)
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
