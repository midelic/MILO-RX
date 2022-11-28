
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

#ifdef MSW_SERIAL
    #define  IDLE            0
    #define  TXPENDING       1 
    #define  TRANSMIT        2     
    #define  STOP_BIT        3          
    #define  RECEIVE         4
    #define  WAITING         5

    #define CLEAR_TX_BIT  GPOC=1<<SPORT_pin  // = digitalWrite(SPORT_pin,LOW) but about 10X faster
    #define SET_TX_BIT  GPOS=1<<SPORT_pin    // = digitalWrite(SPORT_pin,HIGH) but about 10X faster
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
#endif

#ifdef MSW_SERIAL
void sendSTxData() 
{ // send on Sport bus the data that have been prepared in sTxData[] ; sportTxCount= nbr of bytes 
        noInterrupts();
        sportindex = 0;
        detachInterrupt(digitalPinToInterrupt(SPORT_pin));
        timer0_detachInterrupt(); // mstrens not sure it is needed
        pinMode(SPORT_pin,OUTPUT);
        CLEAR_TX_BIT;
        state = TXPENDING;
        timer0_attachInterrupt(SerialBitISR); 
        timer0_write(ESP.getCycleCount() + 2*BIT_TIME);
        interrupts(); 
}
#endif

#ifdef MSW_SERIAL
    void ICACHE_RAM_ATTR SerialPinISR()   // called when a pin rise = start bit with inverted serial (we where waiting to receive a byte)
    {   
            detachInterrupt(digitalPinToInterrupt(SPORT_pin));   //disable pin change interrupt         
            sportRXbit = 0;   //Clear received bit counter.
            sportRX = 0;      // accumulate the bit received waiting for a full byte
            state = RECEIVE ;   // Change state
            //G1PULSE(5);
            timer0_attachInterrupt(SerialBitISR);
            nextSerialBitCycle = ESP.getCycleCount() + BIT_TIME_GET_FIRST_BIT;
            timer0_write(nextSerialBitCycle);  
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
                        if(digitalRead(SPORT_pin)==0)
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
                        attachInterrupt(digitalPinToInterrupt(SPORT_pin), SerialPinISR, RISING);   //switch to RX serial receive.             
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
                    pinMode(SPORT_pin,INPUT);   //RX serial on ESP8266 has pullup but Sport is inverted and it can't be used
                    timer0_detachInterrupt();//stop timer interrupt
                    state = IDLE ;   //change state to idle 
                    attachInterrupt(digitalPinToInterrupt(SPORT_pin), SerialPinISR, RISING);   //switch to RX serial receive.
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
            case IDLE:
                break;
        }
    }
#endif // end MSW_SERIAL

#ifdef MSW_SERIAL
void initSportUart()
{
    timer0_isr_init();
}
#endif

