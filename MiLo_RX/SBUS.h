//SBUS driver

#if defined(SBUS)
    #define SBUS_SYNCBYTE 0x0F 
    #define SBUS_ENDBYTE 0x00
    #define TXBUFFER_SIZE 25
    
    #ifdef SBUS_INTERRUPT
        volatile uint8_t tx_buff[TXBUFFER_SIZE];
        volatile uint8_t tx_head;
        volatile uint8_t tx_tail;
    #endif
    volatile uint16_t sbus[TXBUFFER_SIZE];
    uint16_t channel[16];
    volatile uint32_t sbus_timer;
	uint8_t sbus_counter = 0;

    void init_SBUS()
    {
        Serial.begin(100000,SERIAL_8E2,SERIAL_TX_ONLY);
        USC0(UART0) |= BIT(UCTXI);//inverted signal
    }
    
    #ifdef SBUS_INTERRUPT
        void Serial_write(uint8_t data)
        {
            uint8_t t = tx_head;
            if(++t >= TXBUFFER_SIZE)
                t = 0;
            tx_buff[t]=data;
            tx_head = t;
        }
    #endif
    
    void  SBUS_frame()
    {
        sbus[0] = SBUS_SYNCBYTE;
        sbus[1] = lowByte(channel[0]);
        sbus[2] = highByte(channel[0]) | lowByte(channel[1])<<3;
        sbus[3] = channel[1]>>5|(channel[2]<<6);
        sbus[4] = (channel[2]>>2)& 0x00ff;
        sbus[5] = channel[2]>>10|lowByte(channel[3])<<1;
        sbus[6] = channel[3]>>7|lowByte(channel[4])<<4;
        sbus[7] = channel[4]>>4|lowByte(channel[5])<<7;
        sbus[8] = (channel[5]>>1)& 0x00ff;
        sbus[9] = channel[5]>>9|lowByte(channel[6])<<2;
        sbus[10] = channel[6]>>6|lowByte(channel[7])<<5;
        sbus[11] = (channel[7]>>3)& 0x00ff;//end
        //
        sbus[12] = lowByte(channel[8]);
        sbus[13] = highByte(channel[8]) | lowByte(channel[9])<<3;
        sbus[14] = channel[9]>>5|(channel[10]<<6);
        sbus[15] = (channel[10]>>2)& 0xff;
        sbus[16] = channel[10]>>10|lowByte(channel[11])<<1;
        sbus[17] = channel[11]>>7|lowByte(channel[12])<<4;
        sbus[18] = channel[12]>>4|lowByte(channel[13])<<7;
        sbus[19] = (channel[13]>>1)& 0xff;
        sbus[20] = channel[13]>>9|lowByte(channel[14])<<2;
        sbus[21] = channel[14]>>6|lowByte(channel[15])<<5;
        sbus[22] = (channel[15]>>3)& 0xff;
        
        sbus[23] = 0x00;	
        if(missingPackets >= 1)
        sbus[23] |= (1<<2);//frame lost
        if(missingPackets > MAX_MISSING_PKT)
        sbus[23] |= (1<<3);//FS activated
        
        sbus[24] = SBUS_ENDBYTE;//endbyte	
    }	
#endif //SBUS
