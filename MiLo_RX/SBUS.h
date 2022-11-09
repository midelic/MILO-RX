//SBUS driver

#if defined(SBUS)
    #define SBUS_SYNCBYTE 0x0F 
    #define SBUS_ENDBYTE 0x00
    #define TXBUFFER_SIZE 25
    #define SBUS_INTERVAL 9 // in msec
    
    uint16_t sbus[TXBUFFER_SIZE]; // frame to be sent via Serial.print()
    uint16_t sbusChannel[16];     // values to generate the sbus frame
    
    uint32_t lastSbusMicros = 0; // last time that sbus has been generated

    void init_SBUS()
    {
        Serial.begin(100000,SERIAL_8E2,SERIAL_TX_ONLY);
        USC0(UART0) |= BIT(UCTXI);//inverted signal
    }
       
    void  SBUS_frame() 
    {  // create frame mainly based on sbusChannel[]
        sbus[0] = SBUS_SYNCBYTE;
        sbus[1] = lowByte(sbusChannel[0]);
        sbus[2] = highByte(sbusChannel[0]) | lowByte(sbusChannel[1])<<3;
        sbus[3] = sbusChannel[1]>>5|(sbusChannel[2]<<6);
        sbus[4] = (sbusChannel[2]>>2)& 0x00ff;
        sbus[5] = sbusChannel[2]>>10|lowByte(sbusChannel[3])<<1;
        sbus[6] = sbusChannel[3]>>7|lowByte(sbusChannel[4])<<4;
        sbus[7] = sbusChannel[4]>>4|lowByte(sbusChannel[5])<<7;
        sbus[8] = (sbusChannel[5]>>1)& 0x00ff;
        sbus[9] = sbusChannel[5]>>9|lowByte(sbusChannel[6])<<2;
        sbus[10] = sbusChannel[6]>>6|lowByte(sbusChannel[7])<<5;
        sbus[11] = (sbusChannel[7]>>3)& 0x00ff;//end
        sbus[12] = lowByte(sbusChannel[8]);
        sbus[13] = highByte(sbusChannel[8]) | lowByte(sbusChannel[9])<<3;
        sbus[14] = sbusChannel[9]>>5|(sbusChannel[10]<<6);
        sbus[15] = (sbusChannel[10]>>2)& 0xff;
        sbus[16] = sbusChannel[10]>>10|lowByte(sbusChannel[11])<<1;
        sbus[17] = sbusChannel[11]>>7|lowByte(sbusChannel[12])<<4;
        sbus[18] = sbusChannel[12]>>4|lowByte(sbusChannel[13])<<7;
        sbus[19] = (sbusChannel[13]>>1)& 0xff;
        sbus[20] = sbusChannel[13]>>9|lowByte(sbusChannel[14])<<2;
        sbus[21] = sbusChannel[14]>>6|lowByte(sbusChannel[15])<<5;
        sbus[22] = (sbusChannel[15]>>3)& 0xff;
        
        sbus[23] = 0x00;    
        if(missingPackets >= 1) sbus[23] |= (1<<2);//frame lost
        if(missingPackets > MAX_MISSING_PKT) sbus[23] |= (1<<3);//FS activated
        sbus[24] = SBUS_ENDBYTE;//endbyte   
    }   
#endif //SBUS
