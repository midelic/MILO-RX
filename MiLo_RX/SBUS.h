//SBUS driver
uint16_t sbusChannel[16];     // values to generate the sbus frame; is used to store data even if SBUS is not activated


#if defined(SBUS)
    #define SBUS_SYNCBYTE 0x0F 
    #define SBUS_ENDBYTE 0x00
    #define TXBUFFER_SIZE 25
    #define SBUS_INTERVAL 9000 // in usec
    
    uint16_t sbus[TXBUFFER_SIZE]; // frame to be sent via Serial.print()
    
    uint32_t lastSbusMicros = 0; // last time that sbus has been generated

    
    void init_SBUS()
    {
        #ifdef ESP8266_PLATFORM
            Serial.begin(100000,SERIAL_8E2,SERIAL_TX_ONLY);
            USC0(UART0) |= BIT(UCTXI);//inverted signal
        #endif
        #ifdef RP2040_PLATFORM
            // Set up our UART with the required speed.
            uart_init(uart0, 100000);
            // Set the TX and RX pins by using the function select on the GPIO
            // Set datasheet for more information on function select
            gpio_set_function(SBUS_pin, GPIO_FUNC_UART);
            gpio_set_function(UART0_RX_pin, GPIO_FUNC_UART);
            // Set our data format (UART_ID, DATA_BITS, STOP_BITS, PARITY);
            uart_set_format(uart0, 8, 2, UART_PARITY_EVEN);
            uart_set_fifo_enabled (uart0 ,true); // enable the fifo (32 bytes)

            //Serial1.setTX(SBUS_pin);
            //Serial1.setRX(UART0_RX_pin);
            //Serial1.begin(100000, SERIAL_8N2);
            gpio_set_outover(SBUS_pin,  GPIO_OVERRIDE_INVERT) ; // inverted UART signal for Sbus
            gpio_pull_down(SBUS_pin); 
        #endif

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
