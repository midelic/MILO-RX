#include "iface_sx1280.h"
#include "pins.h"

#define RX_TIMEOUT_PERIOD_BASE SX1280_RADIO_TICK_SIZE_0015_US
#define RX_TIMEOUT_PERIOD_BASE_NANOS 15625

#ifdef USE_SX1280_DCDC
    #ifndef OPT_USE_SX1280_DCDC
        #define OPT_USE_SX1280_DCDC true
    #endif
    #else
        #define OPT_USE_SX1280_DCDC false
#endif


uint32_t BusyDelayStart;
uint32_t BusyDelayDuration;
//static int8_t powerCaliValues[PWR_COUNT] = {0};
int8_t CurrentPower = -19; // initialized with a dummy value to force a first update/saving when SX1280_SetOutputPower() is called 

void  ICACHE_RAM_ATTR3 SX1280_WriteReg(uint16_t address, uint8_t data)
{
    SX1280_WriteRegisterMulti(address, &data, 1);
}

void  ICACHE_RAM_ATTR3 SX1280_WriteRegisterMulti(uint16_t address, uint8_t *data, uint8_t size)
{
    uint8_t OutBuffer[size + 3];
    OutBuffer[0] = (SX1280_RADIO_WRITE_REGISTER);
    OutBuffer[1] = ((address & 0xFF00) >> 8);
    OutBuffer[2] = (address & 0x00FF);
    memcpy(OutBuffer + 3, data, size);
    WaitOnBusy();
    digitalWrite(SX1280_CSN_pin,LOW);
    SPI.transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
    digitalWrite(SX1280_CSN_pin,HIGH);
    BusyDelay(15);
}

uint8_t  ICACHE_RAM_ATTR3 SX1280_ReadReg(uint16_t address)
{
    uint8_t data;
    SX1280_ReadRegisterMulti( address, &data, 1);
    return data;
}

void  ICACHE_RAM_ATTR3 SX1280_ReadRegisterMulti(uint16_t address, uint8_t *data, uint8_t size)
{ 
    uint8_t OutBuffer[size + 4];
    OutBuffer[0] = (SX1280_RADIO_READ_REGISTER);
    OutBuffer[1] = ((address & 0xFF00) >> 8);
    OutBuffer[2] = (address & 0x00FF);
    OutBuffer[3] = 0x00;

    WaitOnBusy();
    digitalWrite(SX1280_CSN_pin,LOW);
    SPI.transfer(OutBuffer, uint8_t(sizeof(OutBuffer)));
    memcpy(data, OutBuffer + 4, size);
    digitalWrite(SX1280_CSN_pin,HIGH);
}

void  ICACHE_RAM_ATTR3 SX1280_WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    uint8_t localbuf[size];
    for (int i = 0; i < size; i++) // todo check if this is the right want to handle volatiles
    {
        localbuf[i] = buffer[i];
    }
    uint8_t OutBuffer[size + 2];
    OutBuffer[0] = SX1280_RADIO_WRITE_BUFFER;
    OutBuffer[1] = offset;
    memcpy(OutBuffer + 2, localbuf, size);
    WaitOnBusy();
    digitalWrite(SX1280_CSN_pin,LOW);
    SPI.transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
    digitalWrite(SX1280_CSN_pin,HIGH);  ;
    BusyDelay(15);
}

void  ICACHE_RAM_ATTR3 SX1280_ReadBuffer( uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 3];
    uint8_t localbuf[size];
    OutBuffer[0] = SX1280_RADIO_READ_BUFFER;
    OutBuffer[1] = offset;
    OutBuffer[2] = 0x00;
    WaitOnBusy();
    digitalWrite(SX1280_CSN_pin,LOW);
    SPI.transfer(OutBuffer, uint8_t(sizeof(OutBuffer)));
    digitalWrite(SX1280_CSN_pin,HIGH);
    memcpy(localbuf, OutBuffer + 3, size);
    for (int i = 0; i < size; i++) // todo check if this is the right wany to handle volatiles
    {
        buffer[i] = localbuf[i];
    }
}

void  ICACHE_RAM_ATTR3 SX1280_WriteCommand(uint8_t command, uint8_t val,uint32_t busyDelay)
{
    SX1280_WriteCommandMulti(command ,&val,1,busyDelay);//15ms
}

void  ICACHE_RAM_ATTR3 SX1280_WriteCommandMulti(uint8_t command, uint8_t *data, uint16_t size,uint32_t busyDelay)
{
    uint8_t OutBuffer[size + 1];
    OutBuffer[0] = (uint8_t)command;
    memcpy(OutBuffer + 1, data, size);
    WaitOnBusy();
    digitalWrite(SX1280_CSN_pin,LOW);
    SPI.transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
    digitalWrite(SX1280_CSN_pin,HIGH);
    BusyDelay(busyDelay);           
} 

void  ICACHE_RAM_ATTR3 SX1280_ReadCommand(uint8_t command, uint8_t data)
{
    SX1280_ReadCommandMulti(command,&data,1); 
}

void  ICACHE_RAM_ATTR3 SX1280_ReadCommandMulti(uint8_t command, uint8_t *data, uint8_t size)
{
    uint8_t OutBuffer[size + 2];
    #define RADIO_GET_STATUS_BUF_SIZEOF 3 // special case for command == SX1280_RADIO_GET_STATUS, fixed 3 bytes packet size
    WaitOnBusy();
    digitalWrite(SX1280_CSN_pin,LOW);
    if (command == SX1280_RADIO_GET_STATUS)
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        OutBuffer[2] = 0x00;
        SPI.transfer(OutBuffer, RADIO_GET_STATUS_BUF_SIZEOF);
        data[0] = OutBuffer[0];
    }
    else
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        memcpy(OutBuffer + 2, data, size);
        SPI.transfer(OutBuffer, sizeof(OutBuffer));
        memcpy(data, OutBuffer + 2, size);
    }
    digitalWrite(SX1280_CSN_pin,HIGH);  
}


//***************************************************8

void  ICACHE_RAM_ATTR3 SX1280_Reset()
{
    pinMode(SX1280_RST_pin, OUTPUT);
    delay(50);
    digitalWrite(SX1280_RST_pin,LOW);
    delay(50);
    digitalWrite(SX1280_RST_pin,HIGH);
    delay(100); // typically 2ms observed
    WaitOnBusy(); // instead of waiting to long, we look at the bysy pin (goes low when reset is done)

}


uint16_t ICACHE_RAM_ATTR3 SX1280_GetFirmwareVersion( void )
{
    digitalWrite(SX1280_CSN_pin,LOW);
    return( ( ( (uint16_t)SX1280_ReadReg( REG_LR_FIRMWARE_VERSION_MSB ) ) << 8 ) | ( SX1280_ReadReg( REG_LR_FIRMWARE_VERSION_MSB + 1 ) ) );
}

#ifdef HAS_PA_LNA
    void SX1280_SetTxRxMode(uint8_t mode)
    {
        if(mode == RX_EN)
        {   
            SX1280_RXEN_on;
            SX1280_TXEN_off;
        }
        else
        if (mode == TX_EN)
        {
            SX1280_RXEN_off;
            SX1280_TXEN_on;
        }
        else
        {
            SX1280_TXEN_off;
            SX1280_RXEN_off;
        }
    }
#endif

void ICACHE_RAM_ATTR3 SX1280_SetFrequency(uint32_t frequency)
{
    uint32_t f = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
    uint8_t data[3];
    data[0] = f >> 16;
    data[1] = f >> 8;
    data[2] = f;
    SX1280_WriteCommandMulti(SX1280_RADIO_SET_RFFREQUENCY, data, 3,15);
}

void ICACHE_RAM_ATTR3 SX1280_SetFrequencyReg(uint32_t frequency)
{
    uint8_t data[3];
    data[0] = frequency >> 16;
    data[1] = frequency >> 8;
    data[2] = frequency;
    SX1280_WriteCommandMulti(SX1280_RADIO_SET_RFFREQUENCY, data, 3,15);
}

void ICACHE_RAM_ATTR3 SX1280_SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[2];
    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    SX1280_WriteCommandMulti(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf),15);
}

void ICACHE_RAM_ATTR3 SX1280_SetMode(uint8_t  OPmode)
{
    if (OPmode == currOpmode)
    {
        return;
    }
    uint8_t buf[3];
    switch (OPmode)
    {
        case SX1280_MODE_SLEEP:
            //hal.WriteCommand(SX1280_RADIO_SET_SLEEP, (uint8_t)0x01);
            SX1280_WriteCommand(SX1280_RADIO_SET_SLEEP,(uint8_t)0x01,15);
            break;
        case SX1280_MODE_CALIBRATION:
            break;
        case SX1280_MODE_STDBY_RC:
            //hal.WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC, 1500);
            SX1280_WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC,1500);
            //delayMicroseconds(1500);
            break;
        case SX1280_MODE_STDBY_XOSC:
            //hal.WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC, 50);
            SX1280_WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC,50);
            //delayMicroseconds(50);
            break;
        case SX1280_MODE_FS:
            //hal.WriteCommand(SX1280_RADIO_SET_FS, (uint8_t)0x00, 70);
            SX1280_WriteCommand(SX1280_RADIO_SET_FS,(uint8_t)0x00,70);
            //delayMicroseconds(70);
            break;
        case SX1280_MODE_RX:
            buf[0] = RX_TIMEOUT_PERIOD_BASE;
            buf[1] = timeout >> 8;
            buf[2] = timeout & 0xFF;
            //hal.WriteCommand(SX1280_RADIO_SET_RX, buf, sizeof(buf), 100);
            SX1280_WriteCommandMulti(SX1280_RADIO_SET_RX, buf, sizeof(buf),100);
            //delayMicroseconds(100);
            break;
        case SX1280_MODE_TX:
            //uses timeout Time-out duration = periodBase * periodBaseCount
            buf[0] = RX_TIMEOUT_PERIOD_BASE;
            buf[1] = 0xFF; // no timeout set for now
            buf[2] = 0xFF; // TODO dynamic timeout based on expected on air time
            //hal.WriteCommand(SX1280_RADIO_SET_TX, buf, sizeof(buf), 100);
            SX1280_WriteCommandMulti(SX1280_RADIO_SET_TX, buf, sizeof(buf),100);
            //delayMicroseconds(100);
            break;
        case SX1280_MODE_CAD:
            break;
        default:
            break;
    }
    currOpmode = OPmode;
}

void ICACHE_RAM_ATTR3 SX1280_ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used
    uint8_t rfparams[3] = {sf, bw, cr};
    // hal.WriteCommand(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams), 25);
    SX1280_WriteCommandMulti(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams),25);
    //delayMicroseconds(25);
    switch (sf)
    {
        case SX1280_LORA_SF5:
        case SX1280_LORA_SF6:
            SX1280_WriteReg(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x1E);
            break;
        case SX1280_LORA_SF7:
        case SX1280_LORA_SF8:
            SX1280_WriteReg(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x37); // for SF7 or SF8
            break;
        default:
            SX1280_WriteReg(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x32); // for SF9, SF10, SF11, SF12
    }
    // Enable frequency compensation // datasheet says that it must be done
    SX1280_WriteReg(SX1280_REG_FREQ_ERR_CORRECTION, 0x1); // mstrens :this line does not exist in ELTS
}

void ICACHE_RAM_ATTR3 SX1280_SetPacketParamsLoRa(uint8_t PreambleLength, uint8_t HeaderType,
    uint8_t PayloadLength, uint8_t crc, uint8_t InvertIQ)
{
    uint8_t buf[7];
    buf[0] = PreambleLength;
    buf[1] = HeaderType;
    buf[2] = PayloadLength;
    buf[3] = crc;
    buf[4] = InvertIQ ? SX1280_LORA_IQ_INVERTED : SX1280_LORA_IQ_NORMAL;
    buf[5] = 0x00;
    buf[6] = 0x00;
    SX1280_WriteCommandMulti(SX1280_RADIO_SET_PACKETPARAMS, buf, sizeof(buf),20);
    //delayMicroseconds(20);
}

void ICACHE_RAM_ATTR3 SX1280_SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];
    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX1280_WriteCommandMulti( SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8, 15 );
}

uint16_t ICACHE_RAM_ATTR3 SX1280_GetIrqStatus( void )
{
    uint8_t irqStatus[2];
    SX1280_ReadCommandMulti( SX1280_RADIO_GET_IRQSTATUS, irqStatus, 2 );
    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

void ICACHE_RAM_ATTR3 SX1280_ClearIrqStatus( uint16_t irqMask )
{
    uint8_t buf[2];
    buf[0] = ( uint8_t )( ( ( uint16_t )irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irqMask & 0x00FF );
    SX1280_WriteCommandMulti( SX1280_RADIO_CLR_IRQSTATUS, buf, 2 ,15);
}

uint8_t ICACHE_RAM_ATTR3 SX1280_GetRxBufferAddr()
{
    WORD_ALIGNED_ATTR uint8_t status[2] = {0};
    SX1280_ReadCommandMulti(SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

uint8_t  ICACHE_RAM_ATTR3 SX1280_GetStatus()
{
    uint8_t status = 0;
    SX1280_ReadCommandMulti(SX1280_RADIO_GET_STATUS, (uint8_t *)&status, 1);
    return status;
}

int8_t ICACHE_RAM_ATTR3 SX1280_GetRssiInst( void )
{
    uint8_t raw = 0;
    SX1280_ReadCommandMulti( SX1280_RADIO_GET_RSSIINST, &raw, 1 );
    return ( int8_t ) ( -raw / 2 );
}

void ICACHE_RAM_ATTR3 SX1280_GetLastPacketStats()
{
    uint8_t status[2];
    SX1280_ReadCommandMulti(SX1280_RADIO_GET_PACKETSTATUS, status, 2);
    // LoRa mode has both RSSI and SNR
    LastPacketRSSI = -(int8_t)(status[0] / 2);
    LastPacketSNR = (int8_t)status[1] / 4;
    // https://www.mouser.com/datasheet/2/761/DS_SX1280-1_V2.2-1511144.pdf
    // need to subtract SNR from RSSI when SNR <= 0;
    int8_t negOffset = (LastPacketSNR < 0) ? LastPacketSNR : 0;
    LastPacketRSSI += negOffset;
}

uint8_t ICACHE_RAM_ATTR3 SX1280_GetPacketType(){
    uint8_t packetType = 0;
    SX1280_ReadCommandMulti(SX1280_RADIO_GET_PACKETTYPE, (uint8_t *)&packetType, 1);
    return packetType;
}

bool ICACHE_RAM_ATTR3 WaitOnBusy()
{
    uint32_t wtimeoutUS = 1000U;
    uint32_t startTime = micros();
    if(SX1280_BUSY_pin != -1)
    {
        while (IS_SX1280_BUSY_on) // wait untill not busy or until wtimeoutUS
        {
            if ((micros() - startTime) > wtimeoutUS)
            {
                return false;
            }
            else
            {
                NOP();
            }
        }
    }
    else
    {
    // observed BUSY time for Write* calls are 12-20uS after NSS de-assert
    // and state transitions require extra time depending on prior state
        if (BusyDelayDuration)
        {
            while ((micros() - BusyDelayStart) < BusyDelayDuration)
            NOP();
            BusyDelayDuration = 0;
        }
    }
    return true;
}



void ICACHE_RAM_ATTR3 BusyDelay(uint32_t duration)
{
    if (SX1280_BUSY_pin == -1)
    {
        BusyDelayStart = micros();
        BusyDelayDuration = duration;
    }
}

int32_t ICACHE_RAM_ATTR3 SX1280_complement2( const uint32_t num, const uint8_t bitCnt ) // convert bitCnt bitlength number in 2-complement notation to int32_t
{
    int32_t retVal = ( int32_t )num; //cast bitCnt length number to int32
    if( retVal & (1<<(bitCnt-1)) ) // check for sign bit at position bitCnt, if not set return value as is, else...
    {
        retVal -= 1<<bitCnt;       // convert from bitCnt 2-complement to 32bit 2-complement format by inverting all bits and subtracting 1, done by subtrating binary value of 1 followed by bitCnt zeroes
    }
    return retVal;
}

//*********************************
//*********************************
/* Steps for startup
    1. If not in STDBY_RC mode, then go to this mode by sending the command:
    SetStandby(STDBY_RC)
    2. Define the LoRa® packet type by sending the command:
    SetPacketType(PACKET_TYPE_LORA)
    3. Define the RF frequency by sending the command:
    SetRfFrequency(rfFrequency)
    The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz. SetRfFrequency() defines the Tx frequency.
    4. Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
    byte of the data payload by sending the command:
    SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
    Note:
    txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map.
    5. Define the modulation parameter signal BW SF CR
       SetModulationParams(modParam1,modParam2, modParam3)
       0x1 must also be written to the Frequency Error Compensation mode register 0x093C
    6. Define the packet format to be used by sending the command:  PreambleLength,  HeaderType, PayloadLength , CRC, InvertIQ/chirp invert
       SetPacketParams(pktParam1, pktParam2, pktParam3, pktParam4, pktParam5)

For Rx mode:
    1. Configure the DIOs and Interrupt sources (IRQs) by using command:
        SetDioIrqParams(irqMask,dio1Mask,dio2Mask,dio3Mask)
    2. Once configured, set the transceiver in receiver mode to start reception using command:
        SetRx(periodBase, periodBaseCount[15:8], periodBaseCount[7:0])
        periodBase = 0 = no time out - single shot
                   = 0xFFFF, Rx Continuous mode
                   = other, time out - single shot
    3.   Check the packet status to make sure that the packet has been received properly, by sending the command:
        GetPacketStatus()
    4.Once all checks are complete, clear IRQs by sending the command:
        ClrIrqStatus(irqMask)   
    5. (when required) Get the packet length and start address of the received payload by sending the command:
        GetRxBufferStatus()
    6. Read the data buffer using the command:
        ReadBuffer(offset, payloadLengthRx)                     
*/
bool  ICACHE_RAM_ATTR3 SX1280_Begin()
{
    SX1280_Reset();
    uint16_t firmwareRev = SX1280_GetFirmwareVersion();
    currOpmode = SX1280_MODE_SLEEP;
    if ((firmwareRev == 0) || (firmwareRev == 65535))
    {
        // SPI communication failed, just return without configuration
        return false;
    }
    SX1280_SetMode(SX1280_MODE_STDBY_XOSC); // in this mode, some commands are processed faster                                              
    SX1280_WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA,15);//Set packet type to LoRa  
    SX1280_SetFrequencyReg(currFreq);                                                                                                    //Set Freq
    SX1280_SetFIFOaddr(0x00, 0x00);                                                                                                      //Config FIFO addr
    SX1280_ConfigModParamsLoRa(SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_4_7); //Configure Modulation Params                                                                          
    SX1280_WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01,15);     //Enable auto FS                                                                 
    SX1280_WriteReg(0x0891, (SX1280_ReadReg(0x0891) | 0xC0));  //default is low power mode, switch to high sensitivity instead
    SX1280_SetPacketParamsLoRa(12, SX1280_LORA_PACKET_IMPLICIT, 15, SX1280_LORA_CRC_ON, SX1280_LORA_IQ_NORMAL); 
    //SX1280_SetDioIrqParams(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE);  //set IRQ to both RXdone/TXdone on DIO1
    SX1280_SetDioIrqParams(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_RX_DONE, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE);  //set IRQ to both RXdone/TXdone on DIO1
    if (OPT_USE_SX1280_DCDC)
    {
        SX1280_WriteCommand(SX1280_RADIO_SET_REGULATORMODE, SX1280_USE_DCDC,15);
    }
    
    #ifdef  SX1280_DIO1_pin
        attachInterrupt(digitalPinToInterrupt(SX1280_DIO1_pin), dioISR, RISING); //attach interrupt to DIO1 pin
    #endif
    
    return true;
}

void  ICACHE_RAM_ATTR3 SX1280_Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,uint8_t PreambleLength, bool InvertIQ, uint8_t _PayloadLength)
{
    //uint8_t irqs = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE; //mstrens commented this because we expect that RX done is enough
    uint8_t const mode = SX1280_PACKET_TYPE_LORA;    
    PayloadLength = _PayloadLength;
    IQinverted = InvertIQ;
    SX1280_SetMode(SX1280_MODE_STDBY_XOSC);
    SX1280_WriteCommand(SX1280_RADIO_SET_PACKETTYPE, mode,15);
    SX1280_ConfigModParamsLoRa(bw, sf, cr);
    packetLengthType = SX1280_LORA_PACKET_FIXED_LENGTH;
    SX1280_SetPacketParamsLoRa(PreambleLength, packetLengthType,_PayloadLength, SX1280_LORA_CRC_ON, InvertIQ);
    SX1280_SetFrequencyReg(freq);
    SX1280_SetDioIrqParams(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_RX_DONE, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE);
}

bool  ICACHE_RAM_ATTR3 SX1280_GetFrequencyErrorbool()
{
    // Only need the highest bit of the 20-bit FEI to determine the direction
    uint8_t feiMsb = SX1280_ReadReg(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
    // fei & (1 << 19) and flip sign if IQinverted
    if (feiMsb & 0x08)
        return IQinverted;
    else
        return !IQinverted;
}

void  ICACHE_RAM_ATTR3 getRFlinkInfo()
{
    int32_t rssiDBM = LastPacketRSSI;
    if (antenna == 0){
        if (rssiDBM > 0) 
            rssiDBM = 0;
        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        MiLoStats.uplink_RSSI_1 = -rssiDBM;
    }
    else
    {
        if (rssiDBM > 0) 
            rssiDBM = 0;
        MiLoStats.uplink_RSSI_2 = -rssiDBM;
    }
    MiLoStats.uplink_SNR = LastPacketSNR;
    MiLoStats.uplink_Link_quality = uplinkLQ;
}

void  ICACHE_RAM_ATTR3 SX1280_SetOutputPower( int8_t power )//default values 13 ->12.5dbm;no external PA
{
    #ifdef USER_MAX_POWER
        if ( power > USER_MAX_POWER)
            power = USER_MAX_POWER; // limit the power to the max defined by the user
    #endif 
    if (power == CurrentPower)
        return;
    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    uint8_t buf[2];
    if (power < -18) 
        power = -18;
    else if (13 < power) 
        power = 13;
    CurrentPower = power; // store the value to avoid updates with the same value    
    buf[0] = power + 18;
    buf[1] = (uint8_t)SX1280_RADIO_RAMP_04_US;
    SX1280_WriteCommandMulti( SX1280_RADIO_SET_TXPARAMS, buf, 2,15 );
}

int32_t ICACHE_RAM_ATTR3 SX1280_GetLoRaBandwidth( )
{
    int32_t bwValue = 0;    
    switch(LoRaBandwidth )
    {
        case LORA_BW_0200:
            bwValue = 203125;
            break;
        case LORA_BW_0400:
            bwValue = 406250;
            break;
        case LORA_BW_0800:
            bwValue = 812500;
            break;
        case LORA_BW_1600:
            bwValue = 1625000;
            break;
        default:
            bwValue = 0;
    }
    return bwValue;
}

double ICACHE_RAM_ATTR3 SX1280_GetFrequencyError( )
{
    uint8_t efeRaw[3] = {0};
    uint32_t efe = 0;
    double efeHz = 0.0;
    efeRaw[0] = SX1280_ReadReg( SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
    efeRaw[1] = SX1280_ReadReg( SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
    efeRaw[2] = SX1280_ReadReg( SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
    efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
    efe &= SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;
    efeHz = 1.55 * ( double )SX1280_complement2( efe, 20 ) / ( 1600.0 / ( double ) SX1280_GetLoRaBandwidth( ) * 1000.0 );
    return efeHz;
}
