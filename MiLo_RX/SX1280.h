
//SX1280 RFfunctions
uint16_t ICACHE_RAM_ATTR SX1280_GetIrqStatus( void );
void ICACHE_RAM_ATTR SX1280_ClearIrqStatus( uint16_t irqMask );
bool  ICACHE_RAM_ATTR SX1280_Begin(void);
void ICACHE_RAM_ATTR SX1280_SetMode(uint8_t  OPmode);
void  ICACHE_RAM_ATTR SX1280_ReadBuffer( uint8_t offset, volatile uint8_t *buffer, uint8_t size);
uint8_t  ICACHE_RAM_ATTR SX1280_ReadReg(uint16_t address);
void  ICACHE_RAM_ATTR SX1280_WriteReg(uint16_t address, uint8_t data);
void  ICACHE_RAM_ATTR SX1280_WriteRegisterMulti(uint16_t address, uint8_t *data, uint8_t size);
void  ICACHE_RAM_ATTR SX1280_ReadRegisterMulti(uint16_t address, uint8_t *data, uint8_t size);
void  ICACHE_RAM_ATTR SX1280_WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size);
void  ICACHE_RAM_ATTR SX1280_WriteCommand(uint8_t command, uint8_t val,uint32_t busyDelay);
void  ICACHE_RAM_ATTR SX1280_WriteCommandMulti(uint8_t command, uint8_t *data, uint16_t size,uint32_t busyDelay);
void  ICACHE_RAM_ATTR SX1280_ReadCommand(uint8_t command, uint8_t data);
void  ICACHE_RAM_ATTR SX1280_ReadCommandMulti(uint8_t command, uint8_t *data, uint8_t size);
void  ICACHE_RAM_ATTR SX1280_Reset(void);
uint16_t ICACHE_RAM_ATTR SX1280_GetFirmwareVersion( void );
void ICACHE_RAM_ATTR SX1280_SetFrequency(uint32_t frequency);
void ICACHE_RAM_ATTR SX1280_SetFrequencyReg(uint32_t frequency);
void ICACHE_RAM_ATTR SX1280_SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
void ICACHE_RAM_ATTR SX1280_ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr);
void ICACHE_RAM_ATTR SX1280_SetPacketParamsLoRa(uint8_t PreambleLength, uint8_t HeaderType,uint8_t PayloadLength, uint8_t crc,uint8_t InvertIQ);
void ICACHE_RAM_ATTR SX1280_SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
uint8_t ICACHE_RAM_ATTR SX1280_GetRxBufferAddr();
uint8_t  ICACHE_RAM_ATTR SX1280_GetStatus(void);
int8_t ICACHE_RAM_ATTR SX1280_GetRssiInst( void );
void ICACHE_RAM_ATTR SX1280_GetLastPacketStats(void);
uint8_t ICACHE_RAM_ATTR SX1280_GetPacketType(void);
bool ICACHE_RAM_ATTR WaitOnBusy(void);
void ICACHE_RAM_ATTR BusyDelay(uint32_t duration);
int32_t ICACHE_RAM_ATTR SX1280_complement2( const uint32_t num, const uint8_t bitCnt );
void  ICACHE_RAM_ATTR SX1280_Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,uint8_t PreambleLength, bool InvertIQ, uint8_t _PayloadLength);
void ICACHE_RAM_ATTR  SX1280_SetRxTimeoutUs(uint32_t interval);
void  ICACHE_RAM_ATTR getRFlinkInfo(void);
void  ICACHE_RAM_ATTR SX1280_setPower(uint8_t Power);
void  ICACHE_RAM_ATTR SX1280_SetOutputPower( int8_t power );
int32_t ICACHE_RAM_ATTR SX1280_GetLoRaBandwidth(void);
int32_t ICACHE_RAM_ATTR complement2( const uint32_t num, const uint8_t bitCnt );
double ICACHE_RAM_ATTR SX1280_GetFrequencyError(void);
uint32_t SpreadingFactorToRSSIvalidDelayUs(uint8_t SF);
int8_t ICACHE_RAM_ATTR PowerEnumToLBTLimit(uint8_t txPower);
void ICACHE_RAM_ATTR BeginClearChannelAssessment(void);
bool ICACHE_RAM_ATTR ChannelIsClear(void);
