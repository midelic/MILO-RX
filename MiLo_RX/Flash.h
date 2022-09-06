
#include <EEPROM.h>
#define EEPROM_SIZE 256

void  EEPROMWriteInt(uint32_t p_address, uint16_t p_value)
{
	uint8_t Byte1 = ((p_value >> 0) & 0xFF);
	uint8_t Byte2 = ((p_value >> 8) & 0xFF);
	EEPROM.write((p_address),Byte1);
	EEPROM.write((p_address+1),Byte2);
}

uint16_t  EEPROMReadInt(uint32_t p_address)
{
	uint8_t  Byte1;
	uint8_t Byte2;
	Byte1= EEPROM.read(p_address);
	Byte2 = EEPROM.read(p_address+1);
	uint16_t firstTwoBytes = ((Byte1 << 0) & 0xFF) + ((Byte2 << 8) & 0xFF00);
	return (firstTwoBytes);
}

void StoreEEPROMdata(int startAddress)
{
	
	EEPROM.write(startAddress,MiLoStorage.txid[0]);
	EEPROM.write(startAddress+1,MiLoStorage.txid[1]);
	EEPROM.write(startAddress+2,MiLoStorage.rx_num);
	EEPROM.write(startAddress+3,MiLoStorage.chanskip);
	for(uint8_t i = 0;i < 8;i++)
	EEPROMWriteInt(startAddress+4+2*i,MiLoStorage.FS_data[i]);
	startAddress += sizeof(MiLoStorage);
	for(uint8_t i = 0;i < 4;i++)
	EEPROM.write(startAddress+i,MProtocol_id >> (i*8));
	EEPROM.commit();
}

void ReadEEPROMdata(uint32_t  startAddress)
{
	MiLoStorage.txid[0] = EEPROM.read(startAddress);
	MiLoStorage.txid[1] = EEPROM.read(startAddress+1);
	MiLoStorage.rx_num  = EEPROM.read(startAddress+2);
	MiLoStorage.chanskip = EEPROM.read(startAddress+3);
	
	for(uint8_t i = 0;i<8;i++){
		MiLoStorage.FS_data[i] = EEPROMReadInt(startAddress+4+2*i);
	}
	startAddress += sizeof(MiLoStorage);
	for(uint8_t i = 4;i > 0;i--)
	{
		MProtocol_id<<=8;
		MProtocol_id |=  EEPROM.read(startAddress+i-1);
	}	
}