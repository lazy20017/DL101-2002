#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H


#include "spi.h"








//********WDZ*****
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


void SPI_EEPROM_Init(void);





void SPI_EE_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite,uint8_t NSS);
void SPI_EE_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite,uint8_t NSS);

void SPI_EE_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead,uint8_t NSS);

void SPI_EE_BufferWrite2(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);  //写缓冲区数据到 EEPROM中
void SPI_EE_BufferRead2(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);   //读 EEPROM中数据到 缓冲区中


uint8_t AT25_ReadByte(uint32_t addr);
void AT25_WriteByte(uint8_t data, uint32_t addr);

uint8_t SPI_WriteByte(uint8_t data);
uint8_t SPI_ReadByte(void);


void SPI_WREN(uint8_t nss);
void SPI_WRDI(uint8_t nss);
void NSS_CS_ENABLE(uint8_t NSS);
void NSS_CS_DISABLE(void);

void SPI_EEPROM_WRITE_Start(void);
void SPI_EEPROM_WRITE_END(void);
void SPI_Read_status(void);

#endif /* __SPI_FLASH_H */

