
#include "stm32f10x_i2c.h"
#include "i2c.h"  // delay_ms

#ifndef AT24C02_H_
#define AT24C02_H_

void AT24C02_Write(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte, unsigned char Value);
unsigned char AT24C02_Read(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte);
uint8_t I2C_EE_ByteRead( uint16_t ReadAddr);
void RecToEEPROM(uint16_t Adr_OffSet_EPROM,	char Data[], uint8_t Len);
int Read_ByteEEPROM(char read_to_buf[], int ReadAddr,uint8_t Len);
#endif /* AT24C02_H_ */
