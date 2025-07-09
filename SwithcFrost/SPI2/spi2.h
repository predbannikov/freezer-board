
#ifndef SPI2_H
#define SPI2_H
#include "stdint.h"
 
#define CS        GPIO_Pin_12
#define DRDY      GPIO_Pin_11
#define SCK GPIO_Pin_13
#define MOSI GPIO_Pin_15
#define MISO GPIO_Pin_14

void    _SPI2_Init(void);
void     SPI2_Write(uint8_t data);
uint8_t  SPI2_Read(void);
void     SPI_Send_Byte(uint8_t data);
void     SPI2_Tx(uint8_t *data, int32_t len);
void exti_PB11(void);
 
#endif
