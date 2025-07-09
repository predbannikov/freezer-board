#ifndef __DS18B20_H
#define __DS18B20_H 

#include "main.h"
#include "stdio.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// IO direction setting
#define DS18B20_IO_IN()  {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=8<<0;}
#define DS18B20_IO_OUT() {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=3<<0;}

// IO operation function											   
#define	DS18B20_DQ_IN  HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin)  // Data port PA0 

#define DS18B20_DQ_OUT_H()	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET)
#define DS18B20_DQ_OUT_L()	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET)

u8 DS18B20_Init(void);			// Initialize DS18B20
float DS18B20_Get_Temp(void);	// Get the temperature
void DS18B20_Start(void);		// Start temperature conversion
void DS18B20_Write_Byte(u8 dat);// write a byte
u8 DS18B20_Read_Byte(void);		// Read out a byte
u8 DS18B20_Read_Bit(void);		// Read a bit
u8 DS18B20_Check(void);			// Check for the presence of DS18B20
void DS18B20_Rst(void);			// Reset DS18B20    
#endif

