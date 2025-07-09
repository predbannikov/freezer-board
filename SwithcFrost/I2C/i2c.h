

#ifndef _I2C_H_
#define _I2C_H_

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>
#include <stdio.h>

void 				I2C2_init22(void);
void 				I2C2_init222(void);
void 				I2C2_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
uint8_t 		I2C2_read_ack(I2C_TypeDef* I2Cx);
uint8_t 		I2C2_read_nack(I2C_TypeDef* I2Cx);
void 				I2C2_stop(I2C_TypeDef* I2Cx);
void 				I2C2_write(I2C_TypeDef* I2Cx, uint8_t data);
void delay_ms2(uint32_t ms);   

#endif
