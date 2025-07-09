#ifndef ENCODERTIM2_PA0_PA1_H 
#define ENCODERTIM2_PA0_PA1_H

#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"


uint32_t tmpcnt=0;
int Counter=0;

uint8_t Rotate;
int dsp(int cnt);
uint32_t encoder_read(uint32_t offset);
uint8_t EC12_1SCAN (void);

void EC12_1Handle (void); // This function can be put into interrupt processing

#endif		