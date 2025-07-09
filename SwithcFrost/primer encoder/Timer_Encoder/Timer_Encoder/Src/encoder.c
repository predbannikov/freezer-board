/*
 * encoder.c
 *
 *  Created on: Jan 19, 2023
 *      Author: hussamaldean
 */


#include "encoder.h"
#include "stm32f1xx.h"

void encoder_init(uint16_t max_value)
{
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;

	/*Configure PA0 as Output Alternate Push/Pull */
	GPIOA->CRL&=~GPIO_CRL_MODE0;
	GPIOA->CRL|=(GPIO_CRL_CNF0_0);
	GPIOA->CRL&=~(GPIO_CRL_CNF0_1);

	/*Configure PA1 as Output Alternate Push/Pull*/

	GPIOA->CRL&=~GPIO_CRL_MODE1;
	GPIOA->CRL|=(GPIO_CRL_CNF1_0);
	GPIOA->CRL&=~(GPIO_CRL_CNF1_1);


	/*Enable clock access to timer2*/
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;

	/*Configure timer2*/
	TIM2->ARR=max_value-1;

	TIM2->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 );
	TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
	TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM2->CR1 |= TIM_CR1_CEN;


}

uint16_t encoder_read(void){

	return  TIM2->CNT;
}
