#include "stm32f10x.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_rcc.h"
#include "bkp.h"
#include "misc.h"
 NVIC_InitTypeDef NVIC_InitStructure;
void RTC_Config(void)
{
  

   /* Configure one bit for preemption priority */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

   /* Enable the RTC Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
   // разрешение тактирования  */
   // Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_ADC1 | RCC_APB1Periph_PWR 
                                                | RCC_APB1Periph_BKP
                                                | RCC_APB2Periph_GPIOA
    											| RCC_APB2Periph_GPIOB
    											, ENABLE);

   PWR_BackupAccessCmd(ENABLE);
   RCC_LSEConfig(RCC_LSE_ON);
   while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
   RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
   RCC_RTCCLKCmd(ENABLE);

   RTC_WaitForSynchro();
   RTC_WaitForLastTask();
   RTC_ITConfig(RTC_IT_SEC, ENABLE);
   RTC_WaitForLastTask();
   RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
   RTC_WaitForLastTask();
}

void RTC_IRQHandler(void)
{
   if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
   {
      RTC_ClearITPendingBit(RTC_IT_SEC);
      RTC_WaitForLastTask();
   }
   //times=RTC_GetCounter();
}
//------------------------------------

/*
У STM32F103C8T6 нет поддержки календаря. 
Там просто счетчик, который инкрементируется раз в секунду. 
Как правило за ноль счетчика принимают дату 1 января 1970 года, и счетчик показывает количество секунд, 
прошедшее с этого момента времени. День недели, год, месяц, число высчтитываются программно.
*/
