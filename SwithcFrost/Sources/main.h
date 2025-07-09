

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
//#include "stm32f10x_rcc.h"
#include "bkp.h"
#include "misc.h"
#include "onewire.h"
#include "at24c02.h"
#include <stdlib.h>
#include <stdio.h>
#include "spi2.h"
//#include "encoderTIM2_PA0_PA1.h"

#define UWORD unsigned short //65536                     16bit
#define UBYTE unsigned char  //255                        8bit
#define ULONG unsigned long long //18446744073709551615  64bit
//---------------------------------------------------------------

#define OUT_A         GPIOA   //выходы	
#define OUT_B         GPIOB   //выходы	
#define OUT_C         GPIOC   //выходы	
#define OUT_D         GPIOD   //выходы	


#define PA6          GPIO_Pin_6
#define BLUE       GPIO_Pin_10    
#define RED        GPIO_Pin_8    
#define GREEN      GPIO_Pin_9      
//#define pa15          GPIO_Pin_15
//#define AIN_PA1          GPIO_Pin_1 	 
//#define LED        GPIO_Pin_13 //   GPIO_Pin_6  //PB6
//#define STOP         GPIO_Pin_7  //PB7



//При таком предделителе у меня получается один тик таймера на 10 мкс
#define TIMER_PRESCALER	   719//480
void encoder_TIM2_init(void);
void encoder_TIM4_init(void);
 void init_adc_PA3(void);
void TIM3_PWM_Init2(u16 arr,u16 psc);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void delay(unsigned int countDown);
void NVIC_Configuration(void);
void RTC_Configuration(void); // 
void GPIO_Configuration(void);
void ADC_setup(void);
//void register_red(void);
void ADC_simple(void);

void Init(void);
void init_adc(void);
void init_adc_pa0(void);
//--------------------------------------
int i,counter=0;
unsigned int result_1;//данные от АЦП

//NVIC_InitTypeDef NVIC_InitStruct2;
GPIO_InitTypeDef GPIO_InitStruct;
EXTI_InitTypeDef EXTI_InitStruct; 
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef timer;
RTC_TypeDef rtc;

//---------------- конфигурация выводов прерываний по таймеру --- сдесь неиспользуется но код оставил, вдруг что то надо будет отследить на каком нибудь пине-------------------
void NVIC_Configuration(void)
{



/*NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
ADC1_IRQn  stm32f10x.hNVIC_InitTypeDef NVIC_InitStructure;
NVIC_InitStructure.NVIC_IRQChannel =ADC1_2_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

NVIC_EnableIRQ (ADC1_2_IRQn); // Разрешаем прерывания от АЦП

NVIC_EnableIRQ (RTC_IRQn); // Разрешаем прерывания от 
*/
}

void InitTim4(void)
{
    TIM_TimeBaseInitTypeDef Timer;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructInit(&Timer);

    Timer.TIM_Prescaler = 35999;      // 36 МГц → 1 кГц (1 мс на тик)
    Timer.TIM_Period = 2200 - 1;       // 750 тиков → 100 мс

    TIM_TimeBaseInit(TIM4, &Timer);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    NVIC_EnableIRQ(TIM4_IRQn);
}


//void InitTim4(void)
//{
///*
//SystemCoreClock - это магическое  72000000
//Если да, то 72000000/1000-1 = 71999 > 65535
//*/
//TIM_TimeBaseInitTypeDef Timer;
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
//   TIM_TimeBaseStructInit(&Timer);
//  // Timer.TIM_Prescaler = SystemCoreClock / 1000 - 1;
//  // Timer.TIM_Period = 1000;
//   //--------------- 1 sec---------------------------------
//   Timer.TIM_Prescaler = (SystemCoreClock / 2) / 1000 - 1; //35999
//   Timer.TIM_Period = 1000 * 5;
//   
//   
//   
//   TIM_TimeBaseInit(TIM4, &Timer);
//   TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
//   TIM_Cmd(TIM4, ENABLE);
//   NVIC_EnableIRQ(TIM4_IRQn);
//
//}

 

//----------------- настройка частоты контроллера и тактирования ---------------------
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    // RCC system reset(for debug purpose) */
    RCC_DeInit();

    // Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    // Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        // HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        // PCLK1 = HCLK */
        RCC_PCLK1Config(RCC_HCLK_Div2); //8
 
        // Select HSE as system clock source */
       RCC_PLLConfig(0x00010000, RCC_PLLMul_9);// 8mhz*9 =72


        // Enable PLL */
        RCC_PLLCmd(ENABLE);

        // Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

        // Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // Wait till HSE is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08){}
    }

  	// разрешение тактирования  */
   // Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA
    											| RCC_APB2Periph_GPIOB
    											, ENABLE);

}
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
NVIC_EnableIRQ (RTC_IRQn); // Разрешаем прерывания от 

    // разрешение тактирования  */
   
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR  | RCC_APB1Periph_BKP, ENABLE);
  
                                                
                                                
  // RCC_DeInit();                                             

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
   
   
  // Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(	RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA
    											| RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO
                                                | RCC_APB2Periph_GPIOC
                                                | RCC_APB2Periph_GPIOD
    											, ENABLE); /**/
   
}

void RTC_IRQHandler(void)
{
   if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
   { 
      RTC_ClearITPendingBit(RTC_IT_SEC);
      RTC_WaitForLastTask();
   
   }
   
     //----------- точка в часах -----------
  
   //times=RTC_GetCounter();
}
//------------------------------------
void adc_init()
{
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

// настройки ADC
ADC_InitTypeDef ADC_InitStructure;
ADC_StructInit(&ADC_InitStructure);
ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // режим работы - одиночный, независимый
ADC_InitStructure.ADC_ScanConvMode = DISABLE; // не сканировать каналы, просто измерить один канал
ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // однократное измерение
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // без внешнего триггера
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //выравнивание битов результат - прижать вправо
ADC_InitStructure.ADC_NbrOfChannel = 1; //количество каналов - одна штука
ADC_Init(ADC1, &ADC_InitStructure);
ADC_Cmd(ADC1, ENABLE);

// настройка канала
ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);

// калибровка АЦП
ADC_ResetCalibration(ADC1);
while (ADC_GetResetCalibrationStatus(ADC1));
ADC_StartCalibration(ADC1);
while (ADC_GetCalibrationStatus(ADC1));
}

void init_adc_PA3(void)//PA3 датчик тока
{
//ADC
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    // input of ADC (it doesn't seem to be needed, as default GPIO state is floating input)
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;        // that's ADC1 (PA3 on STM32)
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    //clock for ADC (max 14MHz --> 72/6=12MHz)
    RCC_ADCCLKConfig (RCC_PCLK2_Div6);
    // enable ADC system clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
 
    // define ADC config
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // we work in continuous sampling mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
 
    ADC_RegularChannelConfig(ADC1,ADC_Channel_3, 0,ADC_SampleTime_1Cycles5); // define regular conversion config
  //  ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 1,ADC_SampleTime_28Cycles5); // define regular conversion config
  
  ADC_Init ( ADC1, &ADC_InitStructure);   //set config of ADC1
 
    // enable ADC
    ADC_Cmd (ADC1,ENABLE);  //enable ADC1
 
    //  ADC calibration (optional, but recommended at power on)
    ADC_ResetCalibration(ADC1); // Reset previous calibration
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); // Start new calibration (ADC must be off at that time)
    while(ADC_GetCalibrationStatus(ADC1));
 
 //--------------- прописал в таблице прерываний -------------------------------------------	 
  
NVIC_InitStructure.NVIC_IRQChannel =ADC1_2_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
NVIC_EnableIRQ (ADC1_2_IRQn); // Разрешаем прерывания от АЦП
 
    // start conversion
    ADC_Cmd (ADC1,ENABLE);  //enable ADC1
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // start conversion (will be endless as we are in continuous mode)


}



//--------------- конфигураци камня-------------------------
void gpio_init(void)
{
	 /* Set variables used */
   
 
    /* Enable clock for AFIO */
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB  ,ENABLE); 
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //Enable GPIO peripheral and AFIO multiplexing function module clock

   
	/*
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_12 ;//| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//         GPIO_Mode_Out_OD;//GPIO_Mode_IPU; GPIO_Mode_Out_OD 
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
   // GPIO_Init(GPIOB, &GPIO_InitStruct);
		  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_11 ;//rele start 
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//         GPIO_Mode_Out_OD;//GPIO_Mode_IPU; GPIO_Mode_Out_OD 
   //PB2 PB12 PB11
  	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	*/

 
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6 ;//PA6  Start motor 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP ;//      GPIO_Mode_Out_OD;//GPIO_Mode_IPU; GPIO_Mode_Out_OD 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.GPIO_Pin =  BLUE;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//    
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.GPIO_Pin =  GREEN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//    
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.GPIO_Pin =  RED;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//    
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	
	
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_7 ;//PA6  Start motor 
  GPIO_InitStruct.GPIO_Mode =   GPIO_Mode_Out_PP ;//      GPIO_Mode_Out_OD;//GPIO_Mode_IPU; GPIO_Mode_Out_OD 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStruct);
/**/
	/*GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_7 ;//PA 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;//  GPIO_Mode_Out_PP       GPIO_Mode_Out_OD;//GPIO_Mode_IPU; GPIO_Mode_Out_OD 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStruct);*/

	//Pb2  fuse начало фазы
/*		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//   
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
	
    NVIC_InitStructure.NVIC_IRQChannel =EXTI2_IRQn;// EXTI2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 		//Pb2
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
    EXTI_InitStruct.EXTI_Line =EXTI_Line2; 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger =EXTI_Trigger_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising_Falling;//  EXTI_Trigger_Rising
    EXTI_Init(&EXTI_InitStruct);
		NVIC_EnableIRQ(EXTI_Line2);*/
//****************************************************************

	

	//PA0  encoder
 	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//  A   
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	
      NVIC_InitStructure.NVIC_IRQChannel =EXTI0_IRQn;// EXTI0_IRQn; 
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	  
    EXTI_InitStruct.EXTI_Line =EXTI_Line0; 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger =EXTI_Trigger_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising_Falling;//  EXTI_Trigger_Rising
    EXTI_Init(&EXTI_InitStruct);
		NVIC_EnableIRQ(EXTI_Line0);
//******************************************
  //PA1  encoder
 	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//  A   
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
		
      NVIC_InitStructure.NVIC_IRQChannel =EXTI1_IRQn;// EXTI0_IRQn; 
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
	   GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
	  
    EXTI_InitStruct.EXTI_Line =EXTI_Line1; 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger =EXTI_Trigger_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising_Falling;//  EXTI_Trigger_Rising
    EXTI_Init(&EXTI_InitStruct);
    NVIC_EnableIRQ(EXTI_Line1);
//********************************************************
  //PA4  button
 	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//  A   
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
		
      NVIC_InitStructure.NVIC_IRQChannel =EXTI4_IRQn;// EXTI4_IRQn; 
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
	   GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
	  
    EXTI_InitStruct.EXTI_Line =EXTI_Line4; 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger =EXTI_Trigger_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising_Falling;//  EXTI_Trigger_Rising
    EXTI_Init(&EXTI_InitStruct);
    NVIC_EnableIRQ(EXTI_Line4);
//**********************************************************
//PA5 fuse начало фазы
GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//  A   
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
		
      NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn ;// EXTI9_5_IRQn ; 
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
	   GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
	  
    EXTI_InitStruct.EXTI_Line =EXTI_Line5 ; 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger =EXTI_Trigger_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising_Falling;//  EXTI_Trigger_Rising
    EXTI_Init(&EXTI_InitStruct);
    NVIC_EnableIRQ(EXTI_Line5 );
//**********************************************************
	//pb12 EXTI15_10_IRQHandler
	/*  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//     
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
	
   	 NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;// EXTI15_10_IRQn; 
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);//pb12
	 
    EXTI_InitStruct.EXTI_Line =EXTI_Line12; 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger =EXTI_Trigger_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising;//EXTI_Trigger_Rising_Falling;//  EXTI_Trigger_Rising
    EXTI_Init(&EXTI_InitStruct);
		NVIC_EnableIRQ(EXTI_Line12);
	*/
/*		 NVIC_SetPriority(EXTI_Line0,1);//PA0 encoder
	 NVIC_EnableIRQ(EXTI_Line0);
	  NVIC_SetPriority(EXTI_Line12, 0);//PB12 button
	 NVIC_EnableIRQ(EXTI_Line12);*/
	/**/ 
//****************************************************************
		
}	
	
 
void delay(unsigned int countDown)
{
          // arbitrary int to count down

  while(countDown--);                      // count down
}


//---------------- запуск АЦП 13,5 циклов------------------------
/*void ADC_simple(void)
{
 ADC_Cmd (ADC1,ENABLE);	//enable ADC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5); // поставил на всякий случай, но работает и без него
 ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
}*/
//--------------------------------
// Select the input channel 1, 2 of the timer 4 to pick up the encoder AB phase
void encoder_TIM4_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);// Enable Timer 4 Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// Enable GPIOB Clock
	
	// TIM4_CH1, TIM4_CH2 corresponds to PB.6 PB.7 * /
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;// pull-up input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);                           
	
	// * Configure Timer 4 * /
	TIM_TimeBaseStructure.TIM_Period = 65535;// The preload value is full
	TIM_TimeBaseStructure.TIM_Prescaler = 0;// is not divided
	TIM_TimeBaseStructure.TIM_ClockDivision =0 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// Count
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              
              
//	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	// TIMX_SMCR register SMS bit selection encoder mode selection SMS = 011; ie the upper and lower edges are simultaneously triggered
TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising | TIM_ICPolarity_Falling ,TIM_ICPolarity_Rising | TIM_ICPolarity_Falling);	


	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; // CC1S = 01 input channel IC1, 2 map to encoder interface channel Ti1, 2
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising | TIM_ICPolarity_Falling;//TIM_ICPolarity_BothEdge;	// bilateral edge trigger
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // Direct input
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 // is not divided
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;// Configure input filtering, not filtered


	TIM_ICInit(TIM4, &TIM_ICInitStructure);// Timer 4 interrupt configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
	
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // Clear status flag
	TIM_SetCounter(TIM4,0); // TIM4-> CNT = 0; counter value is clear
	TIM_Cmd(TIM4, ENABLE);   // Enable TIM4
}

void encoder_TIM2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);// Enable Timer 2 Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// Enable GPIOB Clock
	
	// TIM2_CH1, TIM2_CH2 corresponds to Pa.0 Pa.1 * /
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;// pull-up input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);                           
	GPIO_PinRemapConfig( GPIO_FullRemap_TIM2, ENABLE ); // Map TIM2 to GPIOA
	
	// * Configure Timer 2 * /
	TIM_TimeBaseStructure.TIM_Period =  2000;// The preload value is full
	TIM_TimeBaseStructure.TIM_Prescaler = 0;// is not divided
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1;//0 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// Count
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              
              
//	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	// TIMX_SMCR register SMS bit selection encoder mode selection SMS = 011; ie the upper and lower edges are simultaneously triggered
TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, 
                               /*  TIM_ICPolarity_Rising  |*/ TIM_ICPolarity_Falling ,
															/* TIM_ICPolarity_Rising  |*/ TIM_ICPolarity_Falling);	


	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; // CC1S = 01 input channel IC1, 2 map to encoder interface channel Ti1, 2
 	TIM_ICInitStructure.TIM_ICPolarity = /*TIM_ICPolarity_Rising|*/ TIM_ICPolarity_Falling;//TIM_ICPolarity_BothEdge;	// bilateral edge trigger
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // Direct input
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 // is not divided
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;// Configure input filtering, not filtered


	TIM_ICInit(TIM2, &TIM_ICInitStructure);// Timer 4 interrupt configuration
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
	
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // Clear status flag
	TIM_SetCounter(TIM2,0); // TIM4-> CNT = 0; counter value is clear
	TIM_Cmd(TIM2, ENABLE);   // Enable TIM4
}


//***************
void Tim3_init(u16 arr,u16 psc)
{
    // Подаем тактовую частоту на таймеры и порт A
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  //  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Конфигурация порта
    GPIO_InitTypeDef GPIO_Config;
    // Пины 0, 6, 7
    GPIO_Config.GPIO_Pin =  GPIO_Pin_7;
    // Альтернативная функция (в нашем случае - выход таймера), Push-Pull
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    // Частота - 50 MHz
    GPIO_Config.GPIO_Speed = GPIO_Speed_50MHz;

    // Инициализируем порт A этой конфигурацией
    GPIO_Init(GPIOA, &GPIO_Config);

    // Конфигурация таймера
    TIM_TimeBaseInitTypeDef TIM_BaseConfig;
    // Конфигурация выхода таймера
    TIM_OCInitTypeDef TIM_OCConfig;

    // Запускаем таймер на тактовой частоте в 7200 kHz
    TIM_BaseConfig.TIM_Prescaler =psc;// (uint16_t) (SystemCoreClock / 7200000) - 1;
    // Период - 150 тактов => 4800/150 = 32 kHz
    TIM_BaseConfig.TIM_Period =150; //arr;//1999;
    TIM_BaseConfig.TIM_ClockDivision = 0;
    // Отсчет от нуля до TIM_Period
    TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

    // Инициализируем таймер №3 (его выходы как раз на порту A)
    TIM_TimeBaseInit(TIM3, &TIM_BaseConfig);

    // Конфигурируем выход таймера, режим - PWM1
    TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
    // Собственно - выход включен
    TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
    // Пульс длинной 75 тактов => 75/150 = 50%
    TIM_OCConfig.TIM_Pulse =75;// arr/2;//1000;
    // Полярность => пульс - это единица (+3.3V)
    TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

    // Инициализируем первый выход таймера №3 (у HD это PA6)
    TIM_OC1Init(TIM3, &TIM_OCConfig);
/*
    // Конфигурируем второй выход таймера
    TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
    // Пульс длинной 30 тактов => 30/150 = 20%
    TIM_OCConfig.TIM_Pulse = 29;
    // Ради эксперемента, меняем полярность (пульс - 0V).
    TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_Low;

    // Инициализируем второй выход таймера №3 (PA7)
    TIM_OC2Init(TIM3, &TIM_OCConfig);
*/
    // Как я понял - автоматическая перезарядка таймера, если неправ - поправте.
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // Включаем таймер
    TIM_Cmd(TIM3, ENABLE);

    // Тоже самое, только теперь таймер №5 на частоте 10 kHz
  /*  TIM_BaseConfig.TIM_Prescaler =  (SystemCoreClock / 10000) - 1;
    // Мигаем светодиодом 1 раз в секунду.
    TIM_BaseConfig.TIM_Period = 9999;
    TIM_BaseConfig.TIM_ClockDivision = 0;
    TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM5, &TIM_BaseConfig);

    TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCConfig.TIM_Pulse = 4999;
    TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

    // Первый выход - PA0
    TIM_OC1Init(TIM5, &TIM_OCConfig);

    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM5, ENABLE);

    TIM_Cmd(TIM5, ENABLE);*/
		
}
//**********************
void PWM_TIM3_Init222(void)
{ // установка начальных значений шима
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t CCR1_Val = 333;
  uint16_t CCR2_Val = 249;
  uint16_t PrescalerValue = 0;

   // разрешить клокирование ШИМа
  /* System Clocks Configuration */
 /* TIM3 clock enable */ // разрешить клокирование TIM3 и альтернативного ввода-
                          // вывода на порт B
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  // установить кофигурацию штырей ввода-вывода для ШИМа
  /* GPIO Configuration */
 GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;//6|7
  GPIO_Init(GPIOA, &GPIO_InitStructure);	 
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3 partial remap TIM3_CH2->PB5    
  // инициализация TIM3 режиме ШИМ целиком выдрана из примеров к стандартной
  // библиотеке STM32  используются 2 из 4х каналов ШИМ
  /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 72 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at  KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 72 MHz / 1800 = 40 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 72000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 1999;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
 /**/
  /* PWM1 Mode configuration: Channel2 */
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
/* */
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
} 
