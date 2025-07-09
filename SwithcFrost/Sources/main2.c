/*







*/

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_adc.h"
#include "misc.h"

//-----------------------------------

#define OUT_RELE GPIOA   //выходы	 
#define IN_LINE  GPIOB   // входы 

#define DS     GPIO_Pin_8   //A8   данные для регистра  
#define ST_CP  GPIO_Pin_9   //A9   защёлкнуть данные
#define SH_CP  GPIO_Pin_10  //A10 сдвиг данных

//#define AIN0   GPIO_Pin_0   // PA0 -ADC
#define Line1  GPIO_Pin_0   //PB0	первая входящая линия
#define Line2  GPIO_Pin_1   //PB1
#define Line3  GPIO_Pin_10   //PB10
#define Line4  GPIO_Pin_11   //PB11
#define Line5  GPIO_Pin_12   //PB12
#define Line6  GPIO_Pin_13   //PB13
//PB15 AIN

#define Rele1  GPIO_Pin_2  //PA2	первое реле для входящей линии 1
#define Rele2  GPIO_Pin_3  //PA3
#define Rele3  GPIO_Pin_4  //PA4
#define Rele4  GPIO_Pin_5  //PA5
#define Rele5  GPIO_Pin_6  //PA6
#define Rele6  GPIO_Pin_7   //PA7


NVIC_InitTypeDef NVIC_InitStructure;

//-----------------------------------
int i,counter=0;
unsigned int result_1;//данные от АЦП
int PB0_on,PB1_on,PB2_on,PB3_on,PB4_on,PB5_on,test_leds=0;  //отслеживание
unsigned char  register_dat_red[24];
//----------------------------------------------------------------------
void RCC_Configuration(void);
void GPIO_Configuration(void);
void delay_(void);
void NVIC_Configuration(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_setup(void);
void  delay_(void);
void register_red(void);
void ADC_simple(void);
void test_led(void);
void test_circut(void);
//----------------------------------------------------------------------
//---------------- конфигурация выводов прерываний----------------------
void NVIC_Configuration(void)
{


    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
/*
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//ADC1_IRQn  stm32f10x.hNVIC_InitTypeDef NVIC_InitStructure;
NVIC_InitStructure.NVIC_IRQChannel =ADC1_2_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

NVIC_EnableIRQ (ADC1_2_IRQn); // Разрешаем прерывания от АЦП
*/
NVIC_EnableIRQ (RTC_IRQn); // Разрешаем прерывания от 

}
//--------------------------- 1 sec--------------------
 void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

      /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}
//----------------- проверка входящих линий -----------
void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        /* Clear the RTC Second interrupt */
        RTC_ClearITPendingBit(RTC_IT_SEC);

        //--------------------------------------
       
       //   if(PB0_on==0 && PB1_on==0  && PB2_on==0  && PB3_on==0  && PB4_on==0   && PB5_on==0 ) ADCSRA=0;

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }
}
//------------------- определить нажатую кнопку -------
void ADC1_2_IRQHandler(void)
{
	// не очень хороший пример как работать в прерывание, но сдесь проходит.
 //if (ADC_GetITStatus(ADC1, ADC_IT_EOC))
// {     
   ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    result_1 = ADC_GetConversionValue(ADC1);         
if(result_1<=0x200) goto ed;
//-------------------------- SW1 ---------------9-chanel
else  if(result_1 >= 0x390 && result_1 <= 0x410 )
     {  
			
			 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();
			   register_dat_red[19]=1;// r sw1
			//  register_dat_red[18]=1;// r sw1
			 	register_red();
			 
			// test_led();	
     if(PB0_on ==1)// был сделан вызов, ожидает на линии
       {// если другие каналы были подключены, то выключить их
         if(PB1_on==3){ PB1_on =0;  GPIO_WriteBit(OUT_RELE,Rele2,Bit_RESET); } // 2- chanel off
         if(PB2_on==3){ PB2_on =0;  GPIO_WriteBit(OUT_RELE,Rele3,Bit_RESET); } // 3- chanel off  
         if(PB3_on==3){ PB3_on =0;  GPIO_WriteBit(OUT_RELE,Rele4,Bit_RESET); } // 4- chanel off 
         if(PB4_on==3){ PB4_on =0;  GPIO_WriteBit(OUT_RELE,Rele5,Bit_RESET); } // 5- chanel off 
         if(PB5_on==3){ PB5_on =0;  GPIO_WriteBit(OUT_RELE,Rele6,Bit_RESET); } // 6- chanel off 
          
				 GPIO_WriteBit(OUT_RELE,Rele1,Bit_SET);  // 1- chanel on
				 PB0_on=3;  // 1 канал на линии
				 
        }
			 goto ed;
     }
//---------------------------SW2----------------5-chanel
 else if(result_1 >= 0x535 && result_1 <= 0x570 )
     { 
			 
			 	 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red(); register_dat_red[12]=0; register_dat_red[13]=1;
			  			 	register_red();
			 
			 
			 
			 
			

       if(PB1_on ==1)
       {
         if(PB0_on==3) { PB0_on =0;  GPIO_WriteBit(OUT_RELE,Rele1,Bit_RESET); } // 1- chanel off
         if(PB2_on==3) { PB2_on =0;  GPIO_WriteBit(OUT_RELE,Rele3,Bit_RESET); } // 3- chanel off  
         if(PB3_on==3) { PB3_on =0;  GPIO_WriteBit(OUT_RELE,Rele4,Bit_RESET); } // 4- chanel off  
         if(PB4_on==3) { PB4_on =0;  GPIO_WriteBit(OUT_RELE,Rele5,Bit_RESET); } // 5- chanel off 
         if(PB5_on==3) { PB5_on =0;  GPIO_WriteBit(OUT_RELE,Rele6,Bit_RESET); } // 6- chanel off 
	      
				 GPIO_WriteBit(OUT_RELE,Rele2,Bit_SET);  // 2- chanel on
				 PB1_on=3;  // 2 канал на линии
        }

goto ed;

      }
	
			
//--------------------------SW3-----------------1-chanel
 else if(result_1 >= 0x799  && result_1 <= 0x81f  )		
     {
			 
			 
			 	 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();
			  register_dat_red[4]=0; register_dat_red[5]=1;
			 	register_red();
			 
			 
			
     if(PB2_on ==1)
       {
         if(PB0_on==3) { PB0_on =0;  GPIO_WriteBit(OUT_RELE,Rele1,Bit_RESET); } // 1- chanel off
         if(PB1_on==3) { PB1_on =0;  GPIO_WriteBit(OUT_RELE,Rele2,Bit_RESET); } // 2- chanel off   //
         if(PB3_on==3) { PB3_on =0;  GPIO_WriteBit(OUT_RELE,Rele4,Bit_RESET); } // 4- chanel off  
         if(PB4_on==3) { PB4_on =0;  GPIO_WriteBit(OUT_RELE,Rele5,Bit_RESET); } // 5- chanel off 
         if(PB5_on==3) { PB5_on =0;  GPIO_WriteBit(OUT_RELE,Rele6,Bit_RESET); } // 6- chanel off 
     
	        GPIO_WriteBit(OUT_RELE,Rele3,Bit_SET);  // 3- chanel on
          PB2_on=3;  // 3 канал на линии
        }

goto ed;


      }
//------------------- SW4 -> SW5 ----------------- without led no use
 else if(result_1 >= 0xfc0  && result_1 <= 0xfff  )			
     {
			register_dat_red[20]=1; register_dat_red[21]=1; 
			 test_led();
    /* if(PB3_on ==1)
       {
         if(PB0_on==3) { PB0_on =0;  GPIO_WriteBit(OUT_RELE,Rele1,Bit_RESET); } // 1- chanel off
         if(PB1_on==3) { PB1_on =0;  GPIO_WriteBit(OUT_RELE,Rele2,Bit_RESET); } // 2- chanel off   //
         if(PB2_on==3) { PB2_on =0;  GPIO_WriteBit(OUT_RELE,Rele3,Bit_RESET); } // 3- chanel off  
         if(PB4_on==3) { PB4_on =0;  GPIO_WriteBit(OUT_RELE,Rele5,Bit_RESET); } // 5- chanel off 
         if(PB5_on==3) { PB5_on =0;  GPIO_WriteBit(OUT_RELE,Rele6,Bit_RESET); } // 6- chanel off 
     
	        GPIO_WriteBit(OUT_RELE,Rele4,Bit_SET);  // 4- chanel on
          PB3_on=3;  // 4 канал на линии
        }*/
goto ed;
      }
//--------------------SW5 10 chanel --------------10-chanel
else  if(result_1 >= 0x340   && result_1 <= 0x381   )				
     {
			 
			 
			 	 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();
			  register_dat_red[20]=0; register_dat_red[21]=1;
			 	register_red(); 
			 
			 
			if(PB3_on ==1)
       {
         if(PB0_on==3) { PB0_on =0;  GPIO_WriteBit(OUT_RELE,Rele1,Bit_RESET); } // 1- chanel off
         if(PB1_on==3) { PB1_on =0;  GPIO_WriteBit(OUT_RELE,Rele2,Bit_RESET); } // 2- chanel off   //
         if(PB2_on==3) { PB2_on =0;  GPIO_WriteBit(OUT_RELE,Rele3,Bit_RESET); } // 3- chanel off  
         if(PB4_on==3) { PB4_on =0;  GPIO_WriteBit(OUT_RELE,Rele5,Bit_RESET); } // 5- chanel off 
         if(PB5_on==3) { PB5_on =0;  GPIO_WriteBit(OUT_RELE,Rele6,Bit_RESET); } // 6- chanel off 
     
	        GPIO_WriteBit(OUT_RELE,Rele4,Bit_SET);  // 4- chanel on
          PB3_on=3;  // 4 канал на линии 
			 
	  

goto ed;
      }
		}	 
//---------------------SW6 ----------------------6-chanel
else  if(result_1 >= 0x44a  && result_1 <= 0x470  )			
     { 
		
			 
			 	 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();
				register_dat_red[14]=0; register_dat_red[15]=1; 
			 	register_red(); 
     
 if(PB4_on ==1)
       {
         if(PB0_on==3) { PB0_on =0;  GPIO_WriteBit(OUT_RELE,Rele1,Bit_RESET); } // 1- chanel off
         if(PB1_on==3) { PB1_on =0;  GPIO_WriteBit(OUT_RELE,Rele2,Bit_RESET); } // 2- chanel off   //
         if(PB2_on==3) { PB2_on =0;  GPIO_WriteBit(OUT_RELE,Rele3,Bit_RESET); } // 3- chanel off  
         if(PB3_on==3) { PB3_on =0;  GPIO_WriteBit(OUT_RELE,Rele4,Bit_RESET); } // 4- chanel off 
         if(PB5_on==3) { PB5_on =0;  GPIO_WriteBit(OUT_RELE,Rele6,Bit_RESET); } // 6- chanel off 
     
	        GPIO_WriteBit(OUT_RELE,Rele5,Bit_SET);  // 5- chanel on
          PB4_on=3;  // 5 канал на линии
        }


goto ed;			 
      }

//------------------SW 7-------------------------2-chanel
else if(result_1 >= 0x5a0  && result_1 <= 0x613  )
{
	
		 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();
				register_dat_red[6]=0; register_dat_red[7]=1;
			 	register_red(); 


			 if(PB5_on ==1)
       {
         if(PB0_on==3) { PB0_on =0;  GPIO_WriteBit(OUT_RELE,Rele1,Bit_RESET); } // 1- chanel off
         if(PB1_on==3) { PB1_on =0;  GPIO_WriteBit(OUT_RELE,Rele2,Bit_RESET); } // 2- chanel off   //
         if(PB2_on==3) { PB2_on =0;  GPIO_WriteBit(OUT_RELE,Rele3,Bit_RESET); } // 3- chanel off  
         if(PB3_on==3) { PB3_on =0;  GPIO_WriteBit(OUT_RELE,Rele4,Bit_RESET); } // 4- chanel off 
         if(PB4_on==3) { PB4_on =0;  GPIO_WriteBit(OUT_RELE,Rele5,Bit_RESET); } // 5- chanel off 
     
	        GPIO_WriteBit(OUT_RELE,Rele6,Bit_SET);  // 6- chanel on
          PB5_on=3;  // 6 канал на линии
        }

goto ed;	



}					
//---------------- SW8 reset all------------------8-chanel
else if(result_1 >= 0x950  && result_1 <= 0x99f  )			
{   


 // test_led();   
	test_circut();
	
	 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();

// SW8 сброс всех
         if(PB0_on==3) { PB0_on =0;  GPIO_WriteBit(OUT_RELE,Rele1,Bit_RESET); } // 1- chanel off
         if(PB1_on==3) { PB1_on =0;  GPIO_WriteBit(OUT_RELE,Rele2,Bit_RESET); } // 2- chanel off   //
         if(PB2_on==3) { PB2_on =0;  GPIO_WriteBit(OUT_RELE,Rele3,Bit_RESET); } // 3- chanel off  
         if(PB3_on==3) { PB3_on =0;  GPIO_WriteBit(OUT_RELE,Rele4,Bit_RESET); } // 4- chanel off 
         if(PB4_on==3) { PB4_on =0;  GPIO_WriteBit(OUT_RELE,Rele5,Bit_RESET); } // 5- chanel off 
				 if(PB5_on==3) { PB5_on =0;  GPIO_WriteBit(OUT_RELE,Rele6,Bit_RESET); } // 6- chanel off 
	goto ed;			 

}




//---------------------SW9---------------11-chanel
//if(result_1 >= 761  && result_1 <= 765  ); // 763 0.615     757=         0.610
//if(result_1 >= 0x2f9  && result_1 <= 0x2fd  );		
//---------------------SW10---------------7-chanel
//if(result_1 >=  936 && result_1 <= 940  ); //938 =0.756     937=         0.755
//if(result_1 >= 0x3c3  && result_1 <= 0x3ac  );		
//---------------------SW11---------------9-chanel
//if(result_1 >=  1215 && result_1 <= 1219  ); // 1217=0.981  1216=        0.980
//if(result_1 >= 0x4bf  && result_1 <= 0x4c3  );		
//----------------------SW12--------------reserv
//if(result_1 >=  1730 && result_1 <= 1734  ); // 1732 =                   1.396v
//if(result_1 >= 0x6c2  && result_1 <= 0x6c6  );		
//-----------------------SW13-------------12-chanel
//if(result_1 >=  675 && result_1 <= 679  ); // 677 =                      0.546v
//if(result_1 >= 0x2a3  && result_1 <= 0x2a7  );		
//-----------------------SW14-------------8--chanel
//if(result_1 >=  809 && result_1 <= 813  ); //811=0.654            812=   0.655
//if(result_1 >= 0x329  && result_1 <= 0x32d  );		
//------------------------SW15------------4-chanel
//if(result_1 >=  1009 && result_1 <= 1013  ); // 1011=                    0.815v
//if(result_1 >= 0x3f1  && result_1 <= 0x3f5  );		


//-----------------------SW16------------ test

else if(result_1 >= 0x530  && result_1 <= 0x54f  ) 
{

	test_circut();
	
	 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();



}		
else;
ed: 

//}
}

//--------------------------------------
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* PCLK1 = HCLK */
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* Select HSE as system clock source */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till HSE is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08){}
    }

  	/* разрешение тактирования  */
   /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA
    											| RCC_APB2Periph_GPIOB
    											, ENABLE);

}
//--------------- конфигураци камня-------------------------
 void GPIO_Configuration(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
 	//ADC_InitTypeDef ADC_InitStructure;

    /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(	 RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);/*RCC_APB2Periph_ADC1 |*/

  
    /* Configure  as input floating  */
    GPIO_InitStructure.GPIO_Pin =  Line1 | Line2 | Line3 | Line4 | Line5 | Line6 ;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(IN_LINE, &GPIO_InitStructure);
   

  // input of ADC 
	 //Разрешаем подачу сигнала тактовой частоты на устройства шины APB2 (PC)  
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);    
	//Настраиваем PC4 как аналоговый вход (АЦП канал 14)  
	// У нас PA0 GPIO_Pin_0 ADC_Channel_0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  
	GPIO_Init(OUT_RELE, &GPIO_InitStructure); 
	
	
 /* Configure  as Output push-pull */
    GPIO_InitStructure.GPIO_Pin =  Rele1 | Rele2 | Rele3 | Rele4 | Rele5 | Rele6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(OUT_RELE, &GPIO_InitStructure);



}
 
void ADC_setup()
 {  
ADC_InitTypeDef ADC_InitStructure;   
 //Разрешаем подачу сигнала тактовой частоты на устройства шины APB2 (АЦП1)  
	 //clock for ADC (max 14MHz --> 72/6=12MHz)
RCC_ADCCLKConfig (RCC_PCLK2_Div6);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
   
//Настройка АЦП1  
ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  
ADC_InitStructure.ADC_ScanConvMode = DISABLE;  
ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  
ADC_InitStructure.ADC_NbrOfChannel = 1;  
ADC_Init(ADC1, &ADC_InitStructure);    
//Настройка канала 0 стандартного преобразования АЦП1  
ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5); 

ADC_Cmd(ADC1, ENABLE);                    //must be called after all other ADC configuration functions.
// Инициализировал АЦП заполненной структурой
ADC_Init(ADC1, &ADC_InitStructure);

// Разрешил прерывания по окончанию преобразования
// Список возможных констант получен поиском по ADC_ITConfig 
// (найдено в stm32f10x_adc.c), из них выбрал нужную.
// EOC это End of conversion
ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
   
//Разрешение работы АЦП1  ADC_ResetCalibration(ADC1);                        
//Сброс регистров калибровки АЦП1  
while(ADC_GetResetCalibrationStatus(ADC1));        
//Ждем пока регистры сбросятся  
	 ADC_StartCalibration(ADC1);                        
//Пуск калибровки АЦП1  
	 while(ADC_GetCalibrationStatus(ADC1));             
//Ждем пока не закончится калибровка 	 
	 
//----------------------------------------------------------	 
  
NVIC_InitStructure.NVIC_IRQChannel =ADC1_2_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

NVIC_EnableIRQ (ADC1_2_IRQn); // Разрешаем прерывания от АЦП





} 




void delay_(void)
{
  unsigned int countDown = 500000;         // arbitrary int to count down

  while(countDown--);                      // count down
}

void register_red(void)
{ /*
	 GPIO_SetBits(IN_LINE,  Line1  | Line2 | Line3 | Line4 | Line5 | Line6);
   GPIO_ResetBits(OUT_RELE, Rele1  | Rele2 | Rele3 | Rele4 | Rele5 | Rele6);
	
	*/
  for (counter = 23; counter >= 0; counter--)
  { if (register_dat_red[counter] == 1)     GPIO_SetBits(OUT_RELE,DS);//GPIO_WriteBit(OUT_RELE,DS,Bit_SET);
    else                                    GPIO_ResetBits(OUT_RELE,DS);//GPIO_WriteBit(OUT_RELE,DS,Bit_RESET);

		
		
   //GPIO_WriteBit(OUT_RELE,SH_CP,Bit_SET);
		 GPIO_SetBits(OUT_RELE,SH_CP);
   //  delay_();  //delay возможно надо будет добавить
  // GPIO_WriteBit(OUT_RELE,SH_CP,Bit_RESET);
		 GPIO_ResetBits(OUT_RELE,SH_CP);
  //   delay_();  //delay
		
	
		// ловушка 
		if(counter==4)
		{
      	delay_();  //delay
		}
		
		
  }
//-----------------------------------------
 // GPIO_WriteBit(OUT_RELE,ST_CP,Bit_SET);
	 GPIO_SetBits(OUT_RELE,ST_CP);
     delay_();  //delay
  //GPIO_WriteBit(OUT_RELE,ST_CP,Bit_RESET);
	 GPIO_ResetBits(OUT_RELE,ST_CP);
 // GPIO_WriteBit(OUT_RELE,DS,Bit_RESET);
	 GPIO_ResetBits(OUT_RELE,DS);
    delay_();  //delay
}
//--------------------------------------
//---------------- запуск АЦП 13,5 циклов------------------------
void ADC_simple(void)
{
	// start conversion
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1
 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5);
 ADC_SoftwareStartConvCmd(ADC1, ENABLE);




}
int main(void)
{
		// инициализация камня  
	RCC_Configuration();
	GPIO_Configuration(); 
	ADC_setup();
	NVIC_Configuration();
 // RTC_Configuration();
  ADC_setup();

	
	  NVIC_EnableIRQ(ADC1_2_IRQn);
// инициализация линий 
     GPIO_SetBits(IN_LINE,  Line1  | Line2 | Line3 | Line4 | Line5 | Line6);
     GPIO_ResetBits(OUT_RELE, Rele1  | Rele2 | Rele3 | Rele4 | Rele5 | Rele6);
test_led();
	 test_circut();
	
	
	while(1)
	{
	//	ADC_Cmd(ADC1,ENABLE); 
	      // для поиска соответствующей кнопки в ADC                    // красный светодиод пришол вызов       // опрос клавы
           if(GPIO_ReadInputDataBit(IN_LINE, Line1) != 1)
              { if(PB0_on !=3) 
							    { PB0_on=1;register_dat_red[19]=1; 
						      	ADC_Cmd(ADC1,ENABLE); 
								  }
							}            
        //-----------------------------
        if(GPIO_ReadInputDataBit(IN_LINE, Line2) != 1) {if(PB1_on !=3) { PB1_on=1;  register_dat_red[13]=1; ADC_Cmd(ADC1,ENABLE);}}/*  Line2 */ 
        //-----------------------------
           if(GPIO_ReadInputDataBit(IN_LINE, Line3) != 1) {if(PB2_on !=3) { PB2_on=1; register_dat_red[5]=1;  ADC_Cmd(ADC1,ENABLE);}}/*  Line3 */ 
        //-----------------------------
         if(GPIO_ReadInputDataBit(IN_LINE, Line4) != 1) {if(PB3_on !=3) { PB3_on=1; register_dat_red[21]=1;  ADC_Cmd(ADC1,ENABLE);}}/*  Line4 */ 
       //-----------------------------
         if(GPIO_ReadInputDataBit(IN_LINE, Line5) != 1) {if(PB4_on !=3) { PB4_on=1; register_dat_red[15]=1; ADC_Cmd(ADC1,ENABLE);}}/*  Line5 */ 
       //-----------------------------
           if(GPIO_ReadInputDataBit(IN_LINE, Line6) != 1) {if(PB5_on !=3) { PB5_on=1; register_dat_red[7]=1; ADC_Cmd(ADC1,ENABLE);}}/*  Line6 */ 
       //-----------------------------------------------------------------------------------------------------------------------------
		       if(PB0_on==3) {  register_dat_red[18]=1; register_dat_red[19]=0;}// линия 1 подключена green -----------SW1 9
           if(PB0_on==0) {  register_dat_red[18]=0; register_dat_red[19]=0;}// линия 1 отключена
       //--------------------------------------
           if(PB1_on==3) {  register_dat_red[12]=1; register_dat_red[13]=0;}// линия 2 подключена-----------SW2 5
           if(PB1_on==0) {  register_dat_red[12]=0; register_dat_red[13]=0;}// линия 2 отключена
      //--------------------------------------
           if(PB2_on==3) {  register_dat_red[4]=1; register_dat_red[5]=0;}// линия 3 подключена -----------SW3  1
					 if(PB2_on==0) {  register_dat_red[4]=0; register_dat_red[5]=0;}// линия 3 отключена 
     //--------------------------------------
           if(PB3_on==3) {  register_dat_red[20]=1; register_dat_red[21]=0;}// линия 4 подключена-----------SW5 4
           if(PB3_on==0) {  register_dat_red[20]=0; register_dat_red[21]=0;}// линия 4 отключена 
     //--------------------------------------
          if(PB4_on==3)  {  register_dat_red[14]=1; register_dat_red[15]=0;}//линия 5 подключена-----------SW6 6
          if(PB4_on==0)  {  register_dat_red[14]=0; register_dat_red[15]=0;}//линия 5 отключена 
     //--------------------------------------
          if(PB5_on==3)   { register_dat_red[6]=1; register_dat_red[7]=0;}//линия 6 подключена-----------SW7 2
          if(PB5_on==0)   { register_dat_red[6]=0; register_dat_red[7]=0;}//линия 6 отключена 
//-------------------------------------------
		


					if(PB0_on != 0 || PB1_on!= 0 || PB2_on!= 0  || PB3_on!= 0  || PB4_on!= 0  || PB5_on!= 0 )
              ADC_Cmd(ADC1,ENABLE);    // опрос клавы	??? SW8  кнопка отбой 
	}	

 
}

//-------------------------------------
void test_led(void)
{

				  register_dat_red[19]=1;// r sw1
				  register_dat_red[13]=1;// r sw2
				  register_dat_red[5]=1; // r sw3
			//non					
				  register_dat_red[21]=1;//r sw5
				  register_dat_red[15]=1; // r sw6		
				  register_dat_red[7]=1;// r sw7
				// reset
				  register_dat_red[23]=1;//r sw9
				  register_dat_red[9]=1;// r sw10
				register_dat_red[1]=1;// r sw11
				//non	
				  register_dat_red[16]=1;// r sw13
				  register_dat_red[11]=1;// r sw14
			    register_dat_red[3]=1;// r sw15
				// test
				
					 
					register_red();
					
					 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
					 
					delay_();
					
					 register_dat_red[18]=1;// g sw1
					 register_dat_red[12]=1;//2
					 register_dat_red[4]=1; //3
					 register_dat_red[20]=1;//g sw5
					 register_dat_red[14]=1;//6
					 register_dat_red[6]=1;//7
				//-----------------------------------
	        register_dat_red[22]=1;//g sw9
          register_dat_red[8]=1;// g sw10
			    register_dat_red[0]=1;// g sw11
					 register_dat_red[17]=1;// g sw13
					 register_dat_red[10]=1;// g sw14
					 register_dat_red[2]=1;
				 	register_red();
						delay_();
						
							 for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
							 	register_red();
delay_();
}
void test_circut(void)
{

for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0;   
	  register_dat_red[19]=1;// r sw1
		register_red();
	  delay_();
	//------------
	 register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=1;// r sw1
		register_red();
	  delay_();
	//------------
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw1
  register_dat_red[5]=1;// r sw1
		register_red();
	  delay_();
	//------------
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw1
  register_dat_red[5]=0;// r sw1
 register_dat_red[21]=1;// r sw1
		register_red();
	  delay_();
	//------------
		register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw1
  register_dat_red[5]=0;// r sw1
 register_dat_red[21]=0;// r sw1
 register_dat_red[15]=1;// r sw6
		register_red();
	  delay_();
	//------------
register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw1
  register_dat_red[5]=0;// r sw1
 register_dat_red[21]=0;// r sw1
 register_dat_red[15]=0;// r sw1	
 register_dat_red[7]=1;// r sw6
		register_red();
	  delay_();
	//------------
	
	
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw2
  register_dat_red[5]=0;// r sw3
 register_dat_red[21]=0;// r sw5
 register_dat_red[15]=0;// r sw6	
 register_dat_red[7]=1;// r sw7
		register_red();
	  delay_();
	//------------
	
	
	
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw2
  register_dat_red[5]=0;// r sw3
 register_dat_red[21]=0;// r sw5
 register_dat_red[15]=0;// r sw6	
 register_dat_red[7]=0;// r sw7
register_dat_red[23]=1;// r sw9
		register_red();
	  delay_();
	//------------	
	
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw2
  register_dat_red[5]=0;// r sw3
 register_dat_red[21]=0;// r sw5
 register_dat_red[15]=0;// r sw6	
 register_dat_red[7]=0;// r sw7
register_dat_red[23]=0;// r sw9
register_dat_red[9]=1;// r sw10

		register_red();
	  delay_();
	//------------		
register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw2
  register_dat_red[5]=0;// r sw3
 register_dat_red[21]=0;// r sw5
 register_dat_red[15]=0;// r sw6	
 register_dat_red[7]=0;// r sw7
register_dat_red[23]=0;// r sw9
register_dat_red[9]=0;// r sw10
register_dat_red[1]=1;// r sw11

		register_red();
	  delay_();
	//------------			
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw2
  register_dat_red[5]=0;// r sw3
 register_dat_red[21]=0;// r sw5
 register_dat_red[15]=0;// r sw6	
 register_dat_red[7]=0;// r sw7
register_dat_red[23]=0;// r sw9
register_dat_red[9]=0;// r sw10
register_dat_red[1]=0;// r sw11
register_dat_red[16]=1;// r sw13

		register_red();
	  delay_();
	//------------			
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw2
  register_dat_red[5]=0;// r sw3
 register_dat_red[21]=0;// r sw5
 register_dat_red[15]=0;// r sw6	
 register_dat_red[7]=0;// r sw7
register_dat_red[23]=0;// r sw9
register_dat_red[9]=0;// r sw10
register_dat_red[1]=0;// r sw11
register_dat_red[16]=0;// r sw13
register_dat_red[11]=1;// r sw14
		register_red();
	  delay_();
	//------------			
	register_dat_red[19]=0;// r sw1
	  register_dat_red[13]=0;// r sw2
  register_dat_red[5]=0;// r sw3
 register_dat_red[21]=0;// r sw5
 register_dat_red[15]=0;// r sw6	
 register_dat_red[7]=0;// r sw7
register_dat_red[23]=0;// r sw9
register_dat_red[9]=0;// r sw10
register_dat_red[1]=0;// r sw11
register_dat_red[16]=0;// r sw13
register_dat_red[11]=0;// r sw14
register_dat_red[3]=1;// r sw15
		register_red();
	  delay_();
	//------------				
	
	
	
	
	

for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0; 
	register_red();
	
	
  register_dat_red[18]=1;// r sw1
		register_red();
	  delay_();
	//------------
	 register_dat_red[18]=0;// r sw1
	  register_dat_red[12]=1;// r sw1
		register_red();
	  delay_();
	//------------
	register_dat_red[18]=0;// r sw1
	  register_dat_red[12]=0;// r sw1
  register_dat_red[4]=1;// r sw1
		register_red();
	  delay_();
	//------------
	register_dat_red[18]=0;// r sw1
	  register_dat_red[12]=0;// r sw1
  register_dat_red[4]=0;// r sw1
 register_dat_red[20]=1;// r sw1
		register_red();
	  delay_();
	//------------
		register_dat_red[18]=0;// r sw1
	  register_dat_red[12]=0;// r sw1
  register_dat_red[4]=0;// r sw1
 register_dat_red[20]=0;// r sw1
 register_dat_red[14]=1;// r sw1
		register_red();
	  delay_();
	//------------
register_dat_red[18]=0;// r sw1
	  register_dat_red[12]=0;// r sw1
  register_dat_red[4]=0;// r sw1
 register_dat_red[20]=0;// r sw1
 register_dat_red[14]=0;// r sw1	
 register_dat_red[6]=1;// r sw1
		register_red();
	  delay_();
	//------------

for (counter = 23; counter >= 0; counter--) register_dat_red[counter] =0; 
	register_red();	
	
	

}















