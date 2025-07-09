#include "stm32f10x.h"                  // Device header

#include "encoderTIM2_PA0_PA1.h"

void Encoder_Init (void)  //------------������������� ��������-----------
{   
	RCC->AHBENR     |= RCC_APB2ENR_IOPBEN;
 // RCC->AHBENR     |= RCC_AHBENR_GPIOAEN;                                         // Clock On - GPIOA
 //  GPIOA->MODER       |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;       //Pin 0,1 - Alternative Mode
 /*Configure PA0 as Output Alternate Push/Pull */
	GPIOA->CRL&=~GPIO_CRL_MODE0;
	GPIOA->CRL|=(GPIO_CRL_CNF0_0);
	GPIOA->CRL&=~(GPIO_CRL_CNF0_1);

	/*Configure PA1 as Output Alternate Push/Pull*/

	GPIOA->CRL&=~GPIO_CRL_MODE1;
	GPIOA->CRL|=(GPIO_CRL_CNF1_0);
	GPIOA->CRL&=~(GPIO_CRL_CNF1_1);


 // GPIOA->OTYPER    &=~(GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1);             //Push_Pull mode
//   GPIOA->OSPEEDR    |= GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1; //Pin0,1 - Maximum speed 50MHz
//   GPIOA->PUPDR       |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;         //Pin0,1 - PullUp mode
 //  GPIOA->AFR[0]    |= 0x01 << (0 * 4);                                              //GPIO_AFRL_AFR0  - AF1
 //  GPIOA->AFR[0]    |= 0x01 << (1 * 4);                                              //GPIO_AFRL_AFR1  - AF1
   
//-------------����������� ������--------
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                                                // Clock On - TIM2
   TIM2->ARR = 100;                                                                    //Period = 4 (0,1,2,3);
   TIM2->PSC = 0;                                                                    //TIM_Prescaler=0;
   TIM2->CR1   &=~(TIM_CR1_ARPE);                                                         //Preload - OFF
   TIM2->CR1 &=  TIM_CR1_CKD;                                                           //ClockDivision = TIM_CKD_DIV1;
//------------������� � ����� ������� ---------------
   TIM2->SMCR       |= TIM_SMCR_SMS_0   | TIM_SMCR_SMS_1;     //����������� ����� �����\����
   TIM2->CCMR1    	|= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;   //����� ������� � ����� ������� "0"
   TIM2->CCER 			&=~(TIM_CCER_CC1P   | TIM_CCER_CC2P);     //���������� �������  - �� ����� "0"
   TIM2->CCER 			&=~(TIM_CCER_CC1NP  | TIM_CCER_CC2NP);    //���������� ������� ��� ��������� "0"
   TIM2->CNT=0;                                               // ����� ��������
   TIM2->DIER    		|= TIM_DIER_UIE;                          // ��������� ���������� �� �������
   TIM2->CR1    		|= TIM_CR1_CEN;                           // ������ �������   
   
NVIC_EnableIRQ(TIM2_IRQn);                                    // ���������� �� TIM3 - ���������
NVIC_SetPriority(TIM2_IRQn,0x00);                             // ��������� ���������� - 1

  //  Counter=5;                                             // ���������� �������� ��������.
   }
int sign =0;
void TIM2_IRQHandler(void)
{
    TIM2->DIER &= ~(TIM_DIER_UIE); // Interrupt disable
   if( TIM2->SR & TIM_SR_UIF )
      { //TIM2->SR &= ~(TIM_SR_UIF);
         // if ((TIM2->CR1 & TIM_CR1_DIR))  
					 if((TIM2->CR1 & TIM_CR1_DIR) > 0)
					     Counter=1;
          else {
								//if (Counter) 
								//		Counter=-1;
					  	TIM2->CNT--;
					    } 
			TIM2->SR &= ~(TIM_SR_UIF);		
    }


   TIM2->DIER |= TIM_DIER_UIE; //Interrupt enable   
 		
	
		
}
//********************************
int encoder_set(int cnt)
{//encoder_read(cnt)
	if(Counter==1)
      return 1;
	if(Counter==-1)
	return -1;
	else
return 0;

}



uint32_t encoder_read(uint32_t offset )
{
	offset = offset + sign;
	sign =0;
// displey(90, 50,offset ,7)	;
//	SSD1306_UpdateScreen();		

//return  offset=offset +Counter;
	return offset;// TIM2->CNT;
}

//**************************************************

#define QTY_MENU       10    //__���������� ����� ���� ( �������������� ������ �� 0 �� 9).
#define QTY_MENU       10    //__���������� ����� ���� ( �������������� ������ �� 0 �� 9).
#define START_MENU        3    //__��������� ������ ����.
#define EN�_MAX_VAL    255   //__������������ �������� �������� ��������.
#define QTY_TICK         2    //__����� ����� �� 1 ������� ��������.

uint16_t ENC_MenuNum = START_MENU;

/////////////////////__������� ������ ��������__////////////////////////
//   �������� � ����� �� ���� ��� ����� ������ 25�� ����� �����
//   ���������� �������� ��� ������� �������� �� ����� ���������,
//   ��� ������� �� ���� ��� � ��� ����� ��� �����������...
//   ��� ����� ����� �� "������", ��� �� ������� ����� ������� ��
//   ��������, �� ��� ���� ��� �� ��� ������� ��������.
////////////////////////////////////////////////////////////////////////
void ENC_Read(void)
{
   if(TIM2->CNT == 0)
      return;
   //__����������� ����������� �������� (����� � ���� ������ �� �����!).
   //   !!! ���� �� � ��� ����������� ������ ��������� ���������� ��������
   //         ">" �� "==" � ������ ���� ^_^.
   if((TIM2->CR1 & TIM_CR1_DIR) > 0)   //__�����.
      ENC_MenuNum -= (255 - TIM2->CNT) / QTY_TICK;
   else      //__������.
      ENC_MenuNum += TIM2->CNT / QTY_TICK;
   //__����������� ����.
   if(ENC_MenuNum >= QTY_MENU)
      ENC_MenuNum = (QTY_MENU - 1);
   if(ENC_MenuNum < 0)
      ENC_MenuNum = 0;
   //__��������� ��������, ��� �� ��� ��������� ������
   //   ����� ���� ���������� �� ������� ���������� ��������.
   TIM1->CNT = 0;
}

//***********************************************************
/*void TIM2_IRQHandler(void)
{
delay_ms(100);
if (TIM_GetITStatus(TIM2, TIM_IT_Update) != 0)
{
TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 

if (TIM2->CR1 & TIM_CR1_DIR) //������� ���� �������� ������� ����� �������, ����� ���� � ����������
TIM_SetCounter(TIM2, 0);
else
TIM_SetCounter(TIM2, 200);
}
}*/
//***********************************************
// uint8_t EC12_1SCAN (void);

#define  EC_A    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define  EC_B    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)
/*void main(void)
{
   System_init (); // System Initialization
	while (1)
	{
		 if (Rotate == 0) // Release encoder
	               Rotate = EC12_1SCAN (); // Gets the positive reversal of the encoder 1
       
    }
}
 */
 uint8_t EC12_1SCAN (void) // EC12 Scan function, put in the main function while cycle
{
	int ret=0;
	// ______2_|--3---|_4_____
	//    ___2____3_|---4--|__
	//ret         1     2
 	while (EC_A != 1 || EC_B != 1){;	}
	while (EC_A == 0 && EC_B == 0){;	}
	
		while (EC_A != 1 || EC_B != 1){;	}
	while (EC_A == 0 && EC_B == 0){;	}
	

	
	
			do
		//	while(1)
			{//Delay_us(1);
				if (EC_A == 1 && EC_B == 0) //3
				{ret=1; 
				break;}//return 1; //++
				if (EC_A == 0 && EC_B == 1)//4
				{ret=2;
				break; }//	return 2; //-- 
				//ret=0; break;
			} while (EC_A == 1 && EC_B == 1);
	

	return ret;
}
 void EC12_1Handle (void) // This function can be put into interrupt processing
{
 
		 if (Rotate == 1) // meets clockwise
		{
         Counter--;           // Add your code here 
		}
    if (Rotate == 2) // meets counterclockwise
		{
        Counter++;             // Add your code here 
		}
		 Rotate = 0; // Release the encoder! ! Be sure to release the encoder!
}



		