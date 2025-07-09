/*
 * spi.c
 *
 *  Created on: Mar 1, 2023
 *      Author: hussamaldean
 */


#include "spi2_new.h"
//#include "stm32f1xx.h"
#include "stm32f10x.h"                  // Device header

uint8_t SPI_Write(uint8_t data){
// uint8_t b;

	 while (!((SPI2->SR)&(1<<1))) {};
		   /*Write the data to the Data Register*/
			 *(volatile uint8_t *)&SPI2->DR =data;
		 
			return  SPI2->DR;
	}
	
	
void SPI_Send(uint8_t byte){

  SPI_Write(byte);

}	


	
uint8_t SPI_Receive(void){
//	SPI_Write(0xFF);
	SPI_Write(0x00);
	while ((SPI2->SR & SPI_SR_RXNE)!= SPI_SR_RXNE);
	
return	SPI2->DR;
}
//*************************************
void spi1_init(void)
{
	//Enable Port A clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	/* PA5-SCK and PA7-MOSI */
	//Mode: Output, Speed: 10MHz
	GPIOA->CRL &= ~(GPIO_CRL_MODE5);
	GPIOA->CRL &= ~(GPIO_CRL_MODE7);
	GPIOA->CRL |= (GPIO_CRL_MODE5_0);
	GPIOA->CRL |= (GPIO_CRL_MODE7_0);
	//Alternate function push-pull
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);
	GPIOA->CRL &= ~(GPIO_CRL_CNF7);
	GPIOA->CRL |= (GPIO_CRL_CNF5_1);
	GPIOA->CRL |= (GPIO_CRL_CNF7_1);

	/* PA5-MISO */
	//Mode input
	GPIOA->CRL &= ~(GPIO_CRL_MODE6);
	//Floating input
	GPIOA->CRL &= ~(GPIO_CRL_CNF6);
	GPIOA->CRL |= (GPIO_CRL_CNF6_0);

	//Remap 0: PA5, PA6, PA7
	AFIO->MAPR &= ~(1UL << 0);
	
	/*Configure PA0 as output CS*/
	GPIOA->CRL|=GPIO_CRL_MODE0;
	GPIOA->CRL&=~(GPIO_CRL_CNF0);
	

	//Enable SPI Clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	//Mode: Master
	SPI1->CR1 |= SPI_CR1_MSTR;
	//Baud rate to (8MHz / 2 = 4MHz)
	SPI1->CR1 &= ~SPI_CR1_BR;
	//MSB first
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
	//Full duplex (Transmit/Receive)
	SPI1->CR1 &= ~(SPI_CR1_RXONLY);
	//Data format 8-bit
	SPI1->CR1 &= ~(SPI_CR1_DFF);
	//Software Slave select
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_SSM;
	//SPI Enable
	SPI1->CR1 |= SPI_CR1_SPE;
	//Clear initial flags
	(void)SPI1->SR;


}

void spi2_init(void)
{ /*// Будем обрабатывать прерывание от SPI2, поэтому конфигурируем NVIC.
    // Задаём приоритет (используемая идиома задаёт низший приоритет).
    NVIC_SetPriority(SPI2_IRQn, (1<<__NVIC_PRIO_BITS)-1);
    // Разрешаем обработку этого прерывания.
    NVIC_EnableIRQ(SPI2_IRQn);*/
	//Enable Port   clock
	RCC->APB1ENR |= RCC_APB2ENR_IOPBEN;//B
	RCC->APB1ENR |= RCC_APB2ENR_AFIOEN;
	/* PA5-SCK and PA7-MOSI */
	//Mode: Output, Speed: 10MHz
	GPIOB->CRH &= ~(GPIO_CRH_MODE13);//SCK
	GPIOB->CRH &= ~(GPIO_CRH_MODE15);//MOSI
	GPIOB->CRH |= (GPIO_CRH_MODE13_0);
	GPIOB->CRH |= (GPIO_CRH_MODE15_0);
	//Alternate function push-pull
	GPIOB->CRH &= ~(GPIO_CRH_CNF13);
	GPIOB->CRH &= ~(GPIO_CRH_CNF15);
	GPIOB->CRH |= (GPIO_CRH_CNF13_1);
	GPIOB->CRH |= (GPIO_CRH_CNF15_1);

	/* PA5-MISO */
	//Mode input
	GPIOB->CRH &= ~(GPIO_CRH_MODE14);
	//Floating input
	GPIOB->CRH &= ~(GPIO_CRH_CNF14);
	GPIOB->CRH |= (GPIO_CRH_CNF14_0);

	//Remap 0: PA5, PA6, PA7
	AFIO->MAPR &= ~(1UL << 0);
	
	/*Configure PA0 as output CS*/
	GPIOB->CRH|=GPIO_CRH_MODE12;
	GPIOB->CRH&=~(GPIO_CRH_CNF12);
	

	//Enable SPI Clock
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	//Mode: Master
	SPI2->CR1 |= SPI_CR1_MSTR;
	//Baud rate to (8MHz / 2 = 4MHz)
	//SPI2->CR1 &= ~SPI_CR1_BR;
	
	//MSB first
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);
	//Full duplex (Transmit/Receive)
	SPI2->CR1 &= ~(SPI_CR1_RXONLY);
	//Data format 8-bit
//	SPI2->CR1 &= ~(SPI_CR1_DFF);
	
//	SPI2->CR1 |= SPI_CR1_BR_2 ; //    Fpclk=72Mhz/32 = 2.25 Mhz
//	SPI2->CR1 &=~ SPI_CR1_BR_1|SPI_CR1_BR_0 ;

 SPI2->CR1 |= SPI_CR1_BR_2 ; //    Fpclk=72Mhz/32 = 2.25 Mhz
	SPI2->CR1 &=~ SPI_CR1_BR_1;//|SPI_CR1_BR_0 ;
// Clock Phase
	SPI2->CR1 &=~ SPI_CR1_CPOL; // Set clock Plority
	SPI2->CR1 |=  SPI_CR1_CPHA;  // set Clock Phase
	
	//Software Slave select  mode1
	SPI2->CR1 |= SPI_CR1_SSI;
	SPI2->CR1 |= SPI_CR1_SSM;
	//SPI Enable
	SPI2->CR1 |= SPI_CR1_SPE;
	//Clear initial flags
	(void)SPI2->SR;


}


void spi1_transmit(uint8_t *data,uint32_t size)
{
	uint32_t i=0;

	while(i<size)
	{
		 /*Wait for the TXE bit to set in the Status Register*/
		/*This will indicate that the transmit buffer is empty*/
		 while (!((SPI1->SR)&(1<<1))) {};
		   /*Write the data to the Data Register*/
		   SPI1->DR = data[i];
		   i++;
	}
	 /*Wait for the TXE bit to set in the Status Register*/
	 while (!((SPI1->SR)&(1<<1))) {};

	 /*Wait for the BSY bit to reset in the Status Register*/
	 while (((SPI1->SR)&(1<<7))) {};

	 /*Clear OVR flag*/
	(void) SPI1->DR;
	(void) SPI1->SR;
}

void spi2_transmit(uint8_t *data,uint8_t size)
{
	uint32_t i=0;

	while(i<size)
	{
		 /*Wait for the TXE bit to set in the Status Register*/
		/*This will indicate that the transmit buffer is empty*/
		 while (!((SPI2->SR)&(1<<1))) {};
		   /*Write the data to the Data Register*/
			 *(volatile uint8_t *)&SPI2->DR =data[i];
		   i++;
	}
	 /*Wait for the TXE bit to set in the Status Register*/
	 while (!((SPI2->SR)&(1<<1))) {};

	 /*Wait for the BSY bit to reset in the Status Register*/
	 while (((SPI2->SR)&(1<<7))) {};

	 /*Clear OVR flag*/
	(void) SPI2->DR;
	(void) SPI2->SR;
}

void spi1_cs_low(){	GPIOA->BSRR=GPIO_BSRR_BR0;}
void spi1_cs_high(){	GPIOA->BSRR=GPIO_BSRR_BS0;}

//***************** PB11 *******************
void spi2_csPB11_high(){	GPIOB->BSRR=GPIO_BSRR_BS11;}
void spi2_csPB11_low(){	GPIOB->BSRR=GPIO_BSRR_BR11;}
//***************** PB12 *******************
void spi2_cs_high(){	GPIOB->BSRR=GPIO_BSRR_BS12;}
void spi2_cs_low(){	GPIOB->BSRR=GPIO_BSRR_BR12;}
//*************** PB2 **************
void spi2_csPB2_high(){	GPIOB->BSRR=GPIO_BSRR_BS2;}
void spi2_csPB2_low(){	GPIOB->BSRR=GPIO_BSRR_BR2;}
//************** PB9 ***************
void spi2_csPB9_high(){		GPIOB->BSRR=GPIO_BSRR_BS9;}
void spi2_csPB9_low(){	GPIOB->BSRR=GPIO_BSRR_BR9;}

uint8_t spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
uint32_t i=0;
//spi2_cs_low();
	while(i<len)
	{ //SPI_Send(0xff);
		 /*Wait for the TXE bit to set in the Status Register*/
		/*This will indicate that the transmit buffer is empty*/
		 while (!((SPI2->SR)&(1<<1))) {};
		   /*Write the data to the Data Register*/
			// *(volatile uint8_t *)&SPI2->DR =data[i];
		  // i++;
				 buf[i]=SPI_Receive(); //*(volatile uint8_t *)&SPI2->DR;
			 i++;
	}
	 /*Wait for the TXE bit to set in the Status Register*/
	 while (!((SPI2->SR)&(1<<1))) {};

	 /*Wait for the BSY bit to reset in the Status Register*/
	 while (((SPI2->SR)&(1<<7))) {};

	 /*Clear OVR flag*/
	(void) SPI2->DR;
	(void) SPI2->SR;

//spi2_cs_high();

}

uint8_t spi_max_read(uint8_t reg, uint8_t buf[], uint16_t len)
{
 
volatile uint8_t iii=0;
 while (!((SPI2->SR)&(1<<1))) {};
	  SPI_Send(reg);
	while(len--)
	{//SPI_Send(0x00);
		 /*Wait for the TXE bit to set in the Status Register*/
		/*This will indicate that the transmit buffer is empty*/
		 while (!((SPI2->SR)&(1<<1))) {};
		   /*Write the data to the Data Register*/
			// *(volatile uint8_t *)&SPI2->DR =data[i];
		  // i++;
				 buf[iii]=SPI_Receive(); //*(volatile uint8_t *)&SPI2->DR;
			 if(len==0) break;
			 iii++;
	}
	 /*Wait for the TXE bit to set in the Status Register*/
	 while (!((SPI2->SR)&(1<<1))) {};

	 /*Wait for the BSY bit to reset in the Status Register*/
	 while (((SPI2->SR)&(1<<7))) {};

	 /*Clear OVR flag*/
	(void) SPI2->DR;
	(void) SPI2->SR;

 iii=0;

};




uint8_t spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
 
	 
	for(int i=0;i<=len;i++)
	{
	 while (!((SPI2->SR)&(1<<1))) {};
		   /*Write the data to the Data Register*/
			 *(volatile uint8_t *)&SPI2->DR =buf[i];
	 } 
			return  SPI2->DR;



}

uint8_t spi_max_write(uint8_t reg, uint8_t buf[], uint16_t len)
{
 volatile uint8_t iii=0;
//int i=0;
	
	while (!((SPI2->SR)&(1<<1))) {};
	 *(volatile uint8_t *)&SPI2->DR =reg;
	
	//for(int i=0;i<=len;i++)
	while(len--)
	{
	 while (!((SPI2->SR)&(1<<1))) {};
		   /*Write the data to the Data Register*/
			 *(volatile uint8_t *)&SPI2->DR =buf[iii];
		 iii++;
	 } 
 iii=0;
			return  SPI2->DR;



}

