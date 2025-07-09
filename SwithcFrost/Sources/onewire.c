/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */
#include <stm32f10x.h>
#include "stm32f10x_dma.h"
#include "onewire.h"

#ifdef OW_USART1

#undef OW_USART2
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART1
#define OW_DMA_CH_RX 	DMA1_Channel5
#define OW_DMA_CH_TX 	DMA1_Channel4
#define OW_DMA_FLAG		DMA1_FLAG_TC5

#endif


#ifdef OW_USART2

#undef OW_USART1
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART2
#define OW_DMA_CH_RX 	DMA1_Channel6
#define OW_DMA_CH_TX 	DMA1_Channel7
#define OW_DMA_FLAG		DMA1_FLAG_TC6

#endif


// Буфер для приема/передачи по 1-wire
uint8_t ow_buf[16];
uint8_t tbuf[16];
u16 received_data;

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff
#define OW_R    0xff

DMA_InitTypeDef     DMA_InitStructure;
USART_InitTypeDef   USART_InitStructure;
NVIC_InitTypeDef    NVIC_InitStruct;



const uint8_t convert_T[] = {
                OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
                OW_0, OW_0, OW_1, OW_0, OW_0, OW_0, OW_1, OW_0  // 0x44 CONVERT
};
const uint8_t read_scratch[] = {
                OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
                OW_0, OW_1, OW_1, OW_1, OW_1, OW_1, OW_0, OW_1, // 0xbe READ SCRATCH
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
                OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R
};
uint8_t scratch[sizeof(read_scratch)];
void OW_Init22(USART_TypeDef* USARTx) ;
uint8_t OW_Init(USART_TypeDef* USARTx);

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

//-----------------------------------------------------------------------------
// инициализирует USART и DMA
//-----------------------------------------------------------------------------
uint8_t OW_Init(USART_TypeDef* USARTx) {
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;

    NVIC_InitTypeDef NVIC_InitSrt;

	if (OW_USART == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
				ENABLE);

		// USART TX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		// USART RX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	}

	if (OW_USART == USART2) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
				ENABLE);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD;//
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOA, &GPIO_InitStruct);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	}

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(OW_USART, &USART_InitStructure);
	USART_Cmd(OW_USART, ENABLE);
    USART_HalfDuplexCmd(OW_USART,ENABLE);
    USART_DMACmd(OW_USART,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);//DMA ENABLE
    //------------------------------------------------

		// DMA на чтение
		DMA_DeInit(OW_DMA_CH_RX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;// от перефирии к памяти
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// читать по байту
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// однократное срабатывания прерывания 
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;// режим передачи от памяти к памяти откл.
		DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);
        USART_DMACmd(OW_USART, USART_DMAReq_Rx, ENABLE);// разрешить работу USART в DMA
        DMA_ITConfig(OW_DMA_CH_RX,DMA_IT_TC,ENABLE);    // разрешить прерывания от dma по окончанию приёма DMA_IT_TC=по окончанию передачи
        OW_DMA_CH_RX->CNDTR=(uint32_t)0x00;
	
	// DMA на запись
		DMA_DeInit(OW_DMA_CH_TX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;// от памяти к перефирии
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);
        USART_DMACmd(OW_USART, USART_DMAReq_Tx, ENABLE);
        DMA_ITConfig(OW_DMA_CH_TX,DMA_IT_TC,ENABLE);
        
        
        

		// старт цикла отправки
		//USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
		//USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(OW_DMA_CH_RX, ENABLE);
		DMA_Cmd(OW_DMA_CH_TX, ENABLE);
    //------------------------------------------------
    NVIC_InitSrt.NVIC_IRQChannel=DMA1_Channel6_IRQn; //прерывание от 
    NVIC_InitSrt.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitSrt.NVIC_IRQChannelSubPriority=0;
    NVIC_InitSrt.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitSrt);
    
   // NVIC_EnanableIRQ(DMA1_Channel7_IRQn);
    
    
    
	return OW_OK;
}

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
uint8_t OW_Reset(USART_TypeDef* USARTx) {
//uint8_t i;
	uint8_t ow_presence;
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits =            USART_StopBits_1;
	USART_InitStructure.USART_Parity =              USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USARTx, &USART_InitStructure);

	// отправляем 0xf0 на скорости 9600
	USART_ClearFlag(USARTx, USART_FLAG_TC);
      
	USART_SendData(USARTx, 0x00f0);//
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET) {;
//#ifdef OW_GIVE_TICK_RTOS
//		taskYIELD();
//#endif
	}

	ow_presence = USART_ReceiveData(USARTx);

	USART_InitStructure.USART_BaudRate             = 115200;//9600;
	USART_InitStructure.USART_WordLength =           USART_WordLength_8b;
	USART_InitStructure.USART_StopBits =             USART_StopBits_1;
	USART_InitStructure.USART_Parity =               USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =  USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USARTx, &USART_InitStructure);
    
    

//  ow_presence = USART_ReceiveData(OW_USART);

	if (ow_presence != 0xf0) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}
//---------------PA2---------------------
void OW_out_set_Power_pin(void)
{// PA2 TX for PowerUp
GPIO_InitTypeDef GPIO_In;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

GPIO_SetBits(GPIOA,GPIO_Pin_2);

GPIO_In.GPIO_Pin   =   GPIO_Pin_2;
GPIO_In.GPIO_Mode  =   GPIO_Mode_Out_PP;
GPIO_In.GPIO_Speed =   GPIO_Speed_50MHz;
GPIO_Init(GPIOA,&GPIO_In);
}

void OW_out_Rset_Power_pin(void)
{// PA2 TX for PowerUp
GPIO_InitTypeDef GPIO_In;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

GPIO_ResetBits(GPIOA,GPIO_Pin_2);

GPIO_In.GPIO_Pin   =   GPIO_Pin_2;
GPIO_In.GPIO_Mode  =   GPIO_Mode_Out_OD;
GPIO_In.GPIO_Speed =   GPIO_Speed_50MHz;
GPIO_Init(GPIOA,&GPIO_In);
}

void OW_out_set_TX_pin(void)
{// PA2 TX for USART2
GPIO_InitTypeDef GPIO_In;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
GPIO_In.GPIO_Pin   =   GPIO_Pin_2 ;
GPIO_In.GPIO_Mode  =   GPIO_Mode_AF_OD;
GPIO_In.GPIO_Speed =   GPIO_Speed_50MHz;
GPIO_Init(GPIOA,&GPIO_In);

 
}





//-----------------------------------------------------------------------------
// процедура общения с шиной 1-wire
// sendReset - посылать RESET в начале общения.
// 		OW_SEND_RESET или OW_NO_RESET
// command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOTH
// cLen - длина буфера команд, столько байт отошлется в шину
// data - если требуется чтение, то ссылка на буфер для чтения
// dLen - длина буфера для чтения. Прочитается не более этой длины
// readStart - с какого символа передачи начинать чтение (нумеруются с 0)
//		можно указать OW_NO_READ, тогда можно не задавать data и dLen
//-----------------------------------------------------------------------------

uint8_t OW_Send(USART_TypeDef* USARTx,uint8_t sendReset, uint8_t *command, uint8_t cLen,uint8_t *data, uint8_t dLen, uint8_t readStart)
{
//uint8_t i;
	// если требуется сброс - сбрасываем и проверяем на наличие устройств
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset(USARTx) == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}
 //
  
	while (cLen > 0) {

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;
        
		DMA_InitTypeDef DMA_InitStructure;

		// DMA на чтение
		DMA_DeInit(OW_DMA_CH_RX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);

		// DMA на запись
		DMA_DeInit(OW_DMA_CH_TX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);

		// старт цикла отправки
		
		USART_DMACmd(USARTx, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(OW_DMA_CH_RX, ENABLE);
		DMA_Cmd(OW_DMA_CH_TX, ENABLE);

		// Ждем, пока не примем 8 байт
		while (DMA_GetFlagStatus(OW_DMA_FLAG) == RESET)
        {;}

		// отключаем DMA
    USART_ClearFlag(USARTx, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
	DMA_Cmd(OW_DMA_CH_TX, DISABLE);
	DMA_Cmd(OW_DMA_CH_RX, DISABLE);
	USART_DMACmd(USARTx, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);
    
		// если прочитанные данные кому-то нужны - выкинем их в буфер
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		   } 
      else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}
//-----------DMA--------------------------------
uint16_t ReadData(uint8_t l)
{
uint8_t i;
uint16_t received_d;
//char rx_buf[32];

while(DMA1_Channel6->CNDTR);//RX

DMA1_Channel6->CCR&=~DMA_CCR6_EN; // запрет
DMA1_Channel6->CNDTR=l;           //размер буфера 
DMA1_Channel6->CCR|=DMA_CCR6_EN;  //rx start

while(DMA1_Channel7->CNDTR);
for(i=0;i<l;i++)ow_buf[i]=0xff;

DMA1_Channel7->CCR&=~DMA_CCR7_EN;//TX запрет
DMA1_Channel7->CNDTR=l;
DMA1_Channel7->CCR|=DMA_CCR7_EN;//tx start будет работать и выдаст прерывания

 while(!(DMA1->ISR&DMA_ISR_TCIF6)); // RX ???
 DMA1->IFCR|=DMA_IFCR_CTCIF6;       // очистить флаг прерывания приёма 
 DMA1_Channel6->CCR&=~DMA_CCR6_EN;
 
 received_d=0x0000;
 for(i=0;i<16;i++)
 {
   if(ow_buf[i]==0xff)
   {
   received_d|=(1<<i);
   }
 }
 ;
return received_d;
}




//----------------------PA2 1Wire----------------------
void InitAll(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
//------ Buses configuring — 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //GPIOA clock RCC_APB2Periph_USART1| //USART1 clock RCC_APB2Periph_AFIO,ENABLE); //AFIO enable 
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //USART2 clock 
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); //DMA1 clock
//------ Port configuring — 
GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_Init(GPIOA,&GPIO_InitStructure); //USART2_TX-OW interface

//------ Interrupt configuring — 
NVIC_InitStruct.NVIC_IRQChannel=DMA1_Channel7_IRQn;
NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
NVIC_Init(&NVIC_InitStruct); //USART2_TX interrupt

//------ USART2 (1-wire port) configuring — 
USART_InitStructure.USART_BaudRate=115200;
USART_InitStructure.USART_WordLength=USART_WordLength_8b;
USART_InitStructure.USART_StopBits=USART_StopBits_1;
USART_InitStructure.USART_Parity=USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
USART_Init(OW_USART,&USART_InitStructure); //USART2 configuring
USART_Cmd(OW_USART,ENABLE); //USART2 enable
USART_HalfDuplexCmd(OW_USART,ENABLE); //halfduplex enable
USART_DMACmd(OW_USART,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);//OW port DMA enable

//------ USART2 (1-wire port) transmitter DMA configuring — DMA_DeInit(DMA1_Channel7);
DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&(USART2->DR);
DMA_InitStructure.DMA_MemoryBaseAddr=(u32)&ow_buf[0];
DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralDST;
DMA_InitStructure.DMA_BufferSize=sizeof(ow_buf);
DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;
DMA_InitStructure.DMA_Priority=DMA_Priority_Low;
DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
DMA_Init(DMA1_Channel7,&DMA_InitStructure); //DMA1_channel7 initialization
DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE); //DMA1_channel7 interrupt enable

//------ USART2 (1-wire port) receiver DMA configuring — DMA_DeInit(DMA1_Channel6);
DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&(USART2->DR);
DMA_InitStructure.DMA_MemoryBaseAddr=(u32)&ow_buf[0];
DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
DMA_InitStructure.DMA_BufferSize=sizeof(ow_buf);
DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;
DMA_InitStructure.DMA_Priority=DMA_Priority_Low;
DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
DMA_Init(DMA1_Channel6,&DMA_InitStructure); //DMA1_channel6 initialization
DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE); //DMA1_channel6 interrupt enable
//------ end initialization — DMA1_Channel7->CNDTR=(uint32_t)0x00;
DMA1_Channel6->CNDTR=(uint32_t)0x00;
}
//---------------------------------
void ReadData2(u8 datalenght)
{
u8 i;
while(DMA1_Channel6->CNDTR);
DMA1_Channel6->CCR&=~DMA_CCR6_EN;
DMA1_Channel6->CNDTR=datalenght;
DMA1_Channel6->CCR|=DMA_CCR6_EN; //OW-port Rx start 

while(DMA1_Channel7->CNDTR);
for(i=0;i<datalenght;i++)
{
     ow_buf[i]=0xfe;
}
DMA1_Channel7->CCR&=~DMA_CCR7_EN;
DMA1_Channel7->CNDTR=datalenght;
DMA1_Channel7->CCR|=DMA_CCR7_EN; //OW-port Tx start
}
//--------------------------

void OW_ReadData(USART_TypeDef* USARTx, const uint8_t *command, uint8_t *buf, uint16_t len) {
        DMA_InitTypeDef DMA_InitStructure;

        DMA_DeInit(DMA1_Channel6);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) buf;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = len;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize =
                        DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel6, &DMA_InitStructure);

        DMA_DeInit(DMA1_Channel7);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) command;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        DMA_InitStructure.DMA_BufferSize = len;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize =
                        DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel7, &DMA_InitStructure);

        USART_DMACmd(USARTx, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA1_Channel6, ENABLE);
        DMA_Cmd(DMA1_Channel7, ENABLE);

        while (DMA_GetFlagStatus(DMA1_FLAG_TC6) == RESET);
}
//--------------------------------------------------------------------------
/*OW_Init2(USART2)
{int i;
        OW_Reset(OW_USART);
        OW_SendCommand(OW_USART, convert_T, sizeof(convert_T));

        for (i=0; i<1000000; i++);

        OW_Reset(OW_USART);
        OW_ReadData(OW_USART, read_scratch, scratch,  sizeof(read_scratch));

        uint16_t tt=0;

        for (i=16;i<32; i++) {
                if (scratch[i] == 0xff) {
                        tt = (tt>>1) | 0x8000;
                } else {
                        tt = tt>>1;
                }

        }
}*/
//-------------------------------------------------------------------------------------------
				
void OW_Init22(USART_TypeDef* USARTx) 
	{
        GPIO_InitTypeDef GPIO_InitStruct;
        USART_InitTypeDef USART_InitStructure;

        if (USARTx == USART2) {
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

                GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
                GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
                GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

                GPIO_Init(GPIOA, &GPIO_InitStruct);

                GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
                GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
                GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

                GPIO_Init(GPIOA, &GPIO_InitStruct);

                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        }


        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl =
                        USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Tx;

        USART_Init(USARTx, &USART_InitStructure);
        USART_Cmd(USARTx, ENABLE);

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}
//------------------------------------------------------------------

void OW_SendCommand(USART_TypeDef* USARTx, const uint8_t *command, uint16_t len) {
        DMA_InitTypeDef DMA_InitStructure;

        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART2->DR);
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) command;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        DMA_InitStructure.DMA_BufferSize = len;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize =
                        DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel7, &DMA_InitStructure);

        USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
        DMA_Cmd(DMA1_Channel7, ENABLE);
}



				
				
				
				
				
				
				






