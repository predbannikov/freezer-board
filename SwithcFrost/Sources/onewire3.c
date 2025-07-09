/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */
#include <stm32f10x.h>
#include "stm32f10x_dma.h"
#include "onewire3.h"

#define OW_USART 		USART3
#define OW3_DMA_CH_RX 	DMA1_Channel3
#define OW3_DMA_CH_TX 	DMA1_Channel2
#define OW3_DMA_FLAG		DMA1_FLAG_TC3



// Буфер для приема/передачи по 1-wire
uint8_t ow_buf3[16];
uint8_t tbuf3[16];
u16 received_data3;

#define OW3_0	0x00
#define OW3_1	0xff
#define OW3_R_1	0xff
#define OW3_R    0xff

DMA_InitTypeDef     DMA_InitStructure3;
USART_InitTypeDef   USART_InitStructure3;
NVIC_InitTypeDef    NVIC_InitStruct3;



const uint8_t convert_T3[] = {
                OW3_0, OW3_0, OW3_1, OW3_1, OW3_0, OW3_0, OW3_1, OW3_1, // 0xcc SKIP ROM
                OW3_0, OW3_0, OW3_1, OW3_0, OW3_0, OW3_0, OW3_1, OW3_0  // 0x44 CONVERT
};
const uint8_t read_scratch3[] = {
                OW3_0, OW3_0, OW3_1, OW3_1, OW3_0, OW3_0, OW3_1, OW3_1, // 0xcc SKIP ROM
                OW3_0, OW3_1, OW3_1, OW3_1, OW3_1, OW3_1, OW3_0, OW3_1, // 0xbe READ SCRATCH
                OW3_R, OW3_R, OW3_R, OW3_R, OW3_R, OW3_R, OW3_R, OW3_R,
                OW3_R, OW3_R, OW3_R, OW3_R, OW3_R, OW3_R, OW3_R, OW3_R
};
uint8_t scratch3[sizeof(read_scratch3)];
void OW_Init33(USART_TypeDef* USARTx) ;
uint8_t OW_Init3(USART_TypeDef* USARTx);

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
void OW_toBits3(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW3_1;
		} else {
			*ow_bits = OW3_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
uint8_t OW_toByte3(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW3_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

//-----------------------------------------------------------------------------
// инициализирует USART и DMA
//-----------------------------------------------------------------------------
uint8_t OW_Init3(USART_TypeDef* USARTx) {
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;

    NVIC_InitTypeDef NVIC_InitSrt;

	if (OW_USART == USART3) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,
				ENABLE);

		// USART TX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOB, &GPIO_InitStruct);

		// USART RX
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOB, &GPIO_InitStruct);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//???

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
		DMA_DeInit(OW3_DMA_CH_RX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf3;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;// от перефирии к памяти
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// читать по байту
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// однократное срабатывания прерывания 
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;// режим передачи от памяти к памяти откл.
		DMA_Init(OW3_DMA_CH_RX, &DMA_InitStructure);
        USART_DMACmd(OW_USART, USART_DMAReq_Rx, ENABLE);// разрешить работу USART в DMA
        DMA_ITConfig(OW3_DMA_CH_RX,DMA_IT_TC,ENABLE);    // разрешить прерывания от dma по окончанию приёма DMA_IT_TC=по окончанию передачи
        OW3_DMA_CH_RX->CNDTR=(uint32_t)0x00;
	
	// DMA на запись
		DMA_DeInit(OW3_DMA_CH_TX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf3;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;// от памяти к перефирии
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(OW3_DMA_CH_TX, &DMA_InitStructure);
        USART_DMACmd(OW_USART, USART_DMAReq_Tx, ENABLE);
        DMA_ITConfig(OW3_DMA_CH_TX,DMA_IT_TC,ENABLE);
        
        
        

		// старт цикла отправки
		//USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
		//USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(OW3_DMA_CH_RX, ENABLE);
		DMA_Cmd(OW3_DMA_CH_TX, ENABLE);
    //------------------------------------------------
    NVIC_InitSrt.NVIC_IRQChannel=DMA1_Channel3_IRQn; //прерывание от 
    NVIC_InitSrt.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitSrt.NVIC_IRQChannelSubPriority=0;
    NVIC_InitSrt.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitSrt);
    
   // NVIC_EnanableIRQ(DMA1_Channel2_IRQn);
    
    
    
	return OW3_OK;//?????
}

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
uint8_t OW_Reset3(USART_TypeDef* USARTx) {
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
		return OW3_OK;
	}

	return OW3_NO_DEVICE;
}
//---------------PA2---------------------
void OW_out_set_Power_pin10(void)
{// PA2 TX for PowerUp
GPIO_InitTypeDef GPIO_In;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

GPIO_SetBits(GPIOA,GPIO_Pin_10);

GPIO_In.GPIO_Pin   =   GPIO_Pin_10;
GPIO_In.GPIO_Mode  =   GPIO_Mode_Out_PP;
GPIO_In.GPIO_Speed =   GPIO_Speed_50MHz;
GPIO_Init(GPIOB,&GPIO_In);
}

void OW_out_Rset_Power_pin10(void)
{// Pb10 TX for PowerUp
GPIO_InitTypeDef GPIO_In;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

GPIO_ResetBits(GPIOA,GPIO_Pin_10);

GPIO_In.GPIO_Pin   =   GPIO_Pin_10;
GPIO_In.GPIO_Mode  =   GPIO_Mode_Out_OD;
GPIO_In.GPIO_Speed =   GPIO_Speed_50MHz;
GPIO_Init(GPIOB,&GPIO_In);
}

void OW_out_set_TX_pin10(void)
{// PB10 TX for USART3
GPIO_InitTypeDef GPIO_In;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
GPIO_In.GPIO_Pin   =   GPIO_Pin_10 ;
GPIO_In.GPIO_Mode  =   GPIO_Mode_AF_OD;
GPIO_In.GPIO_Speed =   GPIO_Speed_50MHz;
GPIO_Init(GPIOB,&GPIO_In);

 
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

uint8_t OW_Send3(USART_TypeDef* USARTx,uint8_t sendReset, uint8_t *command, uint8_t cLen,uint8_t *data, uint8_t dLen, uint8_t readStart)
{
//uint8_t i;
	// если требуется сброс - сбрасываем и проверяем на наличие устройств
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset3(USARTx) == OW3_NO_DEVICE) {
			return OW3_NO_DEVICE;
		}
	}
 //
  
	while (cLen > 0) {

		OW_toBits3(*command, ow_buf3);
		command++;
		cLen--;
        
		DMA_InitTypeDef DMA_InitStructure;

		// DMA на чтение
		DMA_DeInit(OW3_DMA_CH_RX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf3;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(OW3_DMA_CH_RX, &DMA_InitStructure);

		// DMA на запись
		DMA_DeInit(OW3_DMA_CH_TX);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->DR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf3;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(OW3_DMA_CH_TX, &DMA_InitStructure);

		// старт цикла отправки
		
		USART_DMACmd(USARTx, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(OW3_DMA_CH_RX, ENABLE);
		DMA_Cmd(OW3_DMA_CH_TX, ENABLE);

		// Ждем, пока не примем 8 байт
		while (DMA_GetFlagStatus(OW3_DMA_FLAG) == RESET)
        {;}

		// отключаем DMA
    USART_ClearFlag(USARTx, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
	DMA_Cmd(OW3_DMA_CH_TX, DISABLE);
	DMA_Cmd(OW3_DMA_CH_RX, DISABLE);
	USART_DMACmd(USARTx, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);
    
		// если прочитанные данные кому-то нужны - выкинем их в буфер
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte3(ow_buf3);
			data++;
			dLen--;
		   } 
      else {
			if (readStart != OW3_NO_READ) {
				readStart--;
			}
		}
	}

	return OW3_OK;
}
//-----------DMA--------------------------------
uint16_t ReadData3(uint8_t l)
{
uint8_t i;
uint16_t received_d;
//char rx_buf[32];

while(DMA1_Channel3->CNDTR);//RX

DMA1_Channel3->CCR&=~DMA_CCR3_EN; // запрет
DMA1_Channel3->CNDTR=l;           //размер буфера 
DMA1_Channel3->CCR|=DMA_CCR3_EN;  //rx start

while(DMA1_Channel2->CNDTR);
for(i=0;i<l;i++)ow_buf3[i]=0xff;

DMA1_Channel2->CCR&=~DMA_CCR2_EN;//TX запрет
DMA1_Channel2->CNDTR=l;
DMA1_Channel2->CCR|=DMA_CCR2_EN;//tx start будет работать и выдаст прерывания

 while(!(DMA1->ISR&DMA_ISR_TCIF3)); // RX ???
 DMA1->IFCR|=DMA_IFCR_CTCIF6;       // очистить флаг прерывания приёма 
 DMA1_Channel3->CCR&=~DMA_CCR3_EN;
 
 received_d=0x0000;
 for(i=0;i<16;i++)
 {
   if(ow_buf3[i]==0xff)
   {
   received_d|=(1<<i);
   }
 }
 ;
return received_d;
}




//----------------------PA2 1Wire----------------------

//--------------------------

void OW_ReadData3(USART_TypeDef* USARTx, const uint8_t *command, uint8_t *buf, uint16_t len) {
        DMA_InitTypeDef DMA_InitStructure;

        DMA_DeInit(DMA1_Channel3);
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
        DMA_Init(DMA1_Channel3, &DMA_InitStructure);

        DMA_DeInit(DMA1_Channel2);
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
        DMA_Init(DMA1_Channel2, &DMA_InitStructure);

        USART_DMACmd(USARTx, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA1_Channel3, ENABLE);
        DMA_Cmd(DMA1_Channel2, ENABLE);

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
				
/*void OW_Init22(USART_TypeDef* USARTx) 
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

}*/
//------------------------------------------------------------------

void OW_SendCommand3(USART_TypeDef* USARTx, const uint8_t *command, uint16_t len) {
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
        DMA_Init(DMA1_Channel2, &DMA_InitStructure);

        USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
        DMA_Cmd(DMA1_Channel2, ENABLE);
}



				
				
				
				
				
				
				






