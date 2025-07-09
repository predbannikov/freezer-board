
#include <stm32f10x.h>
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





u8 OW_Buffer[16];
u8 convert_T2[]={0x00,0x00,0xff,0x00,0x00,0x00,0xff,0x00}; //0x44
u8 skip_ROM[]={0x00,0x00,0xff,0xff,0x00,0x00,0xff,0xff}; //0xcc
u8 read_scratchpad[]={0x00,0xff,0xff,0xff,0xff,0xff,0x00,0xff};//0xbe
u16 received_data;









//-------------------------------------------
/*void Measuring(void)
{
u32 ticks;
u8 i;
OW_Reset(OW_USART); //reset OW line
//SendOWCommand(skip_ROM,sizeof(skip_ROM)); //send 0xcc
//SendOWCommand(convert_T2,sizeof(convert_T2)); //send 0x44
//ticks=os_time_get(); //get current time
//while((os_time_get()-ticks)<750); //delay 750ms
 OW_out_set_TX_pin();
              OW_Send(OW_SEND_RESET,  "\xcc\x44",2,  tbuf, 2, 0);
               OW_out_set_Power_pin();
                 for (i=0; i<10000; i++);  
                 
OW_Reset(OW_USART); //reset OW line
//SendOWCommand(skip_ROM,sizeof(skip_ROM)); //send 0xcc
//SendOWCommand(read_scratchpad,sizeof(read_scratchpad)); //send 0xbe
ReadData(16); //send 0xffff and read 16 byte
while(!(DMA1->ISR&DMA_ISR_TCIF6));
DMA1->IFCR|=DMA_IFCR_CTCIF6;
DMA1_Channel6->CCR&=~DMA_CCR6_EN;
received_data=0x0000;
for(i=0;i<16;i++)
{
if(OW_Buffer[i]==0xff)
{
received_data|=(1<<i);
}
}
}*/
//---------------------------------
u8 OW_Reset2(USART_TypeDef* port)
{
while(!(port->SR&USART_FLAG_TC)); //wait, while port is not ready
port->BRR=0x0EA6; //baudrate 9600
port->DR=(uint8_t)0xf0; //send 'reset' pulse
while(!(port->SR&USART_FLAG_TC));
port->BRR=0x0138; //baudrate 115200
if(port->DR!=(uint8_t)0xf0) //if sensor connected
{
return 1;
}
return 0;
}

void SendOWCommand2(u8* command,u8 count)
{
u8 i;
while(DMA1_Channel7->CNDTR);
for(i=0;i<count;i++)
{
OW_Buffer[i]=*command++;
}
DMA1_Channel7->CCR&=~DMA_CCR7_EN;
DMA1_Channel7->CNDTR=count;
DMA1_Channel7->CCR|=DMA_CCR7_EN; //send command start
}

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
OW_Buffer[i]=0xff;
}
DMA1_Channel7->CCR&=~DMA_CCR7_EN;
DMA1_Channel7->CNDTR=datalenght;
DMA1_Channel7->CCR|=DMA_CCR7_EN; //OW-port Tx start
}