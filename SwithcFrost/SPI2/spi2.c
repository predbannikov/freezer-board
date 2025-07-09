
//PB13 PB14 PB15 PB12
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h" 
#include "stm32f10x_rcc.h"
#include "spi2.h"	
#define SPI_CR2_TXEIE_pos6bit ((uint8_t)0x80) //проверить в дебугере
int32_t tx_index = 0; //тут хранится количество переданных байт
int32_t tx_len = 0;   //сколько всего байт нужно передать
uint8_t *tx_data;     //указатель на массив с передаваемыми данными

		
void _SPI2_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	  EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
	
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


GPIO_StructInit (&GPIO_InitStructure);
//Confikure SPI pins: SCK omd MOSI wyth default alternate function (not re-mapped) push-pull 
GPIO_InitStructure.GPIO_Pin   = SCK | MOSI;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
GPIO_Init(GPIOB, &GPIO_InitStructure);

//GPIO_StructInit (&GPIO_InitStructure);
// Confikure MISO as Input wyth internal pull-up 
GPIO_InitStructure.GPIO_Pin   = MISO;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;//GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;// GPIO_Mode_AF_PP;//GPIO_Mode_IPU;
GPIO_Init(GPIOB, &GPIO_InitStructure);

//GPIO_StructInit (&GPIO_InitStructure);
//Confikure SS as Output_PP 
GPIO_InitStructure.GPIO_Pin = CS;//|GPIO_Pin_0; //Pin_0 used only for debugging
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);

SPI_I2S_DeInit(SPI2);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

SPI_InitTypeDef   SPI2_InitStructure;

SPI_StructInit (&SPI2_InitStructure);

SPI2_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
SPI2_InitStructure.SPI_Mode = SPI_Mode_Master;
SPI2_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
SPI2_InitStructure.SPI_DataSize = SPI_DataSize_8b;
SPI2_InitStructure.SPI_CPOL = SPI_CPOL_Low;
SPI2_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
SPI2_InitStructure.SPI_NSS =SPI_NSS_Hard;// SPI_NSS_Soft;
SPI2_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
SPI2_InitStructure.SPI_CRCPolynomial = 7;
SPI_Init(SPI2, &SPI2_InitStructure);

SPI_CalculateCRC(SPI2, DISABLE);

 NVIC_EnableIRQ(SPI2_IRQn); //Разрешаем прерывания от SPI1
  
 // SPI1->CR1 |= 1<<SPI_CR1_SPE_Pos; //Включаем SPI

SPI_Cmd(SPI2, ENABLE);

//NvicInit();

}
void _SPI2_Init2(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	  EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
	
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);

//***********************DRDY*************************  ---|__|--
GPIO_InitStructure.GPIO_Pin = DRDY ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//  GPIO_Mode_AF_PP;//     Input wyth internal pull-up 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
//	NvicInit();
//************************************************



GPIO_StructInit (&GPIO_InitStructure);
//Confikure SPI pins: SCK omd MOSI wyth default alternate function (not re-mapped) push-pull 
GPIO_InitStructure.GPIO_Pin   = SCK | MOSI;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
GPIO_Init(GPIOB, &GPIO_InitStructure);

//GPIO_StructInit (&GPIO_InitStructure);
// Confikure MISO as Input wyth internal pull-up 
GPIO_InitStructure.GPIO_Pin   = MISO;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
GPIO_Init(GPIOB, &GPIO_InitStructure);

//GPIO_StructInit (&GPIO_InitStructure);
//Confikure SS as Output_PP 
GPIO_InitStructure.GPIO_Pin = CS;//|GPIO_Pin_0; //Pin_0 used only for debugging
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);

SPI_I2S_DeInit(SPI2);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

SPI_InitTypeDef   SPI2_InitStructure;

SPI_StructInit (&SPI2_InitStructure);

SPI2_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
SPI2_InitStructure.SPI_Mode = SPI_Mode_Master;
SPI2_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
SPI2_InitStructure.SPI_DataSize = SPI_DataSize_8b;
SPI2_InitStructure.SPI_CPOL = SPI_CPOL_Low;
SPI2_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
SPI2_InitStructure.SPI_NSS = SPI_NSS_Hard;//SPI_NSS_Soft;//
SPI2_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
SPI2_InitStructure.SPI_CRCPolynomial = 7;
SPI_Init(SPI2, &SPI2_InitStructure);

SPI_CalculateCRC(SPI2, DISABLE);

 NVIC_EnableIRQ(SPI2_IRQn); //Разрешаем прерывания от SPI1
  
 // SPI1->CR1 |= 1<<SPI_CR1_SPE_Pos; //Включаем SPI

SPI_Cmd(SPI2, ENABLE);



}
//**********************************
void SPI2_Write(uint8_t data)
{
  //Ждем, пока не освободится буфер передатчика
  while(!(SPI2->SR & SPI_SR_TXE));
    //заполняем буфер передатчика
  SPI2->DR = data;
}
//*********************************
uint8_t SPI2_Read(void)
{
  SPI2->DR = 0; //запускаем обмен
  
  //Ждем, пока не появится новое значение 
  //в буфере приемника
  while(!(SPI2->SR & SPI_SR_RXNE)) ;
   //возвращаем значение буфера приемника
  return SPI2->DR;
}
//**********************************
void SPI_Send_Byte(uint8_t data)
{
while(!(SPI2->SR & SPI_I2S_FLAG_TXE));
SPI2->DR = data;
}
//***********************************
/*void SPI2_IRQHandler(void)
{
  SPI2->DR = tx_data[tx_index]; //Записываем новое значение в DR
  tx_index++; //увеличиваем счетчик переданных байт на единицу
  
  //если все передали, то отключаем прерывание,
  //тем самым завершаем передачу данных
  if(tx_index >= tx_len)
    SPI2->CR2 &= ~(1<<SPI_CR2_TXEIE_pos6bit); 
	
	
}*/
//***********************************

void SPI2_Tx(uint8_t *data, int32_t len)
{
  if(len<=0)
    return;
  
  //Ждем, пока SPI освободится от предыдущей передачи
  while(SPI2->SR & SPI_SR_BSY);
    
  //Настройка переменных, которые будут
  //использоваться в обработчике прерывания SPI
  tx_index = 0;
  tx_len = len;
  tx_data = data;
  
  //Разрешаем прерывание TXEIE И запускаем обмен
  SPI2->CR2 |= (1<<SPI_CR2_TXEIE_pos6bit); //SPI_CR2_TXEIE ???? может маску поставить???
}
//********************************************************

void exti_PB11(void)
{
	RCC -> APB2ENR |= (RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);             //CMD PORT B. cmd alt func
 GPIOB ->CRH &= ~(GPIO_CRH_CNF11_0);        //Scan Pins: PB11, PB12, PB13, PB14.
 GPIOB ->CRH |= GPIO_CRH_CNF11_1;         //Input Push-down.
 GPIOB ->BSRR |= 0x1E0;      
 AFIO ->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;
    // AFIO -> EXTICR[3] = AFIO_EXTICR4_EXTI12_PB | AFIO_EXTICR4_EXTI13_PB | AFIO_EXTICR4_EXTI14_PB;
 EXTI -> RTSR |= EXTI_FTSR_TR11;//EXTI_RTSR_TR11;//
 EXTI -> IMR |= EXTI_IMR_MR11;
 NVIC_EnableIRQ(EXTI15_10_IRQn);
//__enable_irq(); 

	

}
//****************************************
/*void SPI1_Config(void){
GPIO_InitTypeDef GPIO_InitStruct;
SPI_InitTypeDef SPI_InitStruct;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOA, &GPIO_InitStruct);

GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(GPIOE, &GPIO_InitStruct);
GPIOE->BSRRL |= GPIO_Pin_7;

SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
SPI_Init(SPI1, &SPI_InitStruct);

SPI_Cmd(SPI1, ENABLE);}

*/





//******************





   