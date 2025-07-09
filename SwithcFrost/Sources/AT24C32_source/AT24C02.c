#include "stm32f10x.h"                  // Device header
#include "AT24C02.h"
#include "stm32f10x_i2c.h"

#define I2C_EE I2C1
#define EEPROM_HW_ADDRESS 0xA0
#define ADR_EEPROM        0xA0  
//-------------------------------------------------------------------------------------------------------
/*
void AT24C02_Write(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte, unsigned char Value)
{
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	I2C_GenerateSTART(I2Cx, ENABLE);
	while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

	I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ); // wait send Address

	I2C_SendData(I2Cx, AddressByte);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // wait send Address Byte

	I2C_SendData(I2Cx, Value);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // wait Send Value for Byte

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop
};

unsigned char AT24C02_Read(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte)
{
	unsigned char ReceiveData = 0;

	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);
	while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

	I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ); // wait send Address Device

	I2C_SendData(I2Cx, AddressByte);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) ); // wait send Address Byte

	I2C_GenerateSTART(I2Cx, ENABLE);
	while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

	I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) ); // wait Send Address Device As Receiver

	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ); // wait Receive a Byte
	ReceiveData = I2C_ReceiveData(I2Cx);

//	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
	I2C_PECPositionConfig(I2Cx, I2C_PECPosition_Current); 
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition
	

	I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

	return ReceiveData;
};*/
//---------------------------------------------------------------------
/***************************************************************************//**
 *  @brief  I2C Configuration
 ******************************************************************************/
 /*
void I2C_Configuration(void)
{

           I2C_InitTypeDef  I2C_InitStructure;
           GPIO_InitTypeDef  GPIO_InitStructure;

           RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

           RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO , ENABLE);//

           // Configure I2C1 pins: PB6->SCL and PB7->SDA  
           GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
           GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
           GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
           GPIO_Init(GPIOB, &GPIO_InitStructure);

           I2C_DeInit(I2C_EE);
           I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
           I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
           I2C_InitStructure.I2C_OwnAddress1 = 1;
           I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
           I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
           I2C_InitStructure.I2C_ClockSpeed = 100000; // 100kHz  

           I2C_Cmd(I2C_EE, ENABLE);
           I2C_Init(I2C_EE, &I2C_InitStructure);
           I2C_AcknowledgeConfig(I2C_EE, ENABLE);

}
*/

void I2C_EE_ByteWrite(uint8_t val, uint16_t WriteAddr)
{


    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_HW_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


    /* Send the EEPROM's internal address to write to : MSB of the address first */
    I2C_SendData(I2C_EE, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));



    /* Send the EEPROM's internal address to write to : LSB of the address */
    I2C_SendData(I2C_EE, (uint8_t)(WriteAddr & 0x00FF));

    /* Test on EV8 and clear it */
    while(! I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


     I2C_SendData(I2C_EE, val);

        /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);

    //delay between write and read...not less 4ms
  //  delay_ms(5);
}
//*********************************************************************************
uint8_t I2C_EE_ByteRead( uint16_t ReadAddr)
{
    uint8_t tmp;

        /* While the bus is busy */
    while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_HW_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


    /* Send the EEPROM's internal address to read from: MSB of the address first */
    I2C_SendData(I2C_EE, (uint8_t)((ReadAddr & 0xFF00) >> 8));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the EEPROM's internal address to read from: LSB of the address */
    I2C_SendData(I2C_EE, (uint8_t)(ReadAddr & 0x00FF));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


    /* Send STRAT condition a second time */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for read */
    I2C_Send7bitAddress(I2C_EE, EEPROM_HW_ADDRESS, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE,I2C_EVENT_MASTER_BYTE_RECEIVED));//I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    tmp=I2C_ReceiveData(I2C_EE);


    I2C_AcknowledgeConfig(I2C_EE, DISABLE);

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);

    return tmp;
    }
//--------------------------------------------------------------------------------
		
		
void RecToEEPROM(uint16_t Adr_OffSet_EPROM,	char Data[], uint8_t Len)
{
uint16_t adr_eeprom=0,EndAdr;
 // 28 FF 4B 48 54 15 03 39
EndAdr= Adr_OffSet_EPROM+Len;
	
	
				 for(Adr_OffSet_EPROM;Adr_OffSet_EPROM<=EndAdr;Adr_OffSet_EPROM++)
				 {
					  I2C_EE_ByteWrite(Data[adr_eeprom++],Adr_OffSet_EPROM);
					  delay_ms(80);
					 
				 }


}	
//-----------------------------------------------------------------------------------

int Read_ByteEEPROM(char read_to_buf[], int ReadAddr,uint8_t Len)
{
	int EndAdr,indx=0;
//	char temp_ds[4096];
	EndAdr=ReadAddr+Len;

				  for(ReadAddr;ReadAddr<EndAdr;ReadAddr++)
				 {
					read_to_buf[indx++]= I2C_EE_ByteRead(ReadAddr);
					 
					  delay_ms(10);
				 } 
			 
return *read_to_buf;

}
//---------------------------------------------------------------------------------------



		
		
		
		
		
		
		
		
		
		
		

















