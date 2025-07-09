#include "i2c.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h" // не забываем указать ( 3 галочки) библиотеки
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
int ff;
void delay_ms(uint32_t ms)           // милисекунды
{       volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
        RCC_GetClocksFreq (&RCC_Clocks);
        nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
        for(; nCount!=0; nCount--);
}
void delay_uc(uint32_t mc)           // микросекунды
{       volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
        RCC_GetClocksFreq (&RCC_Clocks);
        nCount=(RCC_Clocks.HCLK_Frequency/10000000)*mc;
        for(; nCount!=0; nCount--);
}

GPIO_InitTypeDef gpio,GPIO_InitStructure;
I2C_InitTypeDef i2C;
//I2C_InitTypeDef kk;

void I2C_init1(void)
{       // Включаем тактирование нужных модулей
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        // настройка I2C
       // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	
	   // I2C использует две ноги микроконтроллера, их тоже нужно настроить
        //               CLK           SDA
        gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
        gpio.GPIO_Mode = GPIO_Mode_AF_OD;
        gpio.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOB, &gpio);
	
        
        i2C.I2C_ClockSpeed = 40000;  //100000 скорость шины 100 кГц
        i2C.I2C_Mode = I2C_Mode_I2C;
        i2C.I2C_DutyCycle = I2C_DutyCycle_16_9;//I2C_DutyCycle_2;
        i2C.I2C_OwnAddress1 = 00;      // тут не важно какой адрес - потом он встанет из #define LCD_ADR
        i2C.I2C_Ack = I2C_Ack_Enable;
        i2C.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        I2C_Init(I2C1, &i2C);
     
        // включаем модуль I2C1
        I2C_Cmd(I2C1, ENABLE);
}
//--------------------------------------------------------
void I2C_init(void) {
RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_Init(GPIOB, &GPIO_InitStructure);

I2C_InitTypeDef I2C_InitStructure;
I2C_StructInit(&I2C_InitStructure);

I2C_InitStructure.I2C_ClockSpeed = 100000;
I2C_InitStructure.I2C_OwnAddress1 = 0;
I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
I2C_Init(I2C1, &I2C_InitStructure);
I2C_Cmd(I2C1, ENABLE);
}
/*
void I2C_init1(void) {

    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    // enable APB1 peripheral clock for I2C1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    // enable clock for SCL and SDA pins
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

     setup SCL and SDA pins
     * You can connect the I2C1 functions to two different
     * pins:
     * 1. SCL on PB6 or PB8
     * 2. SDA on PB7 or PB9
     
    GPIO_InitStruct.GPIO_Pin =    GPIO_Pin_8 | GPIO_Pin_9; // we are going to use PB8 and PB9
    GPIO_InitStruct.GPIO_Mode=    GPIO_Mode_AF; // set pins to alternate function
    GPIO_InitStruct.GPIO_Speed =  GPIO_Speed_50MHz; // set GPIO speed
    GPIO_InitStruct.GPIO_OType =  GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
    GPIO_InitStruct.GPIO_PuPd =   GPIO_PuPd_UP; // enable pull up resistors
    GPIO_Init(GPIOB, &GPIO_InitStruct); // init GPIOB

    // Connect I2C1 pins to AF
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA

    // configure I2C1
    I2C_InitStruct.I2C_ClockSpeed = 100000; // 100kHz --> slow mode
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // I2C mode
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
    I2C_InitStruct.I2C_OwnAddress1 = 0x00; // own address, not relevant in master mode
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable; // disable acknowledge when reading (can be changed later on)
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
    I2C_Init(I2C1, &I2C_InitStruct); // init I2C1

    // enable I2C1
    I2C_Cmd(I2C1, ENABLE);
    printf("\n\rI2C1 INITIALIZED!\n\r");
}
*/
/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 *      I2Cx --> the I2C peripheral e.g. I2C1
 *      address --> the 7 bit slave address
 *      direction --> the transmission direction can be:
 *                      I2C_Direction_Tranmitter for Master transmitter mode
 *                      I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
    // wait until I2C1 is not busy any more
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		{ff=ff;}

    // Send I2C1 START condition
    I2C_GenerateSTART(I2Cx, ENABLE);

    // wait for I2C1 EV5 --> Slave has acknowledged start condition
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        ;

    // Send slave Address for write
    I2C_Send7bitAddress(I2Cx, address, direction);

    /* wait for I2Cx EV6, check if
     * either Slave has acknowledged Master transmitter or
     * Master receiver mode, depending on the transmission
     * direction
     */
    if (direction == I2C_Direction_Transmitter) {
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
            ;
    } else if (direction == I2C_Direction_Receiver) {
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
            ;
    }
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx) {
uint8_t data;
    // enable acknowledge of received data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // wait until one byte has been received
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        ;
    // read data from I2C data register and return data byte
     data = I2C_ReceiveData(I2Cx);
    return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the received data
 * after that a STOP condition is transmitted
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx) {
uint8_t data; 
    // disable acknowledge of received data
    // nack also generates stop condition after last byte received
    // see reference manual for more info
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // wait until one byte has been received
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        ;
    // read data from I2C data register and return data byte
    data = I2C_ReceiveData(I2Cx);
    return data;
}

/* This function issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx) {
    // Wait until end of transission
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
        ;
    // Send I2C1 STOP Condition after last byte has been transmitted
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // wait for I2C1 EV8_2 --> byte has been transmitted
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;
}

/* This function transmits one byte to the slave device
 * Parameters:
 *      I2Cx --> the I2C peripheral e.g. I2C1
 *      data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
    // wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
        ;
    I2C_SendData(I2Cx, data);
}

//-----------------------------------------


//  Адрес часов 0х68, адрес EEPROM 0х50. Если у вас модуль с EEPROM.
// пофиг какой он потом станет 0x68 в ds1307.h
//#define AP3216_ADDRESS 0x1E
//#define AP3216_ADDRESS 0xD0 
#define AP3216_ADDRESS 0x00

void WriteRegisterI2C(uint8_t RegisterAddress, uint8_t RegisterCommand)
{
I2C_GenerateSTART(I2C1, ENABLE);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
I2C_Send7bitAddress(I2C1, AP3216_ADDRESS<<1, I2C_Direction_Transmitter);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
I2C_SendData(I2C1, RegisterAddress);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
I2C_SendData(I2C1, RegisterCommand);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
I2C_GenerateSTOP(I2C1, ENABLE);
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
}

uint8_t ReadRegisterI2C(uint8_t RegisterAddress)
{
uint8_t data;
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
I2C_GenerateSTART(I2C1, ENABLE);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
I2C_Send7bitAddress(I2C1, AP3216_ADDRESS<<1, I2C_Direction_Transmitter);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
I2C_SendData(I2C1, RegisterAddress);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
I2C_GenerateSTART(I2C1, ENABLE);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
I2C_Send7bitAddress(I2C1, AP3216_ADDRESS<<1, I2C_Direction_Receiver);
while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
data = I2C_ReceiveData(I2C1);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
I2C_AcknowledgeConfig(I2C1, DISABLE);
I2C_GenerateSTOP(I2C1, ENABLE);
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
return data;
}
//========================================================================
