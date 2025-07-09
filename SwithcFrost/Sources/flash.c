#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "flash.h"

/* The size of each page of STM32 large-capacity products is 2KByte, and the size of each page of medium and small-capacity products is 1KByte */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE       ((uint16_t)0x800)	// 2048
#else
  #define FLASH_PAGE_SIZE       ((uint16_t)0x400)	// 1024
#endif

#define WRITE_START_ADDR        ((uint32_t)0x08008000)
#define WRITE_END_ADDR          ((uint32_t)0x0800C000)

//EEPROM_BASE = 0x08000000U +0x800 *0x7f  => 0x803F800
//cct6 256k
//#define EEPROM_BASE  (FLASH_BASE + 0x800 * 0x7f) /* Last page of FLASH memory */
//stm32f103c8t6  64 k
#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define EEPROM_BASE  (FLASH_BASE + 0x800 * 0x1f) //2048 *31 =F800  0x08000000U+F800 =0x0800F800
#define EEPROM_OFFSET 8
#define CAL_SIZE (sizeof(struct Calibration) / sizeof(int))
#define EEPROM_LENGTH 64 /* Whatever, just don't run out of memory*/
#define EEPROM_BLANK  0xffffffffU
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
// P*s*  FLASH_KEY1 есть в stm32f10x_flash.h но компил€тор ругаетс€

static unsigned int buf[EEPROM_LENGTH]; /* Better safe than sorry */

//******************************
/*struct Calibration
{
  float slope;    // склон
  float intercept;//кусок пр€мой
//	 uint8_t id;
}calibration;
 */
struct Sensor
{
 // struct SDADC *hSdadc;
  uint8_t channel;
  uint8_t id;

  struct Calibration calibration;
  float offset;
};
//**************************
static inline int flash_is_blank(unsigned int *flashPtr, unsigned int len)
{
	for(unsigned int i = 0; i < len; ++i) {
		if(flashPtr[i] != EEPROM_BLANK)
			return 0;
		else
			continue;
	}

	return 1;
}
static inline int flash_check_eop(void)
{
	if(FLASH->SR & FLASH_SR_EOP) {

		FLASH->SR |= FLASH_SR_EOP;
		return 1;
	} else
		return 0;
}

static inline void flash_wait_busy(void)
{
	while(FLASH->SR & FLASH_SR_BSY) ;
}

static void flash_unlock(int unlock)
{
	if(unlock != 0) {
		__disable_irq();
	//	FLASH->KEYR =
		FLASH->KEYR =   FLASH_KEY1;
		FLASH->KEYR =   FLASH_KEY2;
		__enable_irq();

	} else {
		FLASH->CR |= FLASH_CR_LOCK;
	}
}

static int flash_erase(void)
{
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR  = EEPROM_BASE;
	FLASH->CR |= FLASH_CR_STRT;

	flash_wait_busy();

	FLASH->CR &= ~FLASH_CR_PER;

	return flash_check_eop();
}

/* Write to flash is only allowed by half-words (16 bit) */
static inline void flash_write_helper(void *dataPtr, void *flashPtr, unsigned int len)
{
	uint16_t *dataPtrHw  = (uint16_t*)dataPtr;
	uint16_t *flashPtrHw = (uint16_t*)flashPtr;

	FLASH->CR |= FLASH_CR_PG;

	for(uint32_t i = 0; i < len * 2; ++i) {
		flashPtrHw[i] = dataPtrHw[i];
		flash_wait_busy();
		flash_check_eop();
	}

	FLASH->CR &= ~FLASH_CR_PG;
}


//**************************************************************************
int Flash_Write(unsigned int address, unsigned int data[], unsigned int len)
{
	if((address + len) > EEPROM_LENGTH)
		return 0;

	unsigned int *flashPtr = (unsigned int*)EEPROM_BASE + address;

	/* Check if the required range is not already set to correct value */
//	if(memcmp(data, flashPtr,16 /*len * sizeof(data)*/) == 0)
	//	return 0;

	flash_unlock(1);

	if(!flash_is_blank(flashPtr, len)) {
		memcpy(buf, (void*)EEPROM_BASE, EEPROM_LENGTH * sizeof(unsigned int));

		flash_erase();

		for(unsigned int i = 0; i < len; ++i) {
			buf[address + i] = data[i];
		}

		flash_write_helper(buf, (void*)EEPROM_BASE, EEPROM_LENGTH);
	} 
	
	else {
		flash_write_helper(data, flashPtr, len);
	}

	flash_unlock(0);

	/* Read back flash contents and compare with data */
	if(memcmp(flashPtr, data, len * sizeof(unsigned int)) == 0) return 1;
	else return 0;
}
//*******************************************************************
int Flash_WriteWord(unsigned int address, unsigned int value)
{
	return Flash_Write(address, &value, 1);
}
//********************************************************************
//void Sensor_SetCalibration(struct Sensor *sensor, struct Calibration calibration)
void Sensor_SetCalibration(unsigned int adr, unsigned int data[])//,unsigned int len)
{
	//calibration.slope=(float)data[0];
	calibr_LoadCell.slope=data[0];
	Flash_Write(adr, (unsigned int*)&(calibr_LoadCell), CAL_SIZE);
//	Flash_Write(adr, data, len);
}
//**********************************************
void Sensor_SetCalibration2f(struct Calibration *calibr, float slope, float intercept,int id)
{// calibr->id=id;
	
	//calibr->slope = slope;
//	calibr->intercept = intercept;
//	Flash_Write((CAL_SIZE * id) + EEPROM_OFFSET, (unsigned int*)&(calibr), CAL_SIZE);
	Flash_Write(id, (unsigned int*)&(calibr), 8);
	
}



//***********************************************
unsigned int Flash_ReadWord(unsigned int address)
{
	return ((unsigned int*)EEPROM_BASE)[address];
}
//***********************************************
float Flash_ReadFloat(unsigned int address)
{
	return ((float*)EEPROM_BASE)[address];
}
//***********************************************
void Flash_Read(unsigned int address, unsigned int *dst, unsigned int len)
{
	unsigned int *readAddr = (unsigned int*)EEPROM_BASE + address;
	memcpy(dst, (void*)readAddr, len * sizeof(unsigned int));
}
//***********************************************
unsigned int Sensor_GetCalibration(struct Calibration *s)
{
	return (unsigned int)s->slope;
}
//***********************************************
float Sensor_ReadCalibration(struct Calibration *calibr)
{
	//01 03 00 02 00 04 E5 C9  ask
	//01 03 00 02 00 04 08 00000000 00000000 crc crc
unsigned int adr= 0x00;//=((CAL_SIZE * calibr->id) + EEPROM_OFFSET);
volatile unsigned int Slope 	=	Flash_ReadWord(adr);
volatile unsigned int Intercept =	Flash_ReadWord(adr+1);

	calibr->slope=Slope;
	calibr->intercept =Intercept;
	
	volatile unsigned b0,b1,b2,b3,b4,b5,b6,b7;
	const int destOffset =  0x02;

//01 03 08   C0 FA 3C 79 00 00 00 00 5B 7E
// in eeprom FA C0 79 3C
b0=Slope&0xff;      //0xa1
b1=(Slope>>8)&0xff; //0xa0
b2=(Slope>>16)&0xff;//0xa0
b3=(Slope>>24)&0xff;//0x3c
//Intercept=0x11223344;
b4=(Intercept)&0xff;     //0xe2
b5=(Intercept>>8)&0xff;  //0xe1
b6=(Intercept>>16)&0xff;  //0xa1
b7=(Intercept>>24)&0xff;  //0xc1

	

return	0;
}











/**
   @brief internal Flash write
   @param address -[in] write address
   @param pData -[in&out] points to the data to be manipulated
   @param dataLen -[in] data length
   @return true-success; false-failure
*/
/*int WriteFlash(uint32_t addrStart, uint32_t *pData, uint32_t dataLen)
{   
    uint32_t i = 0;
    uint32_t eraseCounter = 0x00;                                                               // record how many pages to erase
    uint32_t address = 0x00;                                                                    // Record the address written
    uint32_t numberOfPage = 0x00;                                                               // record how many pages are written
    FLASH_Status flashStatus = FLASH_COMPLETE;                                                  // Record the result of each erasure
    
    address = addrStart;
    
    FLASH_Unlock();                                                                             // unlock
  //  numberOfPage = (WRITE_END_ADDR - address) / FLASH_PAGE_SIZE;                                // Calculate how many pages to erase
	numberOfPage =1;
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);  // Clear all flags
    
    // Erase by page
    for(eraseCounter = 0; (eraseCounter < numberOfPage) && (flashStatus == FLASH_COMPLETE); eraseCounter++)
    {
        flashStatus = FLASH_ErasePage(address + (FLASH_PAGE_SIZE * eraseCounter));
	}
    
    for(i = 0; (i < dataLen)&&(flashStatus == FLASH_COMPLETE); i++)
    {
        flashStatus = FLASH_ProgramWord(address, pData[i]);                                     // Write a word (32 bits) of data into the specified address
        address = address + 4;                                                                  // The address is offset by 4 bytes 
    }
		
    //FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data)
    FLASH_Lock();                                                                               // relock
    
    if(flashStatus == FLASH_COMPLETE)
    {
        return 1;
    }
    return 0;
}
int WriteFlash16(uint32_t addrStart, uint16_t *pData, uint8_t dataLen)
{   
    uint32_t i = 0;
    uint32_t eraseCounter = 0x00;                                                               // record how many pages to erase
   // uint32_t address = 0x00;                                                                    // Record the address written
    uint32_t numberOfPage = 0x00;                                                               // record how many pages are written
    FLASH_Status flashStatus = FLASH_COMPLETE;                                                  // Record the result of each erasure
    
   // address = addrStart;
    
    FLASH_Unlock();                                                                             // unlock
    numberOfPage = (WRITE_END_ADDR - addrStart) / FLASH_PAGE_SIZE;                                // Calculate how many pages to erase
	numberOfPage =1;
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);  // Clear all flags
    
    // Erase by page
    for(eraseCounter = 0; (eraseCounter < numberOfPage) && (flashStatus == FLASH_COMPLETE); eraseCounter++)
    {
        flashStatus = FLASH_ErasePage(addrStart + (FLASH_PAGE_SIZE * eraseCounter));
	}
    
    for(i = 0; (i < dataLen)&&(flashStatus == FLASH_COMPLETE); i++)
    {
        flashStatus = FLASH_ProgramHalfWord(addrStart, pData[i]);                                     // Write a word (32 bits) of data into the specified address
        addrStart = addrStart + 2;                                                                  // The address is offset by 4 bytes 
    }
		
    //FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data)
    FLASH_Lock();                                                                               // relock
    
    if(flashStatus == FLASH_COMPLETE)
    {
        return 1;
    }
    return 0;
}*/
/**
   @brief internal Flash read
   @param address -[in] read address
   @param pData -[in&out] points to the data to be manipulated
   @param dataLen -[in] data length
   @return true-success; false-failure
*/
int Internal_ReadFlash(uint32_t addrStart, uint32_t *pData, uint32_t dataLen)
{  uint32_t address = 0x00;
	 address =  EEPROM_BASE + address;
	
    uint32_t i = 0;
  
    
    for(i = 0; i < dataLen; i++)
    {
        pData[i] = (*(__IO uint32_t*) address);      // Read a word of data at the specified address
        address += 4;                                // The address is offset by 4 bytes        
    }
    
    return 1;
}
int Internal_ReadFlash8(uint32_t addrStart, uint8_t *pData, uint8_t dataLen)
{  uint32_t address = 0x00;
	 address =  EEPROM_BASE + address;
	 
    uint32_t i = 0;
     
    for(i = 0; i < dataLen; i++)
    {
        pData[i] = (*(__IO uint8_t*) address);                                                 // Read a word of data at the specified address
        address += 1;                                                                           // The address is offset by 4 bytes        
    }
    
    return 1;
}



/*

int main(void)
{    
    u32 in_data[5]={11,22,33,44,55};//Data to be written
    u32 out_data[5];//Read and store
    int i;
    u8 STATUS=0;
    USART1_Config();//Serial port 1 configuration
    GPIO_Configuration();//GPIO configuration, used to light up the led
    STATUS=Internal_WriteFlash(0x08001800,in_data,5);
    Delay(0x02FFFF);
    if(STATUS)
    {
            GPIO_SetBits(GPIOD, GPIO_Pin_13);//Light up led1
            Internal_ReadFlash(0x08001800,out_data,5);
            printf("\r\n The Five Data Is : \r\n");
            for(i=0;i<5;i++)
            {
                    printf("\r %d \r",out_data[i]);
            }
    }
    while(1);
}



*/
void testread(void)
{
 uint32_t out_data[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//Read and store
  uint8_t out_data8[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//Read and store
	uint32_t temp_ts[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};	 
	uint8_t send[8];
 	uint8_t STATUS;
	
	
  STATUS= 	Flash_Write(0x00,temp_ts,8);	//0x0801FC00
	
   delay(1000);
 if(STATUS)
    {
 
 			
			Flash_Read(0,out_data,8);
			

		}


}

