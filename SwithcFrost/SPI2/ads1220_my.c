

#include "ads1220_my.h"
unsigned char ADS1220ReceiveByte(void);

void Delay_us (uint32_t __IO us) //Функция задержки в микросекундах us
{
//us *=(SystemCoreClock/1000000)/5;
	//���� ��������� 72 mhz
	us *=(SystemCoreClock/72000000); //1 uS
	
	while(us--);
}



void ADS1220AssertCS( int fAssert)
{
   if (!fAssert)
   {
		CS_LO; 
	//	Delay_us(250);
   }
   else
   {
	//   Delay_us(250);
		CS_HI;
   }
}
void ADS1220ReadRegister(int StartAddress, int NumRegs, unsigned * pData)
{
   int i;

	// assert CS to start transfer
//	ADS1220AssertCS(1);
   
	// send the command byte
	SPI2_Write(ADS1220_CMD_RREG | (((StartAddress<<2) & 0x0c) |((NumRegs-1)&0x03)));
//	ADS1220SendByte(ADS1220_CMD_RREG | (((StartAddress<<2) & 0x0c) |((NumRegs-1)&0x03)));
   
	// get the register content
	for (i=0; i< NumRegs; i++)
	{
		*pData++ =SPI2_Read();// ADS1220ReceiveByte();
	}
   
	// de-assert CS
//	ADS1220AssertCS(0);
	
	return;
}

void ADS1220WriteRegister(int StartAddress, int NumRegs, unsigned * pData)
{
	int i;
   
	// assert CS to start transfer
	//ADS1220AssertCS(1);
   
	// send the command byte
	SPI2_Write(ADS1220_CMD_WREG | (((StartAddress<<2) & 0x0c) |((NumRegs-1)&0x03)));
   
    // send the data bytes
	for (i=0; i< NumRegs; i++)
	{
		SPI_Send_Byte(*pData++);
	}
   
	// de-assert CS
	//ADS1220AssertCS(0);
   
	return;
}

void ADS1220Config(void)
{
	int Temp;
	//ADS1220AssertCS(1);
//	CS_LO;
	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
  //   	CS_HI;delay(10);	CS_LO;// delay(50);
	// clear prev value;
   	Temp &= 0x0f;
   	Temp |= ADS1220_MUX_0_G;
//   CS_HI;delay(5);	CS_LO;
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp);
// 	CS_HI;delay(5);	CS_LO; 
	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
 // 	CS_HI;delay(5);	CS_LO;  
	// clear prev DataRate code;
	Temp &= 0x1f;
	Temp |= (ADS1220_DR_600 + ADS1220_CC);		// Set default start mode to 600sps and continuous conversions
   
	// write the register value containing the new value back to the ADS
	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &Temp);
//	 	CS_HI;	CS_LO; delay(50);
// //	ADS1220AssertCS(0);
}

void ADS1220Config_LoadCeal(void)
{
	int Temp;
 
	  ADS1220ReadRegister(ADS1220_0_REGISTER, 0x03, &Temp);
DelayTimerUs(100);
   	Temp &= 0x00;
	
   	Temp |= ADS1220_MUX_1_2 + ADS1220_GAIN_128;
   DelayTimerUs(100);
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
DelayTimerUs(100);
	
	  ADS1220ReadRegister(ADS1220_0_REGISTER, 0x03, &Temp);
DelayTimerUs(100);
	
	
	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x02, &Temp);
	DelayTimerUs(100);
  Temp &= 0x00;
	Temp |= (ADS1220_DR_600 + ADS1220_CC);		// Set default start mode to 600sps and continuous conversions
 	// write the register value containing the new value back to the ADS
	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	
DelayTimerUs(100);	
	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x02, &Temp);
	DelayTimerUs(100);
	Temp &= 0x00;
	Temp |= 0x98;		 //External reference (REFP1, REFN1), simultaneous 50-Hz and 60-Hz rejection, PSW = 1
//	SPI_Send_Byte(ADS1220_CMD_WREG);
  ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &Temp);

DelayTimerUs(100);
   ADS1220ReadRegister(ADS1220_3_REGISTER, 0x02, &Temp);
	 Delay_us(100);
	Temp &= 0x00;  //no IDACs used
//		SPI_Send_Byte(ADS1220_CMD_WREG); 
  ADS1220WriteRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	
 /*		*/
 //Delay_us(250);
//CS_HI;//ADS1220AssertCS(0);	
	
}

void ADS1220Config_LoadCeal1(void)
{
	int Temp;
 CS_LO; 
	  ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
Delay_us(10000);
   	Temp &= 0x00;
	
   	Temp |= ADS1220_MUX_1_2 + ADS1220_GAIN_128;
   
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
Delay_us(7000);CS_HI;
/*	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	Delay_us(10);
  Temp &= 0x00;
	Temp |= (ADS1220_DR_600 + ADS1220_CC);		// Set default start mode to 600sps and continuous conversions
 	// write the register value containing the new value back to the ADS
	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	
Delay_us(10);	
	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
	Delay_us(10);
	Temp &= 0x00;
	Temp |= 0x98;		 //External reference (REFP1, REFN1), simultaneous 50-Hz and 60-Hz rejection, PSW = 1
//	SPI_Send_Byte(ADS1220_CMD_WREG);
  ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &Temp);

Delay_us(10);
   ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	 Delay_us(10);
	Temp &= 0x00;  //no IDACs used
//		SPI_Send_Byte(ADS1220_CMD_WREG); 
  ADS1220WriteRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	*/
 
 //Delay_us(250);
//CS_HI;//ADS1220AssertCS(0);	
	
}
void ADS1220Config_LoadCeal2(void)
{
	int Temp;
 CS_LO;
	/*  ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
Delay_us(10);
   	Temp &= 0x00;
	
   	Temp |= ADS1220_MUX_1_2 + ADS1220_GAIN_128;
   
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
Delay_us(10);*/
	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	Delay_us(100);
  Temp &= 0x00;
	Temp |= (ADS1220_DR_600 + ADS1220_CC);		// Set default start mode to 600sps and continuous conversions
 	// write the register value containing the new value back to the ADS
	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	
Delay_us(1000);	CS_HI;
/*	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
	Delay_us(10);
	Temp &= 0x00;
	Temp |= 0x98;		 //External reference (REFP1, REFN1), simultaneous 50-Hz and 60-Hz rejection, PSW = 1
//	SPI_Send_Byte(ADS1220_CMD_WREG);
  ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &Temp);

Delay_us(10);
   ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	 Delay_us(10);
	Temp &= 0x00;  //no IDACs used
//		SPI_Send_Byte(ADS1220_CMD_WREG); 
  ADS1220WriteRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	
 */

	
}
void ADS1220Config_LoadCeal3(void)
{
	int Temp;
 CS_LO;
/*	  ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
Delay_us(10);
   	Temp &= 0x00;
	
   	Temp |= ADS1220_MUX_1_2 + ADS1220_GAIN_128;
   
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
Delay_us(10);
	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	Delay_us(10);
  Temp &= 0x00;
	Temp |= (ADS1220_DR_600 + ADS1220_CC);		// Set default start mode to 600sps and continuous conversions
 	// write the register value containing the new value back to the ADS
	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	*/
//Delay_us(10);	
	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
	Delay_us(100);
	Temp &= 0x00;
	Temp |= 0x98;		 //External reference (REFP1, REFN1), simultaneous 50-Hz and 60-Hz rejection, PSW = 1
//	SPI_Send_Byte(ADS1220_CMD_WREG);
  ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &Temp);

Delay_us(1000);CS_HI;
/*   ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	 Delay_us(10);
	Temp &= 0x00;  //no IDACs used
//		SPI_Send_Byte(ADS1220_CMD_WREG); 
  ADS1220WriteRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	*/
	
}
void ADS1220Config_LoadCeal4(void)
{
	int Temp;
 /*
	  ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
Delay_us(10);
   	Temp &= 0x00;
	
   	Temp |= ADS1220_MUX_1_2 + ADS1220_GAIN_128;
   
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
Delay_us(10);
	ADS1220ReadRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	Delay_us(10);
  Temp &= 0x00;
	Temp |= (ADS1220_DR_600 + ADS1220_CC);		// Set default start mode to 600sps and continuous conversions
 	// write the register value containing the new value back to the ADS
	ADS1220WriteRegister(ADS1220_1_REGISTER, 0x01, &Temp);
	
Delay_us(10);	
	ADS1220ReadRegister(ADS1220_2_REGISTER, 0x01, &Temp);
	Delay_us(10);
	Temp &= 0x00;
	Temp |= 0x98;		 //External reference (REFP1, REFN1), simultaneous 50-Hz and 60-Hz rejection, PSW = 1
//	SPI_Send_Byte(ADS1220_CMD_WREG);
  ADS1220WriteRegister(ADS1220_2_REGISTER, 0x01, &Temp);
*/CS_LO;
//
   ADS1220ReadRegister(ADS1220_3_REGISTER, 0x01, &Temp);
	 Delay_us(100);
	Temp &= 0x00;  //no IDACs used
//		SPI_Send_Byte(ADS1220_CMD_WREG); 
  ADS1220WriteRegister(ADS1220_3_REGISTER, 0x01, &Temp);
Delay_us(1000);	CS_HI;
	
}
void ADS122Start(void)
{
	int Temp;
//CS_LO;//	ADS1220AssertCS(1);	
//	Delay_us(250);
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
   
	// clear prev value;
   	Temp &= 0x00;
   	Temp |= ADS1220_CMD_SYNC;
 //  SPI_Send_Byte(ADS1220_CMD_SYNC);
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
//Delay_us(250);
//CS_HI;//ADS1220AssertCS(0);
	
	
}
void ADS122ReadReg0(void)
{
	int Temp;
//CS_LO;//	ADS1220AssertCS(1);	
//	Delay_us(250);
//	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
   
	// clear prev value;
   	Temp &= 0x00;
   	Temp |= ADS1220_CMD_RDATA;
  //   	SPI_Send_Byte(ADS1220_CMD_RDATA);
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
		Temp &= 0x00;
	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
	//Delay_us(250);
//CS_HI;//ADS1220AssertCS(0);	
	
	
}
void ADS122ReSet(void)
{
	int Temp;
//	ADS1220AssertCS(1);		
	ADS1220ReadRegister(ADS1220_0_REGISTER, 0x01, &Temp);
   
	// clear prev value;
   	Temp &= 0x00;
   	Temp |= ADS1220_CMD_RESET;
  //   	SPI_Send_Byte(ADS1220_CMD_RDATA);
   	// write the register value containing the new value back to the ADS
   	ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &Temp); //0x3e AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled
//ADS1220AssertCS(0);	
}
	
	
long ADS1220ReadData(void)
{
   long Data;
   /* assert CS to start transfer */
   ADS1220AssertCS(1);
   /* send the command byte */
  // ADS1220SendByte(ADS1220_CMD_RDATA);
	SPI2_Write(ADS1220_CMD_RDATA); 
   /* get the conversion result */

   Data = SPI2_Read();//ADS1220ReceiveByte();//
   Data = (Data << 8) |SPI2_Read();// ADS1220ReceiveByte();//
   /* sign extend data */
   if (Data & 0x8000)
      Data |= 0xffff0000;

   /* de-assert CS */
   ADS1220AssertCS(0);
   return Data;
}

/*unsigned char ADS1220ReceiveByte(void)
{
   unsigned char Result = 0, i=0;
   
  for(i=0;i<8;i++)
  {  
	     Result<<=1;
     CS_LO; //ADC_CLK=0
     Delay_us(5);

     //Delay_us(5);
     CS_HI; //ADC_CLK=1
     Delay_us(3);
     if (MISO) Result++;
     Delay_us(2);

  }
  CS_LO; //ADC_CLK=0
	return Result;
}*/
/*void ADS1220SendByte(unsigned char Byte)
{
unsigned char Result = 0x01, i=0, flg=0;
MOSI_LO;
Delay_us(1);
for(i=0;i<8;i++)
  {  

     SCK_LO; //ADC_CLK=0
     Delay_us(4);
     if (flg) MOSI_LO;
     Delay_us(1);
     SCK_HI; //ADC_CLK=1
     Delay_us(1);
     if (Byte&Result){ MOSI_HI; flg=1; }
	 else             MOSI_LO;
     Delay_us(4);
     Result<<=1;
  }
SCK_LO; //ADC_CLK=0
}
*/
