#include "ds18b20.h"

// Reset DS18B20
void DS18B20_Rst(void)
{
    DS18B20_IO_OUT(); //SET PA0 OUTPUT
    DS18B20_DQ_OUT_L(); // pull down DQ
    HAL_Delay(1);    // Pull down 1ms
    DS18B20_DQ_OUT_H(); //DQ=1
    HAL_Delay(1);     //15US
}

// Waiting for the response from DS18B20
// Return 1: The existence of DS18B20 is not detected
// Return 0: exist
u8 DS18B20_Check(void)
{
    u8 retry=0;
    DS18B20_IO_IN();//SET PA0 INPUT
    while (DS18B20_DQ_IN&&retry<200)
    {
        retry++;
        HAL_Delay(1);
    };
    if(retry>=200)return 1;
    else retry=0;
    while (!DS18B20_DQ_IN&&retry<240)
    {
        retry++;
        HAL_Delay(1);
    };
    if(retry>=240)return 1;
    return 0;
}
// Read a bit from DS18B20
// Return value: 1/0
u8 DS18B20_Read_Bit(void) 			 // read one bit
{
    u8 data;
    DS18B20_IO_OUT();//SET PA0 OUTPUT
    DS18B20_DQ_OUT_L();
    HAL_Delay(1);
    DS18B20_DQ_OUT_H();
    DS18B20_IO_IN();//SET PA0 INPUT
    HAL_Delay(1);
    if(DS18B20_DQ_IN)data=1;
    else data=0;
    HAL_Delay(1);
    return data;
}
// Read one byte from DS18B20
// Return value: read data
u8 DS18B20_Read_Byte(void)    // read one byte
{
    u8 i,j,dat;
    dat=0;
    for (i=1; i<=8; i++)
    {
        j=DS18B20_Read_Bit();
        dat=(j<<7)|(dat>>1);
    }
    return dat;
}
// Write a byte to DS18B20
// dat: bytes to be written
void DS18B20_Write_Byte(u8 dat)
{
    u8 j;
    u8 testb;
    DS18B20_IO_OUT();//SET PA0 OUTPUT;
    for (j=1; j<=8; j++)
    {
        testb=dat&0x01;
        dat=dat>>1;
        if (testb)
        {
            DS18B20_DQ_OUT_L();// Write 1
            HAL_Delay(1);
            DS18B20_DQ_OUT_H();
            HAL_Delay(1);
        }
        else
        {
            DS18B20_DQ_OUT_L();// Write 0
            HAL_Delay(1);
            DS18B20_DQ_OUT_H();
            HAL_Delay(1);
        }
    }
}
// Start temperature conversion
void DS18B20_Start(void)// ds1820 start convert
{
    DS18B20_Rst();
    DS18B20_Check();
    DS18B20_Write_Byte(0xcc);// skip rom
    DS18B20_Write_Byte(0x44);// convert
}
// Initialize the IO port DQ of DS18B20 and detect the existence of DS
// Return 1: Does not exist
// Return 0: exist
u8 DS18B20_Init(void)
{
    DS18B20_Rst();
    return DS18B20_Check();
}
// Get the temperature value from ds18b20
// Accuracy: 0.1C
// Return value: temperature value (-550 ~ 1250)
float DS18B20_Get_Temp(void)
{
    u8 temp;
    u8 TL,TH;
    short tem;
    float ret;
    DS18B20_Start ();                    // ds1820 start convert
    DS18B20_Rst();
    DS18B20_Check();
    DS18B20_Write_Byte(0xcc);// skip rom
    DS18B20_Write_Byte(0xbe);// convert
    TL=DS18B20_Read_Byte(); // LSB
    TH=DS18B20_Read_Byte(); // MSB

    if(TH>7)
    {
        TH=~TH;
        TL=~TL;
        temp=0;// The temperature is negative
    } else temp=1;// The temperature is positive
    tem=TH; // Get the high eight
    tem<<=8;
    tem+=TL;// Get the bottom eight
// tem = (float) tem * 0.625; // Convert
// if (temp) return tem; // Return temperature value
//    else return -tem;
    ret = (float)tem * 0.625;
    return temp == 1 ? ret : -ret;
}


