#include "stm32f10x.h"                  // Device header

#ifndef DISPLAY_SSD1306_I2C_H
#define DISPLAY_SSD1306_I2C_H



// PB6  PB7 **********************
#define I2C_PORT				GPIOB
#define I2C_SCL_PIN				7
#define I2C_SDA_PIN				6
// Кроме этого пины указаны в I2C_Init() см. ниже

#define SSD1306_ADDR			0x78

//#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH			128
//#endif
//#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT			64
//#endif

// Следующая константа определяет частоту SCL. Должна быть чётная.  При 6  SCL=330 кГц
#define I2CSWM_DELAY			6

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define I2CSWM_SETSCL() 	(I2C_PORT->BSRR |= 1<<I2C_SCL_PIN)
#define I2CSWM_CLEARSCL() 	(I2C_PORT->BSRR |= 1<<16<<I2C_SCL_PIN)
#define I2CSWM_GETSCL() 	((I2C_PORT->IDR & (1<<I2C_SCL_PIN)) == 0 ? 0 : 1 )

#define I2CSWM_SETSDA() 	(I2C_PORT->BSRR |= 1<<I2C_SDA_PIN)
#define I2CSWM_CLEARSDA() 	(I2C_PORT->BSRR |= 1<<16<<I2C_SDA_PIN)
#define I2CSWM_GETSDA() 	((I2C_PORT->IDR & (1<<I2C_SDA_PIN)) == 0 ? 0 : 1 )

#define I2CSWM_DIRECTION_TX 	0
#define I2CSWM_DIRECTION_RX 	1

#define I2CSWM_NACK 			1
#define I2CSWM_ACK				0



#define SSD1306_COMMAND			0x00
#define SSD1306_DATA			0xC0
#define SSD1306_DATA_CONTINUE	0x40

#define RST_NOT_IN_USE	255

// SSD1306 Commandset
// ------------------
// Fundamental Commands
#define SSD1306_SET_CONTRAST_CONTROL					0x81
#define SSD1306_DISPLAY_ALL_ON_RESUME					0xA4
#define SSD1306_DISPLAY_ALL_ON							0xA5
#define SSD1306_NORMAL_DISPLAY							0xA6
#define SSD1306_INVERT_DISPLAY							0xA7
#define SSD1306_DISPLAY_OFF								0xAE
#define SSD1306_DISPLAY_ON								0xAF
#define SSD1306_NOP										0xE3
// Scrolling Commands
#define SSD1306_HORIZONTAL_SCROLL_RIGHT					0x26
#define SSD1306_HORIZONTAL_SCROLL_LEFT					0x27
#define SSD1306_HORIZONTAL_SCROLL_VERTICAL_AND_RIGHT	0x29
#define SSD1306_HORIZONTAL_SCROLL_VERTICAL_AND_LEFT		0x2A
#define SSD1306_DEACTIVATE_SCROLL						0x2E
#define SSD1306_ACTIVATE_SCROLL							0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA				0xA3
// Addressing Setting Commands
#define SSD1306_SET_LOWER_COLUMN						0x00
#define SSD1306_SET_HIGHER_COLUMN						0x10
#define SSD1306_MEMORY_ADDR_MODE						0x20
#define SSD1306_SET_COLUMN_ADDR							0x21
#define SSD1306_SET_PAGE_ADDR							0x22
// Hardware Configuration Commands
#define SSD1306_SET_START_LINE							0x40
#define SSD1306_SET_SEGMENT_REMAP						0xA0
#define SSD1306_SET_MULTIPLEX_RATIO						0xA8
#define SSD1306_COM_SCAN_DIR_INC						0xC0
#define SSD1306_COM_SCAN_DIR_DEC						0xC8
#define SSD1306_SET_DISPLAY_OFFSET						0xD3
#define SSD1306_SET_COM_PINS							0xDA
#define SSD1306_CHARGE_PUMP								0x8D
// Timing & Driving Scheme Setting Commands
#define SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO				0xD5
#define SSD1306_SET_PRECHARGE_PERIOD					0xD9
#define SSD1306_SET_VCOM_DESELECT						0xDB

void SendOneCommand(uint8_t command);
//unsigned long rand(void);
//int abs(int i);


void i2c2ini();
void i2c1_scan_bus(void);
void I2C_Init_PB6PB7(void);
inline static void I2C_Start();
inline static void I2C_Stop();
inline static uint_fast8_t I2C_Write_Byte(uint8_t b);
uint8_t SendCommand(uint8_t command, uint8_t params, uint8_t param1, uint8_t param2);
void SendOneCommand(uint8_t command);
void UpdateScreen(void);
void UpdateScreenBottom(void);
void SetPixel(uint8_t x, uint8_t y, uint8_t color);
void DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
int16_t SquareDist(int16_t x, int16_t y);
void DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t color);
void PutChar(uint8_t x, uint8_t y, uint8_t symbol, uint8_t color);
void PutString(uint8_t x, uint8_t y, volatile uint8_t *Mess, uint8_t color);
void ScreenSaver(void);










#endif 