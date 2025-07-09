/*   Библиотека для  работы  с oled дисплеем    */
/*   на контроллере SSD1306 по интерфейсу I2C   */
/*   Максим К.  http://ad-res.ru/controllers/   */



#include "stm32f10x_gpio.h"
#include "Font_Verter_10x15.h"
#include "Display_SSD1306_I2C.h"

volatile uint8_t sMessage[16] = {0};
volatile uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8] = {0};

void i2c2ini()
{
	RCC->APB1ENR  =RCC_APB1ENR_I2C2EN; // enable clock for I2C2
//	RCC->APB2ENR |=RCC_APB1ENR_I2C2EN;
/*Enable clock access to GPIOB*/
	RCC->APB2ENR|=RCC_APB2Periph_GPIOB;
	RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;
	/*Set PB6 to output 50MHz*/
	GPIOB->CRL|=GPIO_CRL_MODE6;
	/*Set PB6 to ALternate Open drain*/
	GPIOB->CRL|=GPIO_CRL_CNF6;

	/*Set PB7 to output 50MHz*/
	GPIOB->CRL|=GPIO_CRL_MODE7;
	/*Set PB7 to ALternate Open drain*/
	GPIOB->CRL|=GPIO_CRL_CNF7;
	
		/*Enable clock access to alternate function of the pins*/
	RCC->APB2ENR|=RCC_APB2ENR_AFIOEN;
	
		/*Tell the peripheral that the clock is 8MHz*/
	I2C2->CR2&=~(I2C_CR2_FREQ);
//	I2C2->CR2|=(8<<I2C_CR2_FREQ_Pos);
	//указываем частоту тактирования модуля
	I2C2->CR2 &= ~I2C_CR2_FREQ;
	I2C2->CR2 |= 15; // Fclk1=168/4=42MHz 
	/*Set the rise time
	I2C2->TRISE=9;	
	
	I2C2->CCR|=0x28;
	
	I2C2->CR1|=I2C_CR1_PE;*/
	
  //конфигурируем I2C, standart mode, 100 KHz duty cycle 1/2	
	I2C2->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY);
        //задаем частоту работы модуля SCL по формуле 10 000nS/(2* TPCLK1) 
	I2C2->CCR |= 208; //10 000ns/48ns = 208
	
	//Standart_Mode = 1000nS, Fast_Mode = 300nS, 1/42MHz = 24nS
	I2C2->TRISE = 5; //(1000nS/24nS)+1

        //включаем модуль
	I2C2->CR1 |= I2C_CR1_PE;
}
void I2C2_Init(void)
{
//тактирование RCC->APB1ENR  =RCC_APB1ENR_I2C2EN; // enable clock for I2C2
RCC->APB1ENR=RCC_APB1ENR_I2C2EN; // enable clock for I2C2
//RCC->APB2ENR |=RCC_APB1ENR_I2C2EN;
RCC->APB2ENR |=RCC_APB2ENR_IOPBEN;
RCC->APB2ENR |=RCC_APB2ENR_AFIOEN;
//выставление нулей
GPIOB->CRL &=~GPIO_CRL_MODE6;
GPIOB->CRL &=~GPIO_CRL_CNF6;
GPIOB->CRL &=~GPIO_CRL_MODE7;
GPIOB->CRL &=~GPIO_CRL_CNF7;
//настройка
GPIOB->CRL |= GPIO_CRL_CNF6; //alternate function open-drain
GPIOB->CRL |= GPIO_CRL_CNF7; //alternate function open-drain
GPIOB->CRL |=GPIO_CRL_MODE6; //50MHz
GPIOB->CRL |=GPIO_CRL_MODE7; //50MHz
//I2C initialization
I2C2->CR2 &=~I2C_CR2_FREQ;
I2C2->CR2 |=I2C_CR2_FREQ_1; //standart mode 2MHz
I2C2->CCR &= ~I2C_CCR_CCR;
I2C2->CCR |= 80; //установка предделителя
I2C2->TRISE = 9; //rise time 1000/(1/(8*10^6))+1=9
}

void i2c1_scan_bus(void)
{   int c=0;     int a=0;
         for (uint8_t i=0;i<128;i++)
   {
            I2C2->CR1 |= I2C_CR1_START;
            while(!(I2C1->SR1 & I2C_SR1_SB))
						{
						c++;
							if(c>10000) break; // no answer from device
						
						}
            I2C2->DR=(i<<1|0);
            while(!(I2C2->SR1)|!(I2C2->SR2))
							{
						c++;
							if(c>10000) break; // no answer from device
						
						}
            I2C2->CR1 |= I2C_CR1_STOP;
            Delay_us(100);//minium wait time is 40 uS, but for sure, leave it 100 uS
            a=(I2C2->SR1&I2C_SR1_ADDR);
            if (a==2)
         {
                printf("Found I2C device at adress 0x%X (hexadecimal), or %d (decimal)\n\r",i,i);
         }
     }
}
inline static void I2C_Delay( int n ) { while( n-- ); }
void Delay(volatile uint32_t nCount) {
	for(nCount *= 100000; nCount; nCount--);
}




void I2C_Init_PB6PB7(void) {
/*	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_PORT, &gpio);

    I2CSWM_SETSDA();
    I2CSWM_SETSCL();
*/
	SendOneCommand (0xA8);		// 64 строки
	SendOneCommand (0x3F);
	SendOneCommand (0x20);		// Режим адрессации
	SendOneCommand (0x00);
	SendOneCommand (0xD3);		// Set Display Offset
	SendOneCommand (0);
	SendOneCommand (0xA1);		// Ремап столбцов (127...0)
	SendOneCommand (0xC8);		// Ремап строк (63...0)
	SendOneCommand (0x81);		// Контрастность
	SendOneCommand (0x60);
	SendOneCommand (0x8D);		// ON display
	SendOneCommand (0x14);
	SendOneCommand (0xAF);
	SendOneCommand (0x21);		// 0 столбец стартовый, 127 последний
	SendOneCommand (0);
	SendOneCommand (127);
	SendOneCommand (0x22);		// 0 строка стартовая, 7 последняя
	SendOneCommand (0);
	SendOneCommand (7);

	I2C_Delay(2500);
}



inline static void I2C_Start() {
    I2CSWM_SETSDA();
    I2CSWM_SETSCL();
    I2C_Delay(I2CSWM_DELAY);
    I2CSWM_CLEARSDA();
    I2C_Delay(I2CSWM_DELAY);
    I2CSWM_CLEARSCL();
    I2C_Delay(I2CSWM_DELAY);
}




inline static void I2C_Stop() {
    I2CSWM_CLEARSCL();
    I2CSWM_CLEARSDA();
    I2C_Delay(I2CSWM_DELAY);
    I2CSWM_SETSCL();
    I2C_Delay(I2CSWM_DELAY);
    I2CSWM_SETSDA();
    I2C_Delay(I2CSWM_DELAY);
}




inline static uint_fast8_t I2C_Write_Byte(uint8_t b) {
    uint_fast8_t db = 0x80, ack;

    for(uint_fast8_t i = 0; i < 8; i++) {
		I2C_Delay(I2CSWM_DELAY / 2);
        if( b & db ) {
            I2CSWM_SETSDA();
			}
		else {
            I2CSWM_CLEARSDA();
			}
		I2CSWM_SETSCL();
		I2C_Delay(I2CSWM_DELAY);
		I2CSWM_CLEARSCL();
		I2C_Delay(I2CSWM_DELAY / 2);
        db >>= 1;
		}

    /* Получим ACK */
    I2CSWM_SETSDA(); 				// Отпускаем линию данных SDA наверх
	I2C_Delay(I2CSWM_DELAY / 2);
    I2CSWM_SETSCL(); 				// Выставляем высокий пульс
	I2C_Delay(I2CSWM_DELAY);
	ack = I2CSWM_GETSDA();
	I2CSWM_CLEARSCL();
	I2C_Delay(I2CSWM_DELAY / 2);

    return ack;
}





uint8_t SendCommand(uint8_t command, uint8_t params, uint8_t param1, uint8_t param2) {
	I2C_Start();
	// 0x78 = 01111000  младший бит - Rx/Tx, предпоследний - D/C, но его трогать не надо
	if(I2C_Write_Byte(SSD1306_ADDR)) 		return FALSE;
	// 0x80 = 10000000  старший - isCommand, следующий - D/C, младшие - нули, но выставлять старший не нужно
	if(I2C_Write_Byte(SSD1306_COMMAND)) 	return FALSE;
	if(I2C_Write_Byte(command)) 			return FALSE;
	if(params > 0) {
		if(I2C_Write_Byte(SSD1306_ADDR)) 	return FALSE;
		if(I2C_Write_Byte(SSD1306_COMMAND)) return FALSE;
		if(I2C_Write_Byte(param1))			return FALSE;
		}
	if(params > 1) {
		if(I2C_Write_Byte(SSD1306_ADDR)) 	return FALSE;
		if(I2C_Write_Byte(SSD1306_COMMAND)) return FALSE;
		if(I2C_Write_Byte(param2)) 			return FALSE;
		}
	I2C_Stop();

	return TRUE;
}



void SendOneCommand(uint8_t command) {
	SendCommand(command, 0, 0, 0);
}


void UpdateScreen(void) {
	int i, m;

	for (m = 0; m < 8; m++) {
		SendCommand((0xB0 + m), 0, 0, 0);
		SendCommand(0x00, 0, 0, 0);
		SendCommand(0x10, 0, 0, 0);

    	I2C_Start();
    	I2C_Write_Byte(SSD1306_ADDR);
    	I2C_Write_Byte(SSD1306_DATA_CONTINUE);
		for(i=0; i<128; i++) {
			I2C_Write_Byte(*(SSD1306_Buffer + SSD1306_WIDTH * m + i));
			}
    	I2C_Stop();
	}
}




void UpdateScreenBottom(void) {
	int i, m;

	for (m = 4; m < 8; m++) {
		SendCommand((0xB0 + m), 0, 0, 0);
		SendCommand(0x00, 0, 0, 0);
		SendCommand(0x10, 0, 0, 0);

    	I2C_Start();
    	I2C_Write_Byte(SSD1306_ADDR);
    	I2C_Write_Byte(SSD1306_DATA_CONTINUE);
		for(i=0; i<128; i++) {
			I2C_Write_Byte(*(SSD1306_Buffer + SSD1306_WIDTH * m + i));
			}
    	I2C_Stop();
	}
}




void SetPixel(uint8_t x, uint8_t y, uint8_t color) {
	uint8_t yy, the_byte;
	yy = y % (SSD1306_HEIGHT / 8);

	the_byte = *(SSD1306_Buffer + SSD1306_WIDTH * (y/8) + x);
	if(color)
		the_byte |=  (1 << yy);
	else
		the_byte &= ~(1 << yy);

	*(SSD1306_Buffer + SSD1306_WIDTH * (y/8) + x) = the_byte;
}



// Построение прямой по алгоритму Брезенхэма
void DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {
	const int deltaX = abs(x2 - x1);
	const int deltaY = abs(y2 - y1);
	const int signX = x1 < x2 ? 1 : -1;
	const int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY, error2;

	SetPixel(x2, y2, color);
	while(x1 != x2 || y1 != y2) {
		SetPixel(x1, y1, color);
		error2 = error * 2;
		if(error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
			}
		if(error2 < deltaX) {
			error += deltaX;
			y1 += signY;
			}
		}
}




// Вспомогательная функция для построения окружности
// Возвращает квадрат расстояния от (0, 0) до (x, y)
int16_t SquareDist(int16_t x, int16_t y) {
	return (x * x + y * y);
}



// Построение окружности по алгоритму Брезенхэма
void DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t color) {
	int16_t x = 0, y = r; // Текущая точка рисования
	int16_t move[3]; // Массив расстояний до 3 точек, в которые можно перейти из текущей

	while(y >= (r >> 1)) {
		SetPixel(x0 + x, y0 + y, color);
		SetPixel(x0 - x, y0 + y, color);
		SetPixel(x0 + x, y0 - y, color);
		SetPixel(x0 - x, y0 - y, color);
		SetPixel(x0 + y, y0 + x, color);
		SetPixel(x0 - y, y0 + x, color);
		SetPixel(x0 + y, y0 - x, color);
		SetPixel(x0 - y, y0 - x, color);

		move[0] = abs(SquareDist(x + 1, y) - r * r);
		move[1] = abs(SquareDist(x, y - 1) - r * r);
		move[2] = abs(SquareDist(x + 1, y - 1) - r * r);

		if(move[0] <= move[1] && move[0] <= move[2]) { x++; }
		else
		if(move[1] <= move[0] && move[1] <= move[2]) { y--; }
		else
		if(move[2] <= move[0] && move[2] <= move[1]) { x++; y--; }
		}
}




void PutChar(uint8_t x, uint8_t y, uint8_t symbol, uint8_t color) {
	uint8_t i, j, k, chl, h, fnt_count;

	h = 0; // Здесь h - флаг присутствия искомого символа в шрифте
	fnt_count = sizeof(fnt_Verter) / 21;
	for(i=0; i < fnt_count; i++) {
		if(fnt_Verter[i][0] == symbol) {
			h = i;
			break;
			}
		}
	if(!h) return;
	i = h;

	h = 1; // Теперь h - триггер: нижний ряд байт / верхний ряд байт
	for(j=0; j<20; j++) {
		chl = fnt_Verter[i][j+1];
		if(h) {
			for(k=0; k<8; k++) {
				SetPixel(x + (j/2), y + 7*h + k, (chl >> k) & 0x01);
				}
			h = 0;
			}
		else {
			for(k=1; k<8; k++) {
				SetPixel(x + (j/2), y + 7*h + k-1, (chl >> k) & 0x01);
				}
			h = 1;
			}
		}
}




void PutString(uint8_t x, uint8_t y, volatile uint8_t *Mess, uint8_t color) {
	uint8_t i, max, interval = 10;

	max = sizeof(sMessage);
	for(i=0; *(Mess + i) && i < max; i++) {
		PutChar(x + interval*i, y, *(Mess + i), color);
		}
}






void ScreenSaver(void) {
	uint8_t x, y, i, lighted = 0;

	x = rand() % 63;		// RND_min = 1.00 млрд.
	y = rand() % 31;		// RND_max = 9.99 млрд.

	// Генерация 0 или 1. Для вероятности 50/50 делитель д.б. = 1.1 млрд.  o_O
	i = rand()/2000000000;

	if(i) lighted++;
	else  lighted--;

	x = x << 1;
	y = y << 1;
	SetPixel(x,   y,   i);
	SetPixel(x+1, y,   i);
	SetPixel(x,   y+1, i);
	SetPixel(x+1, y+1, i);
	//sprintf(sMessage, "%d.", lighted);
	//PutString(0,  49, sMessage, 1);
	UpdateScreen();
}


