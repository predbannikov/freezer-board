//#include "main.h"
#include "stm32f10x.h"                  // Device header
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"               // Нужен для NVIC
 
// Максимальное значение уровня громкости
#define VOLUME_MAX_VAL  51
 
// Текущий уровень громкости
volatile uint8_t Vol = 10;
 
 
 
int init_enc(void) {
    // Включаем тактирование модуля ввода-вывода 'B'
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
     
    // Переменные (структуры) для инициализации
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
 
    // Прерывания - это альтернативная функция порта, включаем
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 
    // Настройка пинов A0 и A1
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;      // Вход с подтяжкой вверх
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
 
    // Добавляем вектор прерывания
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    // Устанавливаем приоритет
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x08;   // 0x00 - 0x0F
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x08;
    // Разрешаем прерывание
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
 
    // PA0 подключен к EXTI_Line0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    // Разрешаем прерывание
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    // По спаду
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStruct);
     
 
    
}
 
 
 
// Обработчик прерывания для энкодера по спаду PB0
void EXTI0_IRQHandler(void) {
    uint32_t nCount = 3;
 
    // Убеждаемся, что флаг прерывания установлен
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 1) {
            // Кто-то крутит ручку по часовой стрелке
            if(Vol < VOLUME_MAX_VAL) {
                Vol++;
                }
            }
        else {
            // Ручку крутят против часовой стрелки
            if(Vol > 0) {
                Vol--;
                }
            }
 
      //  for(nCount *= 100000; nCount; nCount--);
 
				#ifdef SERVIS_DEBUG	 
//	encoder_real=enc_read(encoder_real);
	 				 displey(105, 50,nVol,7)	;
					 displey(105, 40,ex,7)	;
					SSD1306_UpdateScreen();						
#endif 
		
						
						
						
        // Сбрасываем флаг прерывания
        EXTI_ClearITPendingBit(EXTI_Line0);
        }
}