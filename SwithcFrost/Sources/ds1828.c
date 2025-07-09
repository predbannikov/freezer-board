#include <stm32f10x.h>
#include <stm32f10x_usart.h>



#define OW_0   0x00
#define OW_1   0xff
#define OW_R_1   0xff

#define OW_OK         1
#define OW_ERROR      2
#define OW_NO_DEVICE           3

#define OW_NO_READ      0xff

uint8_t command_buffer[8]; // Буффер команд
uint8_t read_buffer[8]; // Буффер для чтения
uint8_t temperature_buffer; //Сюда записываем данные с датчика

USART_InitTypeDef huart2;

/*---------------------------------------------------------------------------------------------*/

uint8_t Send(uint8_t *command, uint16_t cLen, uint8_t data, uint8_t readStart)
{
  if (Reset() == OW_NO_DEVICE)
  {
    return OW_NO_DEVICE;
  }
  while (cLen > 0)
  {
    toBits(*command, command_buffer);
    command++;
    cLen--;
    
    /*Посылаем команду, после чего начинаем читать данные*/
    Send_Command();
    
    /*Если прочитанные данные кому-то нужны - выкинем их в буфер*/
    if (readStart == 0)
    {
      data = toByte(read_buffer);
    }
    else
    {
      if (readStart != OW_NO_READ)
      {
        readStart--;
      }
    }
  }
  return data;
}
/*Осуществляет сброс и проверку на наличие устройств на шине*/
uint8_t Reset()
{
  uint8_t presence = 1;
  uint32_t cnt = 0;
  
  HAL_UART_DeInit(&huart2);
  
  huart2.     Instance = USART2;
  huart2.USART_BaudRate = 9600;
  huart2.USART_WordLength = UART_WORDLENGTH_8B;
  huart2.USART_StopBits =   UART_STOPBITS_1;
  huart2.USART_Parity     = UART_PARITY_NONE;
  huart2.USART_Mode       = UART_MODE_TX_RX;
  huart2.USART_HardwareFlowControl = UART_HWCONTROL_NONE;
 // huart2.  .Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

  __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
  USART2->DR = 0xf0;
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) cnt++;  
  presence = USART2->DR;
  
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
  
  if (presence != 0xf0)
  {
    return OW_OK;
  }
  return OW_NO_DEVICE;
}

/*Функция для отправки и приёма данных*/
void Send_Command(void)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TXE);
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
    USART2->DR = command_buffer[i];    
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == RESET);
    if (toByte(command_buffer) == 0xff)
    {
      read_buffer[i] = USART2->DR;
    }
  }
}
//------------------------------
  Send ("\xcc\x44", 2, 0, OW_NO_READ);
    HAL_Delay(750);
    temperature_buffer = Send("\xcc\xbe\xff", 3, temperature_buffer, 2);
    printf("tempreture = %d.%d\n", temperature_buffer/2, (temperature_buffer & 1)*5);

//-------------------------------






