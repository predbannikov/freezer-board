#include "stm32f1xx.h"
#include "encoder.h"
#include "uart.h"



uint16_t encoder_read_previous;

int main(void)
{
	uart2_init();
	encoder_init(10);
	printf("hello from stm32\r\n");
	while(1)
	{

		if(encoder_read()!=encoder_read_previous)
		{
			encoder_read_previous=encoder_read();
			printf("Encoder counts = %d\r\n",encoder_read());

		}

	}
}
