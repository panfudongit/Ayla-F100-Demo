

#include <string.h>
#include <ayla/mcu_platform.h>
#include <mcu_io.h>
#include <stm32.h>
#include <func.h>


int main(int argc, char **argv)
{
	uint8_t cnt;
	
	USART_init();
	
	mcu_io_init();

	spi_platform_init();

	stm32_init();
	
	printd("main start ....... enter fordd");
	for (;;) {
		for(cnt = 0; cnt < 10; cnt++)
		{
			spi_platform_out('0' + cnt);
			delay_ms(10);
		}
		delay_ms(800);
	}
}
