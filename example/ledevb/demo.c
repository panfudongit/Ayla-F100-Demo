

#include <string.h>
#include <ayla/mcu_platform.h>
#include <mcu_io.h>
#include <stm32.h>
#include <func.h>


int main(int argc, char **argv)
{
	
	USART_init();
	
	mcu_io_init();

//	spi_platform_init();

	stm32_init();
	
	printd("main start ....... enter fordd");
	for (;;) {
		;
	}
}
