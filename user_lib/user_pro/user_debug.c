
#include "func.h"

void printd(char *p)
{
#ifdef USER_DEBUG
	char *str = p;
	int index = 0;
	
	while(*(str + index) != '\0' && *(str + index) != 'N')
	{
		USART_send_char(*(str + index), USER_DEBUG_COM);
		index = index + 1;
	}
#else
	;
#endif
}
