/***************************************
*	author: Dong
*	specification: uart test
****************************************/


#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include <func.h>

/***************************************
* Name   : NVIC_Configuration
* Deion  : Configures NVIC and Vector Table location
* Input  : None
* Output : None
* Return : None
*****************************************/
void NVIC_Configuration(void)
{
#ifdef ENABLE_UART1
	NVIC_InitTypeDef NVIC_InitStructure_uart1;
#endif
	
#ifdef ENABLE_UART2
	NVIC_InitTypeDef NVIC_InitStructure_uart2;
#endif
	
#ifdef ENABLE_UART3
	NVIC_InitTypeDef NVIC_InitStructure_uart3;
#endif

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
#ifdef ENABLE_UART1	
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure_uart1.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure_uart1.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure_uart1.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure_uart1);
#endif
	
#ifdef ENABLE_UART2	
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure_uart2.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure_uart2.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure_uart2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure_uart2);
#endif

#ifdef ENABLE_UART3
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure_uart3.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure_uart3.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure_uart3.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure_uart3);
#endif
}

#ifdef ENABLE_UART1
/***************************************
* Name  : USART1_GPIO_Configuration
* Deion : Configures the uart1 gpio ports
* Input : None
* Output: None
* Return: None
****************************************/
void USART1_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
#ifdef ENABLE_UART1_RPA10_TPA9
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1_Tx as alternate push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}

/***************************************
* Name  :  USART1_Configuration
* Deion :  Configures the uart1
* Input :  None
* Output:  None
* Return:  None
****************************************/
void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = USART1_BaudRate;
	USART_InitStructure.USART_WordLength = USART1_WordLength;
	USART_InitStructure.USART_StopBits = USART1_StopBits;
	USART_InitStructure.USART_Parity = USART1_Parity;
	USART_InitStructure.USART_HardwareFlowControl =USART1_HardwareFlowControl;

	USART_InitStructure.USART_Mode	=	USART_Mode_Rx | USART_Mode_Tx;

	/* Init the uart1 */
	USART_Init(USART1, &USART_InitStructure);

	/* Enable uart1 receive and send interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* Enable uart1 */
	USART_Cmd(USART1, ENABLE);
}

void USART1_send_char(char ch)
{
	USART_SendData(USART1, ch);
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
}

void USART1_send_buf(char *buf, int len)
{
	char *send_buf = buf;

	while(len)
	{
			USART1_send_char(*send_buf++);
			len--;
	}
}

char USART1_receive_char(void)
{
	while(!USART_GetFlagStatus(USART1, USART_FLAG_RXNE));
	return USART_ReceiveData(USART1);
}

void USART1_Init(void)
{
	USART1_GPIO_Configuration();	
	USART1_Configuration();	
	USART_ClearFlag(USART1,USART_FLAG_TC);
}

/****************************************
* Name   : USART1_IRQHandler
* Deion  : USART1 irq fuc
* Input  : None
* Output : None
* Return : None
*****************************************/
void USART1_IRQHandler(void)
{
	/* USART_SR:5 , 0: no date, 1: date read */
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}
	
	if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_TC);
	}
}
#endif

#ifdef ENABLE_UART2	
/***************************************
* Name  : USART2_GPIO_Configuration
* Deion : Configures the uart1 gpio ports
* Input : None
* Output: None
* Return: None
****************************************/
void USART2_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef ENABLE_UART2_RPA3_TPA2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Configure USART2_Tx as alternate push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART2_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}

/***************************************
* Name  :  USART2_Configuration
* Deion :  Configures the uart1
* Input :  None
* Output:  None
* Return:  None
****************************************/
void USART2_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = USART2_BaudRate;
	USART_InitStructure.USART_WordLength = USART2_WordLength;
	USART_InitStructure.USART_StopBits = USART2_StopBits;
	USART_InitStructure.USART_Parity = USART2_Parity;
	USART_InitStructure.USART_HardwareFlowControl =USART2_HardwareFlowControl;

	USART_InitStructure.USART_Mode	=	USART_Mode_Rx | USART_Mode_Tx;

	/* Init the uart2 */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable uart2 receive and send interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
	/* USART_ITConfig(USART1, USART_IT_TXE, ENABLE); */

	/* Enable uart2 */
	USART_Cmd(USART2, ENABLE);
}

void USART2_send_char(char ch)
{
	USART_SendData(USART2, ch);
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
}

char USART2_receive_char(void)
{
	while(!USART_GetFlagStatus(USART2, USART_FLAG_RXNE));
	return USART_ReceiveData(USART2);
}

void USART2_send_buf(char *buf, int len)
{
	char *send_buf = buf;

	while(len)
	{
			USART2_send_char(*send_buf++);
			len--;
	}
}

void USART2_Init(void)
{
	USART2_GPIO_Configuration();	
	USART2_Configuration();	
	USART_ClearFlag(USART2,USART_FLAG_TC);
}

/****************************************
* Name   : USART2_IRQHandler
* Deion  : USART2 irq fuc
* Input  : None
* Output : None
* Return : None
*****************************************/
void USART2_IRQHandler(void)
{
	/* USART_SR:5 , 0: no date, 1: date read */
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
	
	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
		USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}
	
	if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART2,USART_IT_TC);
	}
}
#endif

#ifdef ENABLE_UART3
/***************************************
* Name  : USART3_GPIO_Configuration
* Deion : Configures the uart1 gpio ports
* Input : None
* Output: None
* Return: None
****************************************/
void USART3_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef ENABLE_UART3_RPC11_TPC10
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Enable the USART3 Pins Software Remapping */
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	/* Configure USART3_Tx as alternate push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure USART3_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
#ifdef ENABLE_UART3_RPB11_TPB10
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Configure USART3_Tx as alternate push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART3_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
}

/***************************************
* Name  :  USART3_Configuration
* Deion :  Configures the uart1
* Input :  None
* Output:  None
* Return:  None
****************************************/
void USART3_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = USART3_BaudRate;
	USART_InitStructure.USART_WordLength = USART3_WordLength;
	USART_InitStructure.USART_StopBits = USART3_StopBits;
	USART_InitStructure.USART_Parity = USART3_Parity;
	USART_InitStructure.USART_HardwareFlowControl =USART3_HardwareFlowControl;

	USART_InitStructure.USART_Mode	=	USART_Mode_Rx | USART_Mode_Tx;

	/* Init the uart3 */
	USART_Init(USART3, &USART_InitStructure);

	/* Enable uart2 receive and send interrupts */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 
	/* USART_ITConfig(USART1, USART_IT_TXE, ENABLE); */

	/* Enable uart2 */
	USART_Cmd(USART3, ENABLE);
}

void USART3_send_char(char ch)
{
	USART_SendData(USART3, ch);
	while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
}

void USART3_send_buf(char *buf, int len)
{
	char *send_buf = buf;

	while(len)
	{
			USART3_send_char(*send_buf++);
			len--;
	}
}

char USART3_receive_char(void)
{
	while(!USART_GetFlagStatus(USART3, USART_FLAG_RXNE));
	return USART_ReceiveData(USART3);
}

void USART3_Init(void)
{
	USART3_GPIO_Configuration();
	USART3_Configuration();
	USART_ClearFlag(USART3,USART_FLAG_TC);
}

/****************************************
* Name   : USART2_IRQHandler
* Deion  : USART2 irq fuc
* Input  : None
* Output : None
* Return : None
*****************************************/
void USART3_IRQHandler(void)
{
	/* USART_SR:5 , 0: no date, 1: date read */
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}

	if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}

	if (USART_GetITStatus(USART3, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART3,USART_IT_TC);
	}
}

#endif

void USART_init(void)
{
#ifdef ENABLE_UART1
	USART1_Init();
#endif
#ifdef ENABLE_UART2
	USART2_Init();
#endif
#ifdef ENABLE_UART3
	USART3_Init();
#endif	
	NVIC_Configuration();
}
void USART_send_buf(char *buf, int len, int com)
{
#ifdef ENABLE_UART1
	if(com == 1)
	{
		USART1_send_buf(buf, len);
		return;
	}
#endif
#ifdef ENABLE_UART2
	if(com == 2)
	{
		USART2_send_buf(buf, len);
		return;
	}
#endif
#ifdef ENABLE_UART3
	if(com == 3)
	{
		USART3_send_buf(buf, len);
		return;
	}
#endif
}

void USART_send_char(char ch, int com)
{
#ifdef ENABLE_UART1
	if(com == 1)
	{
		USART1_send_char(ch);
		return;
	}
#endif
#ifdef ENABLE_UART2
	if(com == 2)
	{
		USART2_send_char(ch);
		return;
	}
#endif
#ifdef ENABLE_UART3
	if(com == 3)
	{
		USART3_send_char(ch);
		return;
	}
#endif
}

char USART_receive_char(int com)
{
	char ch = 0;
#ifdef ENABLE_UART1
	if(com == 1)
	{
		ch = USART1_receive_char();
		return ch;
	}
#endif
#ifdef ENABLE_UART2
	if(com == 2)
	{
		ch = USART2_receive_char();
		return ch;
	}
#endif
#ifdef ENABLE_UART3
	if(com == 3)
	{
		ch = USART3_receive_char();
		return ch;
	}
#endif
	return ch;
}
