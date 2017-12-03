
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

#define TRUE			1
#define FALSE			0
#define USER_DEBUG
#define USER_DEBUG_COM 1
void printd(char *p);
/*****************************************
*Uart func
*****************************************/
void USART_init(void);
void USART_send_char(char ch, int com);
void USART_send_buf(char *buf, int len, int com);
char USART_receive_char(int com);
#define ENABLE_UART1
#define ENABLE_UART1_RPA10_TPA9
#define USART1_BaudRate 9600
#define USART1_WordLength USART_WordLength_8b
#define USART1_StopBits USART_StopBits_1
#define USART1_Parity USART_Parity_No
#define USART1_HardwareFlowControl USART_HardwareFlowControl_None
//#define ENABLE_UART2
//#define ENABLE_UART2_RPA3_TPA2
//#define USART2_BaudRate 9600
//#define USART2_WordLength USART_WordLength_8b
//#define USART2_StopBits USART_StopBits_1
//#define USART2_Parity USART_Parity_No
//#define USART2_HardwareFlowControl USART_HardwareFlowControl_None
//#define ENABLE_UART3
//#define ENABLE_UART3_RPC11_TPC10
//#define ENABLE_UART3_RPB11_TPB10
#define USART3_BaudRate 9600
#define USART3_WordLength USART_WordLength_8b
#define USART3_StopBits USART_StopBits_1
#define USART3_Parity USART_Parity_No
#define USART3_HardwareFlowControl USART_HardwareFlowControl_None

/********************************************
* delay func
********************************************/
void delay_init(void);
void delay_us(u32 nus);
void delay_ms(u32 nms);

/********************************************
*internal flash rw
********************************************/
void WirteFlash(int addoff, uint32_t *data, int wordn);
uint32_t ReadFlash(int addoff);


// SPI
//#define READ_PB10_INIT_PB11
//#define READ_PA12_INT_PA3
#define USER_SPI1
#define USER_SPI1_RX_IRQ
// #define USER_SPI2
// #define USER_SPI2_RX_IRQ
void spi_platform_init(void);
void spi_platform_out(u8 byte);
u8 spi_platform_in(void);
