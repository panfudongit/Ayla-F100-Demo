
#include "stm32f10x.h"

//#define READ_PB10_INIT_PB11
#define READ_PA12_INT_PA3
#define USER_SPI1
//#define USER_SPI2
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
//#define ENABLE_UART2
//#define ENABLE_UART2_RPA3_TPA2
//#define ENABLE_UART3
//#define ENABLE_UART3_RPC11_TPC10
//#define ENABLE_UART3_RPB11_TPB10

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

/**********************************************
* prop func
***********************************************/
void Receive_Host_Byte(u8 ch);
void single_devices_statu_updata(int dev_id, int val, int type);
void multi_devices_statu_updata(u8 *hostpack, int packlen);
