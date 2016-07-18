
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

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
#define USART1_BaudRate 4800
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
#define ENABLE_UART3
//#define ENABLE_UART3_RPC11_TPC10
#define ENABLE_UART3_RPB11_TPB10
#define USART3_BaudRate 4800
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

#define TIM3_ENABLE
void TIM3_Init(int ms);
void TIM3_Min_Init(void);
/********************************************
*internal flash rw
********************************************/
void WirteFlash(int addoff, uint32_t *data, int wordn);
uint32_t ReadFlash(int addoff);

void Receive_Ctrl_Board_Byte(char ch);
void Receive_display_Byte(char ch);

/******************************************************
* #define DEVICES_MODE         1    "device_work_mode"
* #define DEVICES_SET_HTEMP    2    "heating_set_temp"
* #define DEVICES3_SET_BTEMP   3    "bath_set_temp"
* #define DEVICES_FAULT_CODE   4    "device_fault_code"
* #define DEVICES_WORK_HTEMP   5    "heating_work_temp"
* #define DEVICES_WORK_BTEMP   6    "bath_work_temp"
* #define DEVICES_INOUT_TEMP   7    "indoor_outdoor_temp"
* #define DEVICES_STATUS_CODE  8    "status_code"
* #define DEVICES_CVAL         9    "gas_valve_current"
* #define DEVICES_WATER_RATE   10   "bath_water_rate"
* #define DEVICES_EFUNC        11   "devices_expand_func"
* #define DEVICES_GTEMP        12   "devices_gas_temp"
* #define DEVICES_WLEVEL       13   "condensate_water_level"
* #define DEVICES_HWTEMP       14   "heating_water_temp"
* #define DEVICES_BWTEMP       15   "bath_water_temp"
* #define DEVICES_PVOLTAGE     16   "devices_power_voltage"
* #define DEVICES_FTACHOME     17   "fan_tachometer"
* #define DEVICES_RSTATUS      18   "request_status"
************************************************************/
