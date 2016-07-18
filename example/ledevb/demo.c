/*
 * Copyright 2011-2013 Ayla Networks, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Ayla Networks, Inc.
 */

#include <string.h>
#include <ayla/mcu_platform.h>
#include <mcu_io.h>
#include <stm32.h>
#include <func.h>

#ifdef DEMO_CONF
#include <ayla/conf_token.h>
#include <ayla/conf_access.h>
#endif

#include <ayla/ayla_proto_mcu.h>
#include <ayla/props.h>
#include <ayla/serial_msg.h>



#ifdef DEMO_UART
#include <ayla/uart.h>
#else
#include <spi_platform_arch.h>
#endif /* DEMO_UART */

#ifdef DEMO_FILE_PROP
#include <ayla/prop_dp.h>
#include <demo_stream.h>
#endif /* DEMO_FILE_PROP */

#ifdef DEMO_FILE_PROP
#define VERSION "demo_dp 1.3"
#else
#define VERSION "demo 1.3"
#endif /* DEMO_FILE_PROP */

#ifdef DEMO_SCHED_LIB
#include <ayla/schedeval.h>
#include <ayla/sched.h>
#include <ayla/cmp.h>
#endif /* DEMO_SCHED_LIB */


#ifdef DEMO_IMG_MGMT
#include <flash_layout.h>
#define BUILD_DATE ""
#ifdef AYLA_KEIL
/*
 * Image header location is fixed.
 */
#define IMG_HDR_LOC		(MCU_IMG_ACTIVE + IMAGE_HDR_OFF)
#define IMG_HDR_VER_LOC		(IMG_HDR_LOC + sizeof(struct image_hdr))

const struct image_hdr __img_hdr
			__attribute__((used))
			__attribute((at(IMG_HDR_LOC)));
const char version[72] __attribute((at(IMG_HDR_VER_LOC))) =
	 VERSION " " BUILD_DATE;
#else
const char version[] __attribute__((section(".version"))) =
	VERSION " " BUILD_DATE;
#endif /* AYLA_KEIL */
#else
const char version[] = VERSION;
#endif /* DEMO_IMG_MGMT || AYLA_BUILD_VERSION */

static u8 factory_reset;


static void set_input(struct prop *, void *arg, void *valp, size_t len);
static void set_cmd(struct prop *, void *arg, void *valp, size_t len);
static void set_dec_in(struct prop *prop, void *arg, void *valp, size_t len);

#ifdef DEMO_IMG_MGMT
extern u8 boot2inactive;
extern u8 template_req;
void mcu_img_mgmt_init(void);
int send_inactive_version(struct prop *, void *arg);
void set_boot2inactive(struct prop *, void *arg, void *valp, size_t len);
int send_template_version(struct prop *, void *arg);
void template_version_sent(void);
#endif

static s32 input;
static s32 output;
static s32 decimal_in;
static s32 decimal_out;
static u8 blue_button;
static char cmd_buf[TLV_MAX_STR_LEN + 1];

#ifdef DEMO_SCHED_LIB
static u8 sched_buf[256];
static u8 sched_out_length;
static const u8 *sched_out;
static struct sched_prop schedule_in;
static void set_schedule_in(struct prop *prop, void *arg, void *valp,
    size_t len);
#endif


static void set_led(struct prop *prop, void *arg, void *valp, size_t len)
{
	u8 val = *(u8 *)valp;
	u32 pin = (u32)arg;

	stm32_set_led(pin, val);
}

static int send_led(struct prop *prop, void *arg)
{
	u8 val;
	u32 pin = (u32)prop->arg;

	val = stm32_get_led(pin);
	return prop_send(prop, &val, sizeof(val), arg);
}

static int send_version(struct prop *prop, void *arg)
{
	return prop_send(prop, version, strlen(version), arg);
}

#ifdef DEMO_SCHED_LIB
static int send_schedule(struct prop *prop, void *arg)
{
	return prop_send(prop, sched_out, sched_out_length, arg);
}
#endif

static s32 devices_mode;
static s32 devices_set_htemp;
static s32 devices_set_btemp;
static s32 devices_fcode;
static s32 devices_work_htemp;
static s32 devices_work_btemp;
static s32 devices_iotemp;
static s32 devices_scode;
static s32 devices_cval;
static s32 devices_wrate;
static s32 devices_cpower;
static s32 devices_exfunc;

static u8 send_from_ctrl_board_ready = FALSE;
static u8 send_from_ctrl_display_ready = FALSE;
#define BOARD_LEN 6
static char ctrl_board_pack[BOARD_LEN];
static int index_board = 0;
#define DISPLAY_LEN 6
static char ctrl_display_pack[DISPLAY_LEN];
static int index_display = 0;

static void set_devices_mode(struct prop *prop, void *arg, void *valp, size_t len);
static void set_devices_htemp(struct prop *prop, void *arg, void *valp, size_t len);
static void set_devices_btemp(struct prop *prop, void *arg, void *valp, size_t len);
static void devices_obligate_func(struct prop *prop, void *arg, void *valp, size_t len);
static void devices_expand_func(struct prop *prop, void *arg, void *valp, size_t len);

struct prop prop_table[] = {
#define DEMO_VERSION         0
	{ "version",               ATLV_UTF8, NULL,                  send_version,      NULL,                0,                 AFMT_READ_ONLY},
#define DEVICES_MODE         1
	{ "device_work_mode",      ATLV_INT,  set_devices_mode,      prop_send_generic, &devices_mode,       sizeof(devices_mode)},
#define DEVICES_SET_HTEMP    2
	{ "heating_set_temp",      ATLV_INT,  set_devices_htemp,     prop_send_generic, &devices_set_htemp,  sizeof(devices_set_htemp)},
#define DEVICES3_SET_BTEMP   3
	{ "bath_set_temp",         ATLV_INT,  set_devices_btemp,     prop_send_generic, &devices_set_btemp,  sizeof(devices_set_btemp)},
#define DEVICES_FAULT_CODE   4
	{ "device_fault_code",     ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_fcode,      sizeof(devices_fcode)},
#define DEVICES_WORK_HTEMP   5
	{ "heating_work_temp",     ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_work_htemp, sizeof(devices_work_htemp)},
#define DEVICES_WORK_BTEMP   6
	{ "bath_work_temp",        ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_work_btemp, sizeof(devices_work_btemp)},
#define DEVICES_INOUT_TEMP   7
	{ "indoor_outdoor_temp",   ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_iotemp,     sizeof(devices_iotemp)},
#define DEVICES_STATUS_CODE  8
	{ "status_code",           ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_scode,      sizeof(devices_scode)},
#define DEVICES_CVAL         9
	{ "gas_valve_current",     ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_cval,       sizeof(devices_cval)},
#define DEVICES_WATER_RATE   10
	{ "bath_water_rate",       ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_wrate,      sizeof(devices_wrate)},
#define DEVICES_CUR_POWER    11
	{ "devices_current_power", ATLV_INT,  devices_obligate_func, prop_send_generic, &devices_cpower,     sizeof(devices_cpower)},
#define DEVICES_EFUNC        12
	{ "devices_expand_func",   ATLV_INT,  devices_expand_func,   prop_send_generic, &devices_exfunc,     sizeof(devices_exfunc)},
	{ "Blue_button", ATLV_BOOL, NULL, prop_send_generic,
	    &blue_button, sizeof(blue_button), AFMT_READ_ONLY},
#define PROP_BUTTON 0
	{ "output", ATLV_INT, NULL, prop_send_generic, &output,
	    sizeof(output), AFMT_READ_ONLY},
#define PROP_OUTPUT 1
	{ "log", ATLV_UTF8, NULL, prop_send_generic, &cmd_buf[0],
	    0, AFMT_READ_ONLY},
#define PROP_LOG 2
	{ "decimal_out", ATLV_CENTS, NULL, prop_send_generic, &decimal_out,
	    sizeof(decimal_out), AFMT_READ_ONLY},
#define PROP_DEC_OUT 3
	{ "decimal_in", ATLV_CENTS, set_dec_in, prop_send_generic,
	    &decimal_in, sizeof(decimal_in)},
	{ "Blue_LED", ATLV_BOOL, set_led, send_led,
	    (void *)(1 << LED0_PIN), 1},
	{ "Green_LED", ATLV_BOOL, set_led, send_led,
	    (void *)(1 << LED1_PIN), 1},
#ifdef DEMO_SCHED_LIB
	{ "schedule_in", ATLV_SCHED, set_schedule_in, NULL, &schedule_in},
	{ "schedule_out", ATLV_SCHED, NULL, send_schedule, NULL, 0,
	  AFMT_READ_ONLY},
#define PROP_SCHED_OUT 8
#endif
	{ "cmd", ATLV_UTF8, set_cmd, prop_send_generic, &cmd_buf[0]},
	{ "input", ATLV_INT, set_input, prop_send_generic,
	    &input, sizeof(input)},
#ifdef DEMO_IMG_MGMT
	{ "inactive_version", ATLV_UTF8, NULL, send_inactive_version, NULL,
	  0, AFMT_READ_ONLY },
	{ "boot_to_inactive", ATLV_BOOL, set_boot2inactive, prop_send_generic,
	  &boot2inactive, sizeof(boot2inactive) },
	{ "oem_host_version", ATLV_UTF8, NULL, send_template_version },
#endif
#ifdef DEMO_FILE_PROP
	/*
	 * Long data points must use property type ATLV_LOC in this table,
	 * even though they have type ATLV_BIN in the protocol.
	 */
	{ "stream_up_len", ATLV_INT, set_length_up, prop_send_generic,
	    &stream_up_len, sizeof(stream_up_len)},
	{ "stream_up", ATLV_LOC, NULL, prop_dp_send, &stream_up_state, 0},
	{ "stream_down", ATLV_LOC, prop_dp_set, prop_dp_send,
	    &stream_down_state, 0},
	{ "stream_down_len", ATLV_UINT, NULL, prop_send_generic,
	   &stream_down_state.next_off, sizeof(stream_down_state.next_off)},
	{ "stream_down_match_len", ATLV_UINT, NULL, prop_send_generic,
	   &stream_down_patt_match_len, sizeof(stream_down_patt_match_len)},
#endif /* DEMO_FILE_PROP */
	{ NULL }
};
u8 prop_count = (sizeof(prop_table) / sizeof(prop_table[0])) - 1;

#define CTRL_PANNEL     (0xAA)
#define CTRL_COM        (1)
#define DISPLAY_SCREEEN (0xBA)
#define DISPLAY_COM     (3)
#define CMD_HTEMP       (0xD0)
#define CMD_BTEMP       (0xD1)
#define CMD_MODE        (0xD2)
#define ARGV1           (0x01)
#define PACK_END        (0x0D)

void Clean_ctrl_board()
{
	memset(ctrl_board_pack, 0, BOARD_LEN);
	index_board = 0;
}

void Clean_ctrl_display()
{
	memset(ctrl_display_pack, 0, DISPLAY_LEN);
	index_display = 0;
}

/****************************************************************
* FUNC   :  receive dispaly data of Byte
* uart3  :
********************************************************************/
void Receive_display_Byte(char ch)
{
	USART_send_char((char)ch, 3);

	if((ch == (char)0xAA) && (index_display == 0) && ctrl_display_pack[0] != (char)0xAA) 
	{
		ctrl_display_pack[index_display++] = ch;
		return;
	}
	if(ch == (char)0x0D && index_display == (BOARD_LEN - 1))
	{
		ctrl_display_pack[index_display++] = ch;
		send_from_ctrl_display_ready = TRUE;  // host ready data up to the service
		return;
	}
	ctrl_display_pack[index_display++] = ch;
	return;
}

u8 send_property_from_ctrl_display( void )
{
	char *boardcmd = ctrl_display_pack;
	int hostcmd_len = index_display;
	int arg2_id = 0;

	// board cmd B0:0xAA, end 0x0D
	if(boardcmd[0] != (char)0xAA && hostcmd_len == BOARD_LEN && boardcmd[hostcmd_len - 1] != (char)0x0D)
	{
		Clean_ctrl_display();
		return FALSE;
	}

	// send cmd to master board
	USART_send_buf(boardcmd, hostcmd_len, CTRL_COM);
	if(boardcmd[1] == (char)0xD0) //heating work temp and set temp
	{
		arg2_id = DEVICES_SET_HTEMP;
	}
	if(boardcmd[1] == (char)0xD1) //bath work temp
	{
		arg2_id = DEVICES3_SET_BTEMP;
	}
	if(boardcmd[1] == (char)0xD2) //devices mode, real time sync to server
	{
		arg2_id = DEVICES_MODE;
		*(s32 *)prop_table[arg2_id].arg = (s32)boardcmd[3];
		prop_table[arg2_id].val_len = sizeof(char);
		prop_table[arg2_id].send_mask = ADS_BIT;
	}

	Clean_ctrl_display();
	return TRUE;
}

/****************************************************************
* FUNC   :  receive master board  data of Byte
* uart1  :  B0:0xAA, B1:cmd, B2:arg1, B3:arg2, B4:crc, B5:end(0x0D)
********************************************************************/
void Receive_Ctrl_Board_Byte(char ch)
{
	USART_send_char((char)ch, 1);

	if((ch == (char)0xAA) && (index_board == 0) && ctrl_board_pack[0] != (char)0xAA) 
	{
		ctrl_board_pack[index_board++] = ch;
		return;
	}
	if(ch == (char)0x0D && index_board == (BOARD_LEN - 1))
	{
		ctrl_board_pack[index_board++] = ch;
		send_from_ctrl_board_ready = TRUE;  // host ready data up to the service
		return;
	}
	ctrl_board_pack[index_board++] = ch;
	return;
}

u8 send_property_from_ctrl_board( void )
{
	char *boardcmd = ctrl_board_pack;
	int hostcmd_len = index_board;
	int arg1_id = 0;
	int arg2_id = 0;

	// board cmd B0:0xAA, end 0x0D
	if(boardcmd[0] != (char)0xAA && hostcmd_len == BOARD_LEN && boardcmd[hostcmd_len - 1] != (char)0x0D)
	{
		Clean_ctrl_board();
		return FALSE;
	}

	// send cmd to display
	USART_send_buf(boardcmd, hostcmd_len, DISPLAY_COM);
	if(boardcmd[1] == (char)0xD0) //heating work temp and set temp
	{
		arg1_id = DEVICES_SET_HTEMP;
		arg2_id = DEVICES_WORK_HTEMP;
	}
	if(boardcmd[1] == (char)0xD1) //bath work temp
	{
		arg1_id = DEVICES3_SET_BTEMP;
		arg2_id = DEVICES_WORK_BTEMP;
	}
	if(boardcmd[1] == (char)0xD2) //devices mode, real time sync to server
	{
		arg1_id = DEVICES_MODE;
		*(s32 *)prop_table[arg1_id].arg = (s32)boardcmd[2];
		prop_table[arg1_id].val_len = sizeof(char);
		prop_table[arg1_id].send_mask = ADS_BIT;

		arg2_id = DEVICES_INOUT_TEMP;
	}
	if(boardcmd[1] == (char)0xD5) //failt code, real time sync to server
	{
		arg1_id = 0;

		arg2_id = DEVICES_FAULT_CODE;
		*(s32 *)prop_table[arg2_id].arg = (s32)boardcmd[3];
		prop_table[arg2_id].val_len = sizeof(char);
		prop_table[arg2_id].send_mask = ADS_BIT;
	}
	if(boardcmd[1] == (char)0xD6) //bath weter and status code, real time sync to server
	{
		arg1_id = DEVICES_WATER_RATE;

		arg2_id = DEVICES_STATUS_CODE;
		*(s32 *)prop_table[arg2_id].arg = (s32)boardcmd[3];
		prop_table[arg2_id].val_len = sizeof(char);
		prop_table[arg2_id].send_mask = ADS_BIT;
	}
	if(boardcmd[1] == (char)0xD8) //current
	{
		arg1_id = 0;
		arg2_id = DEVICES_CVAL;
	}

	Clean_ctrl_board();
	return TRUE;
}

static void send_data_to_host(char data, char cmd_type, int com, char host)
{
		char pack[7];

		pack[0] = host;      //direction to host
		pack[1] = cmd_type;  //cmd type
		pack[2] = ARGV1;      //arg1
		pack[3] = data;      //arg2, val
		pack[4] = pack[1] ^ pack[2] ^ pack[3]; //verif
		pack[5] = PACK_END;  //end

		USART_send_buf(pack, 6, com);
}

static void set_devices_mode(struct prop *prop, void *arg, void *valp, size_t len)
{
	char data = *(char *)valp;


	/******************
	* data == 0x00 dev off
	* data == 0x03 dev summer mode on
	* data == 0x05 dev winter mode on
	*******************/
	if(data == (char)0x00 || data == (char)0x03 || data == (char)0x05)
	{
		send_data_to_host(data, CMD_MODE, DISPLAY_COM, DISPLAY_SCREEEN);
		send_data_to_host(data, CMD_MODE, CTRL_COM, CTRL_PANNEL);
	}
}

static void set_devices_htemp(struct prop *prop, void *arg, void *valp, size_t len)
{
	char data = *(char *)valp;

	/************************
	* set heating temp of dev
  * data (30, 85)
	*************************/
	if(data >= (char)0x1E && data <= (char)0x55)
	{
		send_data_to_host(data, CMD_HTEMP, DISPLAY_COM, DISPLAY_SCREEEN);
		send_data_to_host(data, CMD_HTEMP, CTRL_COM, CTRL_PANNEL);
	}
}
static void set_devices_btemp(struct prop *prop, void *arg, void *valp, size_t len)
{
	char data = *(char *)valp;
	
	/************************
	* set bath temp of dev
  * data (30, 60)
	*************************/
	if(data >= (char)0x1E && data <= (char)0x3C)
	{
		send_data_to_host(data, CMD_BTEMP, DISPLAY_COM, DISPLAY_SCREEEN);
		send_data_to_host(data, CMD_BTEMP, CTRL_COM, CTRL_PANNEL);
	}
}
static void devices_obligate_func(struct prop *prop, void *arg, void *valp, size_t len)
{
	;
}
static void devices_expand_func(struct prop *prop, void *arg, void *valp, size_t len)
{
	;
}

static void set_input(struct prop *prop, void *arg, void *valp, size_t len)
{
	s32 i = *(s32 *)valp;

	if (len != sizeof(s32)) {
		return;
	}
	input = i;
	if (i > 0x7fff || i < -0x8000) {
		output = -1;		/* square would overflow */
	} else {
		output = i * i;
	}
	prop_table[PROP_OUTPUT].send_mask = valid_dest_mask;
}

static void set_dec_in(struct prop *prop, void *arg, void *valp, size_t len)
{
	s32 i = *(s32 *)valp;

	if (len != sizeof(s32)) {
		return;
	}
	decimal_in = i;
	decimal_out = i;
	prop_table[PROP_DEC_OUT].send_mask = valid_dest_mask;
}

static void set_cmd(struct prop *prop, void *arg, void *valp, size_t len)
{
	if (len >= sizeof(cmd_buf)) {
		len = sizeof(cmd_buf) - 1;
	}
	memcpy(cmd_buf, valp, len);
	cmd_buf[len] = '\0';
	prop_table[PROP_LOG].send_mask = valid_dest_mask;
}

#ifdef DEMO_SCHED_LIB
static void set_schedule_in(struct prop *prop, void *arg, void *valp,
    size_t len)
{
	if (len > sizeof(sched_buf)) {
		len = sizeof(sched_buf);
	}
	memcpy(sched_buf, valp, len);
	sched_out = sched_buf;
	sched_out_length = len;
	prop_table[PROP_SCHED_OUT].send_mask = valid_dest_mask;
	memcpy(schedule_in.tlvs, valp, len);
	schedule_in.len = sched_out_length;
	sched_run_all(NULL);
}
#endif

/*
 * Blue button push observed by interrupt handler.
 * Callers are in stm32.c
 */
void demo_set_button_state(u8 button_value)
{
	blue_button = button_value;
	prop_table[PROP_BUTTON].send_mask = valid_dest_mask;
}

int main(int argc, char **argv)
{
	struct prop *prop;

	USART_init();
	feature_mask |= MCU_LAN_SUPPORT;
#ifdef DEMO_IMG_MGMT
	mcu_img_mgmt_init();
	feature_mask |= MCU_OTA_SUPPORT;
#endif
#ifdef DEMO_SCHED_LIB
	feature_mask |= MCU_TIME_SUBSCRIPTION;
#endif
	mcu_io_init();
#ifdef DEMO_UART
	feature_mask |= MCU_DATAPOINT_CONFIRM;
	uart_init();
#else
	spi_platform_init();
#endif
	stm32_reset_module();
	stm32_init();
	factory_reset = stm32_factory_reset_detect();
#ifdef DEMO_FILE_PROP
	demo_stream_init();
#endif /* DEMO_FILE_PROP */
	printd("main start ....... enter fordd");
	for (;;) {
		if (stm32_ready()) {
			if (factory_reset &&
			    !serial_tx_cmd(ACMD_LOAD_FACTORY, NULL, 0)) {
				factory_reset = 0;
				stm32_set_factory_rst_led(0);
				while (stm32_ready()) {
					serial_poll();
				}
			}
#ifdef DEMO_CONF
			conf_poll();
#endif
			prop_poll();
			serial_poll();
#ifdef DEMO_SCHED_LIB
			if (sched_next_event_tick &&
			    (tick == sched_next_event_tick ||
			    cmp_gt(tick, sched_next_event_tick))) {
				sched_run_all(&sched_next_event_tick);
			}
#endif
		}
#ifdef DEMO_IMG_MGMT
		if (template_req &&
		    prop_send_done(prop_lookup("oem_host_version")) == 0) {
			/*
			 * Template version number has been sent.
			 */
			template_version_sent();
		}
#endif
		prop = prop_lookup_error();
		if (prop != NULL) {
			/*
			 * Property send has failed with error code.
			 * Error code is available in prop->send_err
			 *
			 * Insert logic here to handle the failure.
			 */
			prop->send_err = 0;
		}
		if( send_from_ctrl_board_ready )
		{
				send_property_from_ctrl_board();
				send_from_ctrl_board_ready = FALSE;
		}
		if( send_from_ctrl_display_ready )
		{
				send_property_from_ctrl_display();
				send_from_ctrl_display_ready = FALSE;
		}
	}
}
