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
#define COM3 3

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

static s32 output;
static s32 decimal_in;
static s32 decimal_out;
static char cmd_buf[TLV_MAX_STR_LEN + 1];

#ifdef DEMO_SCHED_LIB
static u8 sched_buf[256];
static u8 sched_out_length;
static const u8 *sched_out;
static struct sched_prop schedule_in;
static void set_schedule_in(struct prop *prop, void *arg, void *valp,
    size_t len);
#endif

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


static s32 devices1_power;
static s32 devices2_power;
static s32 devices3_power;
static s32 devices4_power;
static s32 devices5_power;
static s32 devices6_power;
static s32 devices7_power;
static s32 devices8_power;
static s32 devices9_power;
static s32 devices10_power;
static s32 devices11_power;
static s32 devices12_power;
static s32 devices13_power;
static s32 devices14_power;
static s32 devices15_power;
static s32 devices16_power;
static s32 devices1_temp;
static s32 devices2_temp;
static s32 devices3_temp;
static s32 devices4_temp;
static s32 devices5_temp;
static s32 devices6_temp;
static s32 devices7_temp;
static s32 devices8_temp;
static s32 devices9_temp;
static s32 devices10_temp;
static s32 devices11_temp;
static s32 devices12_temp;
static s32 devices13_temp;
static s32 devices14_temp;
static s32 devices15_temp;
static s32 devices16_temp;
static void set_devices_send_data(char data, char type, char devid, int len);
static void set_devices_power(struct prop *prop, void *arg, void *valp, size_t len);
static void set_devices_temp(struct prop *prop, void *arg, void *valp, size_t len);
struct prop prop_table[] = {
#define DEMO_VERSION       0
	{ "version",          ATLV_UTF8, NULL,               send_version,      NULL,           0,                 AFMT_READ_ONLY},
#define DEVICES1_POWER     1
	{ "devices1_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices1_power,     sizeof(devices1_power)},
#define DEVICES2_POWER     2
	{ "devices2_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices2_power,     sizeof(devices2_power)},
#define DEVICES3_POWER     3
	{ "devices3_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices3_power,     sizeof(devices3_power)},
#define DEVICES4_POWER     4
	{ "devices4_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices4_power,     sizeof(devices4_power)},
#define DEVICES5_POWER     5
	{ "devices5_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices5_power,     sizeof(devices5_power)},
#define DEVICES6_POWER     6
	{ "devices6_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices6_power,     sizeof(devices6_power)},
#define DEVICES7_POWER     7
	{ "devices7_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices7_power,     sizeof(devices7_power)},
#define DEVICES8_POWER     8
	{ "devices8_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices8_power,     sizeof(devices8_power)},
#define DEVICES9_POWER     9
	{ "devices9_work_power",     ATLV_INT,  set_devices_power,      prop_send_generic, &devices9_power,     sizeof(devices9_power)},
#define DEVICES10_POWER    10
	{ "devices10_work_power",    ATLV_INT,  set_devices_power,     prop_send_generic, &devices10_power,    sizeof(devices10_power)},
#define DEVICES11_POWER    11
	{ "devices11_work_power",    ATLV_INT,  set_devices_power,     prop_send_generic, &devices11_power,    sizeof(devices11_power)},
#define DEVICES12_POWER    12
	{ "devices12_work_power",    ATLV_INT,  set_devices_power,     prop_send_generic, &devices12_power,    sizeof(devices12_power)},
#define DEVICES13_POWER    13
	{ "devices13_work_power",    ATLV_INT,  set_devices_power,     prop_send_generic, &devices13_power,    sizeof(devices13_power)},
#define DEVICES14_POWER    14
	{ "devices14_work_power",    ATLV_INT,  set_devices_power,     prop_send_generic, &devices14_power,    sizeof(devices14_power)},
#define DEVICES15_POWER    15
	{ "devices15_work_power",    ATLV_INT,  set_devices_power,     prop_send_generic, &devices15_power,    sizeof(devices15_power)},
#define DEVICES16_POWER    16
	{ "devices16_work_power",    ATLV_INT,  set_devices_power,     prop_send_generic, &devices16_power,    sizeof(devices16_power)},
#define DEVICES1_TEMP    17
	{ "devices1_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices1_temp,     sizeof(devices1_temp)},
#define DEVICES2_TEMP    18
	{ "devices2_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices2_temp,     sizeof(devices2_temp)},
#define DEVICES3_TEMP    19
	{ "devices3_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices3_temp,     sizeof(devices3_temp)},
#define DEVICES4_TEMP    20
	{ "devices4_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices4_temp,     sizeof(devices4_temp)},
#define DEVICES5_TEMP    21
	{ "devices5_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices5_temp,     sizeof(devices5_temp)},
#define DEVICES6_TEMP    22
	{ "devices6_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices6_temp,     sizeof(devices6_temp)},
#define DEVICES7_TEMP    23
	{ "devices7_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices7_temp,     sizeof(devices7_temp)},
#define DEVICES8_TEMP    24
	{ "devices8_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices8_temp,     sizeof(devices8_temp)},
#define DEVICES9_TEMP    25
	{ "devices9_work_temp",     ATLV_INT,  set_devices_temp,      prop_send_generic, &devices9_temp,     sizeof(devices9_temp)},
#define DEVICES10_TEMP   26
	{ "devices10_work_temp",    ATLV_INT,  set_devices_temp,     prop_send_generic, &devices10_temp,    sizeof(devices10_temp)},
#define DEVICES11_TEMP   27
	{ "devices11_work_temp",    ATLV_INT,  set_devices_temp,     prop_send_generic, &devices11_temp,    sizeof(devices11_temp)},
#define DEVICES12_TEMP   28
	{ "devices12_work_temp",    ATLV_INT,  set_devices_temp,     prop_send_generic, &devices12_temp,    sizeof(devices12_temp)},
#define DEVICES13_TEMP   29
	{ "devices13_work_temp",    ATLV_INT,  set_devices_temp,     prop_send_generic, &devices13_temp,    sizeof(devices13_temp)},
#define DEVICES14_TEMP   30
	{ "devices14_work_temp",    ATLV_INT,  set_devices_temp,     prop_send_generic, &devices14_temp,    sizeof(devices14_temp)},
#define DEVICES15_TEMP   31
	{ "devices15_work_temp",    ATLV_INT,  set_devices_temp,     prop_send_generic, &devices15_temp,    sizeof(devices15_temp)},
#define DEVICES16_TEMP   32
	{ "devices16_work_temp",    ATLV_INT,  set_devices_temp,     prop_send_generic, &devices16_temp,    sizeof(devices16_temp)},
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
#ifdef DEMO_SCHED_LIB
	{ "schedule_in", ATLV_SCHED, set_schedule_in, NULL, &schedule_in},
	{ "schedule_out", ATLV_SCHED, NULL, send_schedule, NULL, 0,
	  AFMT_READ_ONLY},
#define PROP_SCHED_OUT 8
#endif

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
#define NAME_LEN 21
#define CMD_TYPE_POWER 0xF0
#define CMD_TYPE_TEMP  0xF1
#define CMD_TYPE_RES   0xF2
#define CMD_TYPE_REQ   0xF3
static void set_devices_send_data(char data, char type, char devid, int len)
{
	char pack[8];

	if(type == (char)CMD_TYPE_POWER) // dev power
	{
		if(data > 0x02) //data: 0x00, 0x01, 0x02
			return;
	}
	if(type == (char)CMD_TYPE_TEMP) //set dev temperature
	{
		if(data < 0x5 || data > 0x23) // set temperature val: 0x5  ~ 0x23
			return;
	}
	pack[0] = 0x55;  //direction to host
	pack[1] = 0x04;  //pack len
	pack[2] = devid; //dev id
	pack[3] = type;  //cmd type
	pack[4] = data;  //set val
	pack[5] = pack[2] ^ pack[3] ^ pack[4]; //verify
	pack[6] = 0xED;  //end

	USART_send_buf(pack, 7, COM3);
}
static void set_devices_power(struct prop *prop, void *arg, void *valp, size_t len)
{
	char dev_name[NAME_LEN];
	char data = *(char *)valp;
	int dev_name_len = prop->name_len;
	u8 dev_id = 0;

	memcpy(dev_name, prop->name, dev_name_len);
	if(dev_name_len == 19) //devices1_work_power ~ devices9_work_power
	{
		dev_id = dev_name[7] - 48;
	}
	if(dev_name_len == 20) //devices10_work_power ~ devices16_work_power
	{
		dev_id = 10 + dev_name[8] - 48;
	}

	set_devices_send_data(data, CMD_TYPE_POWER, dev_id, 7);

	return;
}

static void set_devices_temp(struct prop *prop, void *arg, void *valp, size_t len)
{
	char data = *(char *)valp;
	int dev_name_len = prop->name_len;
	u8 dev_id = 0;

	if(dev_name_len == 18) //devices1_work_temp ~ devices9_work_temp
	{
			dev_id = prop->name[7] - 48;
	}
	if(dev_name_len == 19) //devices10_work_temp ~ devices16_work_temp
	{
			dev_id = 10 + prop->name[8] - 48;
	}

	set_devices_send_data(data, CMD_TYPE_TEMP, dev_id, 7);

	return;
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
	}
}
