/*
 * Copyright (c) 2012, Timothy Rule <trule.github@nym.hush.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * @file
 * 		Display App, uOLED 96 G2 SGC.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <contiki.h>
#include <rs232.h>
#include <util/delay.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <dev/xmega-sensor.h>
#include <dev/sht21-sensor.h>
#include <dev/ds3231-sensor.h>
#include <dev/ds1820-sensor.h>


#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define SERIAL_PORT			RS232_USARTF0
#define DISPLAY_RESET_PORT	PORTF
#define DISPLAY_RESET_PIN	PIN0_bm

#define ACK					0x06
#define NAK					0x15

#define RS232_BUFFER_SIZE	8

static struct etimer timer_display_app;

/*
 *
 * @Note	Called from interrupt handler.
 */
static uint8_t input_buffer[RS232_BUFFER_SIZE];
static uint8_t buffer_pos;
static uint8_t last_pos;

static int
_rs232_input(uint8_t c)
{
	buffer_pos = (buffer_pos + 1) % RS232_BUFFER_SIZE;
	input_buffer[buffer_pos] = c;

	return 0;
}

static uint8_t
_rs232_getc(void)
{
	int wait_count = 20;

	while ((buffer_pos == last_pos) && wait_count--)
		_delay_ms(50);

	if (buffer_pos == last_pos)
		return 0;

	last_pos = (last_pos + 1) % RS232_BUFFER_SIZE;

	return input_buffer[last_pos];
}

static void
_send_simple_cmd(uint8_t *cmd, int len)
{
	uint8_t ack_nak;

	dprintf("Send simple command, len %d:\n", len);
	while (len--) {
		dprintf("  Send char %02x\n", *cmd);
		rs232_send(SERIAL_PORT, *cmd);
		cmd++;
	}

	ack_nak = _rs232_getc();
	if (ack_nak == ACK)
		dprintf("    Command ACK'ed.\n");
	else
		dprintf("    Command did not ACK! (%02x)\n", ack_nak);
}

static void
init(void)
{
	/* Set reset high. */
	DISPLAY_RESET_PORT.OUTSET = DISPLAY_RESET_PIN;
	DISPLAY_RESET_PORT.DIRSET = DISPLAY_RESET_PIN;

	/* Set the serial port to 256000 8N1. */
	rs232_init(SERIAL_PORT, XMEGA_BAUD_ASYNC_256000_32M, USART_MODE_ASYNC
			| USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);
}

static void
reset(void)
{
	/* Disable serial port input. */
	rs232_set_input(SERIAL_PORT, NULL);

	/* Pulse reset low for 2uS to reset. */
	DISPLAY_RESET_PORT.OUTCLR = DISPLAY_RESET_PIN;
	_delay_us(2);
	DISPLAY_RESET_PORT.OUTSET = DISPLAY_RESET_PIN;

	/* Wait for the display to boot up. */
	_delay_ms(500);

	/* Enable serial port input. */
	last_pos = buffer_pos = 0;
	rs232_set_input(SERIAL_PORT, _rs232_input);

	/* Autobaud and then set the background. */
	_send_simple_cmd((uint8_t *)"\x55", 1);
	_send_simple_cmd((uint8_t *)"\x42\x00\x00", 3);
}

static void
clear(void)
{
	_send_simple_cmd((uint8_t *)"\x45", 1);
}

static void
sleep(void)
{
	/* Display off. */
	_send_simple_cmd((uint8_t *)"\x59\x01\x00", 3);

	/* Sleep with wake on serial. */
	_send_simple_cmd((uint8_t *)"\x5a\x01\x00", 3);
}

static void
wake(void)
{
	/* Autobaud, then display on. */
	_send_simple_cmd((uint8_t *)"\x55", 1);
	_send_simple_cmd((uint8_t *)"\x59\x01\x01", 3);
}

static void
text(uint8_t pos_x, uint8_t pos_y, uint8_t font, uint8_t size, uint16_t colour, char* string)
{
	uint8_t buffer[100];
	uint8_t *cmd = buffer;

	*cmd++ = 0x53;
	*cmd++ = pos_x;
	*cmd++ = pos_y;
	*cmd++ = font;
	*cmd++ = colour >> 8;
	*cmd++ = colour;
	*cmd++ = size;
	*cmd++ = size;
	memcpy(cmd, string, strlen(string) + 1);

	_send_simple_cmd(buffer, cmd - buffer + strlen(string) + 1);
}

static void
rectangle(uint8_t pos_x1, uint8_t pos_y1, uint8_t pos_x2, uint8_t pos_y2, uint16_t colour)
{
	uint8_t buffer[7];
	uint8_t *cmd = buffer;

	*cmd++ = 0x72;
	*cmd++ = pos_x1;
	*cmd++ = pos_y1;
	*cmd++ = pos_x2;
	*cmd++ = pos_y2;
	*cmd++ = colour >> 8;
	*cmd++ = colour;

	_send_simple_cmd(buffer, 7);
}

static void
circle(uint8_t pos_x, uint8_t pos_y, uint8_t radius, uint16_t colour)
{
	uint8_t buffer[6];
	uint8_t *cmd = buffer;

	*cmd++ = 0x43;
	*cmd++ = pos_x;
	*cmd++ = pos_y;
	*cmd++ = radius;
	*cmd++ = colour >> 8;
	*cmd++ = colour;
	*cmd++ = 0x00;

	_send_simple_cmd(buffer, 6);
}

typedef struct {
	int		id;
	struct sensors_sensor *sensor;
	int		divisor;
	char 	*name;
	char	*type;
	char	*unit;
	char	*flag;
} sensor_t;

sensor_t sensor_list[] = {
	{ SHT21_SENSOR_HUMIDITY, &sht21_sensor, 100, "SHT21", "Humidity", "(Rel %)", "" },
	{ SHT21_SENSOR_TEMP, &sht21_sensor, 100, "SHT21", "Temperature", "(degC)", "" },
	{ DS3231_SENSOR_TEMP, &ds3231_sensor, 100, "DS3231", "Temperature", "(degC)", "" },
	{ DS1820_SENSOR_TEMP, &ds1820_sensor, 100, "DS1820", "Temperature", "(degC)", "" },
	{ XMEGA_SENSOR_VCC, &xmega_sensor, 1000, "XMEGA", "Voltage", "(Volt)", "" },
};

static void
print_sensor(sensor_t *data)
{
	char buffer[10];
	int value;

	clear();
	value = data->sensor->value(data->id);
	sprintf(buffer, "%d.%02d", value / data->divisor, value % data->divisor);
	text(1, 1, 2, 1, 0xaaaa, data->name);
	text(1, 16, 0, 1, 0xaaaa, data->type);
	text(3, 25, 0, 1, 0xaaaa, data->unit);
	text(1, 38, 2, 2, 0x8855, buffer);
	rectangle(70, 1, 94, 25, 0x5555);
}



PROCESS(display_app_process, "Display App");
PROCESS_THREAD(display_app_process, ev, data)
{
	PROCESS_BEGIN();

	int i;

	dprintf("Display App starting ...\n");

	/* Enable sensors. */
	for (i = 0; i < ARRAY_SIZE(sensor_list); i++) {
		SENSORS_ACTIVATE(*sensor_list[i].sensor);
	}

	/* Enable display. */
	init();
	reset();

	/* Start the display loop. */
	while (1) {
		wake();
		for (i = 0; i < ARRAY_SIZE(sensor_list); i++) {
			print_sensor(&sensor_list[i]);
			_delay_ms(2000);
		}
		sleep();

		etimer_set(&timer_display_app, CLOCK_SECOND * 10);
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
	}

	PROCESS_END();
}
