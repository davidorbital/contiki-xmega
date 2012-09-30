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
 * 		Sensors for DS1820 (1-Wire Digital Thermometer).
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 * @note
 * 		See http://www.maximintegrated.com/app-notes/index.mvp/id/162
 *
 * 		DS1820 is connected to Port B, Pin 0.
 */

#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <contiki.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "ow-master.h"
#include "ds1820-sensor.h"


//#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif


/* Define the 1-Wire master hosting the DS1820. */
static ow_xmega_master_t ow_master = { &PORTB, PIN0_bm };

/* Define the sensor object. */
const struct sensors_sensor ds1820_sensor;

/* Sensor status. */
enum {
	ON,
	OFF
};
static uint8_t state = OFF;

/**
 * ds1820_temperature
 *
 * Register 0 is Temperature LSB, in 0.5 degC resolution.
 * Register 1 is Temperature MSB, all bits 1 indicates negative temperature.
 * Register 6 is Count Remain, used to calculate more accurate temperature.
 * Register 7 is Count per C, used to calculate more accurate temperature.
 *
 * @return	Temperature * 100 (centi %).
 */
static int
ds1820_temperature(void)
{
	int rc;
	int temperature;
	uint8_t cmd_init_measurement[2] = { 0xcc, 0x44 };
	uint8_t cmd_read_scratchpad[2] = { 0xcc, 0xbe };
	uint8_t scratchpad[9] = {0};

	/* Start the temperature measurement. */
	rc = ow_transaction(&ow_master, cmd_init_measurement, 2, NULL, 0);
	if (rc != 0) {
		return 0;
	}

	/* Wait for the measurement to complete, DS1820 must be connected to VCC. */
	ow_wait_on_read(&ow_master, 1);

	/* Read the scratchpad and extract the temperature measurement. */
	rc = ow_transaction(&ow_master, cmd_read_scratchpad, 2, scratchpad, 9);
	if (rc != 0) {
		return 0;
	}

	/*
	 * Calculate the temperature.
	 *
	 * temperature = TEMP_READ - 0.25 + Count Remain / Count per C
	 * TEMP_READ = scratchpad[0] with bit 0 truncated
	 */
	if (scratchpad[1])
		scratchpad[0] = ~scratchpad[0] + 1; /* Two's complement. */
	temperature = (scratchpad[0] >> 1) * 100;
	if (scratchpad[1])
		temperature *= -1;
	temperature -= 25;
	temperature += (scratchpad[6] * 100) / scratchpad[7];

	dprintf("Temperature is %d\n", temperature);

	return temperature;
}

/**
 * ds1820_rom_code
 *
 * Byte 0: Family Code.
 * Byte 1-6: 48 bit Serial Number
 * byte 7: CRC of preceding 7 bytes.
 *
 * @return	The family code for the DS1820, as reported in the ROM CODE.
 */
static int
ds1820_rom_code(void)
{
	uint8_t cmd[1] = { 0x33 };
	uint8_t rom_code[8] = {0};

	ow_transaction(&ow_master, cmd, 1, rom_code, 8);

	dprintf("ROM CODE: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			rom_code[7], rom_code[6], rom_code[5], rom_code[4],
			rom_code[3], rom_code[2], rom_code[1], rom_code[0]);

	return rom_code[0];
}

/**
 * value
 */
static int
value(int type)
{
	switch (type) {
	case DS1820_SENSOR_TEMP:
		return ds1820_temperature();
	case DS1820_SENSOR_ROM_CODE:
		return ds1820_rom_code();
	default:
		break;
	}

	return 0;
}

/**
 * configure
 */
static int
configure(int type, int c)
{
	switch (type) {
	case SENSORS_ACTIVE:
		state = OFF;
		dprintf("DS1820 Sensor Configure ...\n");
		if (ow_init(&ow_master))
			state = ON;
		break;
	default:
		state = OFF;
	}

	return 0;
}

/**
 * status
 */
static int
status(int type)
{
	switch (type) {
	case SENSORS_ACTIVE:
	case SENSORS_READY:
		return (state == ON);
	default:
		return 0;
	}
}

/* Initialise the sensor object and make it available to Contiki OS. */
SENSORS_SENSOR(ds1820_sensor, "ds1820", value, configure, status);
