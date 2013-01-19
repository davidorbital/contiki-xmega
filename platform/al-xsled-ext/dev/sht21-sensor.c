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
 * 		Sensors for SHT21 (Humidity & Temperature).
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 * @note
 * 		See www.sensirion.com/SHT21
 */

#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <contiki.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "twi-master.h"
#include "sht21-sensor.h"

#define SHT21_ADDR					0x80
#define SHT21_CMD_RESET				0xfe
#define SHT21_CMD_TEMP				0xe3
#define SHT21_CMD_RH				0xe5
#define SHT21_CMD_READ_USER_REG		0xe7
#define SHT21_CMD_WRITE_USER_REG	0xe6
#define SHT21_HEAT_ON				0x04
#define SHT21_HEAT_OFF				(~SHT21_HEAT_ON)

/* SHT21 is connected to TWI Master on Port C. */
#define I2C(a,b,c,d)	twi_transaction(&TWIC, SHT21_ADDR, \
								(a), (b), (c), (d));

//#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif

/* Define the sensor object. */
const struct sensors_sensor sht21_sensor;

/* Sensor status. */
enum {
	ON,
	OFF
};
static uint8_t state = OFF;

/**
 * sht21_soft_reset
 */
static void
sht21_soft_reset(void)
{
	uint8_t cmd_reset_reg[1] = { SHT21_CMD_RESET };
	I2C(cmd_reset_reg, 1, NULL, 0);
	clock_wait(3); /* Takes 15ms, here 3 * 1 / 125 = 24ms. */
}

/**
 * sht21_sn
 *
 * @note	CRC check of returned data is not implemented.
 */
static void
sht21_sn(void)
{
	uint8_t serial_number[8];
	uint8_t cmd_read_sn_pt1[2] = { 0xfa, 0x0f };
	uint8_t cmd_read_sn_pt2[2] = { 0xfc, 0xc9 };
	uint8_t read_buffer[8];

	/* Part 1 of serial number. */
	memset(read_buffer, 0, sizeof(read_buffer));
	I2C(cmd_read_sn_pt1, 2, read_buffer, 8);

	serial_number[5] = read_buffer[0]; /* SNB_3 */
	serial_number[4] = read_buffer[2]; /* SNB_2 */
	serial_number[3] = read_buffer[4]; /* SNB_1 */
	serial_number[2] = read_buffer[6]; /* SNB_0 */

	/* Part 2 of serial number. */
	memset(read_buffer, 0, sizeof(read_buffer));
	I2C(cmd_read_sn_pt2, 2, read_buffer, 6);
	serial_number[1] = read_buffer[0]; /* SNC_1 */
	serial_number[0] = read_buffer[1]; /* SNC_0 */
	serial_number[7] = read_buffer[3]; /* SNA_1 */
	serial_number[6] = read_buffer[4]; /* SNA_0 */

	printf_P(PSTR("SHT21 S/N %02x %02x %02x %02x %02x %02x %02x %02x\n"),
		serial_number[7], serial_number[6], serial_number[5], serial_number[4],
		serial_number[3], serial_number[2], serial_number[1], serial_number[0]);
}

/**
 * sht21_heat
 */
static int
sht21_heat(int on)
{
	int rc;

	uint8_t cmd_read_user_reg[1] = { SHT21_CMD_READ_USER_REG };
	uint8_t cmd_write_user_reg[2] = { SHT21_CMD_WRITE_USER_REG, 0x00 };

	/* Read the User Register into the second byte of cmd_write_user_reg. */
	rc = I2C(cmd_read_user_reg, 1, &cmd_write_user_reg[1], 1);
	if (rc)
		return rc;

	/* Mask in Heat setting. */
	if (on) {
		cmd_write_user_reg[1] |= SHT21_HEAT_ON;
	} else {
		cmd_write_user_reg[1] &= SHT21_HEAT_OFF;
	}

	/* Write the User Register back to the SHT21. */
	rc = I2C(cmd_write_user_reg, 2, NULL, 0);

	return rc;
}

/**
 * sht21_relative_humidity
 *
 * Relative humidity is calculated from the  16 bit value read from the
 * SHT21 (with low 2 bits set to 0) with the following formula:
 *
 * 		RH = -6 + 125 * value / 2^16
 *
 * @return	Relative humidity * 100 (centi %) which gives accuracy of
 * 			two decimal places to the caller.
 *
 * @note	CRC check of returned data is not implemented.
 */
static int
sht21_relative_humidity(void)
{
	int rc;
	uint8_t cmd_rh[1] = { SHT21_CMD_RH };
	uint8_t read_buffer[3];
	uint16_t raw_value;
	float humidity;

	/* Read the RH value, 12 bits (default resolution). */
	rc = I2C(cmd_rh, 1, read_buffer, 3);
	if (rc)
		return 0;
	raw_value = read_buffer[0] << 8;
	raw_value |= read_buffer[1];
	raw_value &= 0xfffc; /* Set the status bits to 0. */

	/* Calculate the RH to x.xx precision. */
	humidity = -6.0 + 125.0 * raw_value / 65536;

	return (int)(humidity * 100);
}

/**
 * sht21_temperature
 *
 * 		T = -46.85 + 175.72 * value / 2^16
 *
 * @return	Temperature * 100 (centi DegC) which gives accuracy of
 * 			two decimal places to the caller.
 *
 * @note	CRC check of returned data is not implemented.
 */
static int
sht21_temperature(void)
{
	int rc;
	uint8_t cmd_th[1] = { SHT21_CMD_TEMP };
	uint8_t read_buffer[3];
	uint16_t raw_value;
	float temperature;

	/* Read the T value, 14 bits (default resolution). */
	rc = I2C(cmd_th, 1, read_buffer, 3);
	if (rc)
		return 0;
	raw_value = read_buffer[0] << 8;
	raw_value |= read_buffer[1];
	raw_value &= 0xfffc; /* Set the status bits to 0. */

	/* Calculate the temperature to x.xx precision. */
	temperature = -46.85 + 175.72 * raw_value / 65536;

	return (int)(temperature * 100);
}

#if 0
/**
 * sht21_test
 */
static int
sht21_test(void)
{
	int i;

	printf_P(PSTR("SHT21 Sensor Self Test\n"));

	sht21_soft_reset();
	sht21_sn();

	/* With the heater on the temperature should rise and the humidity
	 * should fall for each of the readings. */
	sht21_heat(1);
	for (i = 0; i < 10; i++) {
		int hu = sht21_relative_humidity();
		int te = sht21_temperature();
		printf_P(PSTR("    Humidity %2d.%02d, Temperature %2d.%02d\n"),
				hu / 100, hu % 100, te / 100, te % 100);
		clock_wait(125*5); /* 5 seconds. */
	}
	sht21_heat(0);

	return 0;
}
#endif

/**
 * value
 *
 * @note	RH samples take around 20ms to complete and not more than 30ms.
 * @note	T samples take around 66ms to complete and not more than 85ms.
 */
static int
value(int type)
{
	switch (type) {
	case SHT21_SENSOR_TEMP:
		return sht21_temperature();
	case SHT21_SENSOR_HUMIDITY:
		return sht21_relative_humidity();
	case SHT21_SENSOR_HEAT_ON:
		sht21_heat(1);
		break;
	case SHT21_SENSOR_HEAT_OFF:
		sht21_heat(0);
		break;
	case SHT21_SENSOR_SN:
		sht21_sn();
		break;
	case SHT21_SENSOR_RESET:
		sht21_soft_reset();
		break;
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
		dprintf("SHT21 Sensor Configure ...\n");
		sht21_soft_reset();
		sht21_heat(0);
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
SENSORS_SENSOR(sht21_sensor, "sht21", value, configure, status);
