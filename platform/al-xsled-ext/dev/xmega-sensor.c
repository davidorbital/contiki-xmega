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
 * 		Sensors for AVR XMEGA.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 * @note
 * 		See application note AVR1300 for further details.
 */

#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <contiki.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "xmega-sensor.h"

//#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif

/* Define the sensor object. */
const struct sensors_sensor xmega_sensor;

/* For storing the XMEGA calibration values. */
static uint16_t calibration_temp;
static uint16_t calibration_adc;

/* Sensor status. */
enum {
	ON,
	OFF
};
static uint8_t state = OFF;

/**
 * adc_read_internal
 */
static uint16_t
adc_read_internal(uint8_t muxctrl, uint16_t calibration)
{
	uint16_t result;

	if (ADCA.CTRLA & ADC_ENABLE_bm) {
		dprintf("ADC is in use!\n");
		return 0;
	}

	/* Setup and enable the ADC. */
	ADCA.CALL = calibration & 0xff;
	ADCA.CALH = (calibration >> 8) & 0xff;
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm | ADC_TEMPREF_bm;
	ADCA.CTRLA |= ADC_ENABLE_bm;

	/* Setup CH0 and start measurement. */
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc;
	ADCA.CH0.MUXCTRL = muxctrl;
	ADCA.CH0.CTRL |= ADC_CH_START_bm;

	/* Wait for complete, get result. */
	while (!ADCA.CH0.INTFLAGS);
	ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;
	result = ADCA.CH0RES;

	/* Disable the ADC*/
	ADCA.CTRLA = 0;

	return result;
}

/**
 * value
 *
 * @note	Some time (ca. 1 sec) needs to pass between calls to
 * xmega_sensor.value(XMEGA_SENSOR_TEMP) as the temperature reading drifts
 * after the first reading.
 */
static int
value(int type)
{
	uint16_t value;
	int32_t result;

	switch (type) {
	case XMEGA_SENSOR_TEMP:
		/*
		 * Calibration is documented at 85¡C however results calculated from
		 * this value are incorrect. Interestingly, using 85¡F as the
		 * calibration value yields results that seem to be correct, therefore
		 * temperature is calculated using calibration value 29.4¡C (85¡F).
		 *
		 * The point in 29.4 forces floating point calculation.
		 *
		 * The value returned is in units d¡C which provides 1 decimal place
		 * of accuracy to the calling function (the sensor interface defines
		 * an int return value).
		 */
		value = adc_read_internal(ADC_CH_MUXINT_TEMP_gc, calibration_temp);
		result = value;
		result = (((29.4 + 273)/calibration_temp) * result - 273) * 10;
		dprintf("Sensor, XMEGA Temp is %d degC\n", result/10);
		return result;
	case XMEGA_SENSOR_VCC:
		value = adc_read_internal(ADC_CH_MUXINT_SCALEDVCC_gc, calibration_adc);
		result = value;
		value = (value - 200)*10*1000/4095;
		dprintf("Sensor, XMEGA VCC is %d mV\n", result);
		return result;
	default:
		return 0;
	}
}

/**
 * configure
 */
static int
configure(int type, int c)
{
	switch (type) {
	case SENSORS_ACTIVE:
		dprintf("XMEGA Sensor Configure ...\n");

		/* Load calibration settings. */
		calibration_temp = xmega_read_calibration_byte(
				offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE0)) & 0xff;
		calibration_temp |= (xmega_read_calibration_byte(
				offsetof(NVM_PROD_SIGNATURES_t, TEMPSENSE1)) << 8);
		calibration_adc = xmega_read_calibration_byte(
				offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0)) & 0xff;
		calibration_adc |= (xmega_read_calibration_byte(
				offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1)) << 8);

		dprintf("XMEGA ADC calibration (temp/adc) %04x : %04x\n",
				calibration_temp, calibration_adc);

		state = ON;
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
SENSORS_SENSOR(xmega_sensor, "xmega", value, configure, status);
