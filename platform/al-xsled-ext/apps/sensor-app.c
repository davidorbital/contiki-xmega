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
 * 		Sensor App.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

#include <stdio.h>
#include <stddef.h>
#include <contiki.h>
#include <antelope.h>
#include <avr/pgmspace.h>
#include <dev/xmega-sensor.h>
#include <dev/sht21-sensor.h>
#include <dev/ds3231-sensor.h>
#include <dev/ds1820-sensor.h>


#define SENSOR_XMEGA_TEMP		1
#define SENSOR_XMEGA_VCC		2
#define SENSOR_SHT_TEMP			3
#define SENSOR_SHT_HUMIDITY		4
#define SENSOR_RTC_TEMP			5
#define SENSOR_DSOW_TEMP		6


//#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif


int sensor_db_init(void);

static struct etimer timer_monitor;
static struct etimer timer_display;

/**
 * sensor_db_init
 */
int sensor_db_init(void)
{
	db_handle_t handle;
	db_result_t result;

	result = db_query(&handle, "SELECT id, name, unit, scale FROM sensor;");
	db_free(&handle);
	if (DB_ERROR(result) != 0) {
		dprintf("DB creating sensor table...\n");

		db_query(NULL, "REMOVE RELATION sensor;");
		db_query(NULL, "CREATE RELATION sensor;");
		db_query(NULL, "CREATE ATTRIBUTE id DOMAIN INT IN sensor;");
		db_query(NULL, "CREATE ATTRIBUTE name DOMAIN STRING(12) IN sensor;");
		db_query(NULL, "CREATE ATTRIBUTE unit DOMAIN STRING(10) IN sensor;");
		db_query(NULL, "CREATE ATTRIBUTE scale DOMAIN INT IN sensor;");
		db_query(NULL, "CREATE INDEX sensor.id TYPE INLINE;");
		db_query(NULL, "CREATE INDEX sensor.name TYPE INLINE;");

		result = db_query(&handle, "SELECT id, name, unit, scale FROM sensor;");
		db_free(&handle);
		if (DB_ERROR(result) != 0) {
			dprintf("DB table init failed with reason : %s\n",
								db_get_result_message(result));
			return -1;
		}
	}

	result = db_query(&handle, "SELECT sensor_id, time, value FROM sample;");
	db_free(&handle);
	if (DB_ERROR(result) != 0) {
		dprintf("DB creating sample table...\n");

		db_query(NULL, "REMOVE RELATION sample;");
		db_query(NULL, "CREATE RELATION sample;");
		db_query(NULL, "CREATE ATTRIBUTE sensor_id DOMAIN INT IN sample;");
		db_query(NULL, "CREATE ATTRIBUTE time DOMAIN LONG IN sample;");
		db_query(NULL, "CREATE ATTRIBUTE value DOMAIN LONG IN sample;");
		db_query(NULL, "CREATE INDEX sample.sensor_id TYPE INLINE;");
		db_query(NULL, "CREATE INDEX sample.time TYPE INLINE;");

		result = db_query(&handle, "SELECT sensor_id, time, value FROM sample;");
		db_free(&handle);
		if (DB_ERROR(result) != 0) {
			dprintf("DB table init failed with reason : %s\n",
					db_get_result_message(result));
			return -2;
		}
	}

	return 0;
}

/**
 * check_db_result
 */
static int
check_db_result(db_handle_t *handle, db_result_t *result)
{
	if (result == NULL) {
		if (handle)
			db_free(handle);
		return -1;
	}

	if (DB_ERROR(*result) != 0) {
		dprintf("DB failed with reason : %s\n", db_get_result_message(*result));
		if (handle)
			db_free(handle);
		return -2;
	}

	return 0;
}

/**
 * create_sensor
 */
static int
create_sensor(int id, const char *name, const char *unit, int scale)
{
	int rc;
	db_handle_t handle;
	db_result_t result;
	attribute_value_t value;

	value.u.int_value = 0;

	/* Check if sensor exists. */
	dprintf("DB find sensor\n");
	result = db_query(&handle, "SELECT COUNT(id) FROM sensor WHERE id = %d;", id);
	rc = check_db_result(&handle, &result);
	if (rc != 0)
		return -1;

	while (db_processing(&handle)) {
		result = db_process(&handle);
		switch (result) {
		case DB_OK:
			continue;
		case DB_GOT_ROW:
			result = db_get_value(&value, &handle, 0);
			rc = check_db_result(&handle, &result);
			if (rc != 0)
				return -2;
		case DB_FINISHED:
		default:
			rc = check_db_result(&handle, &result);
			if (rc != 0)
				return -3;
			db_free(&handle);
			break;
		}
	}

	dprintf("DB sensor count is %d\n", (int)VALUE_INT(&value));
	if ((int)VALUE_INT(&value) > 0)
		return 0;

	dprintf("DB add sensor\n");

	/* Add sensor. */
	result = db_query(NULL, "INSERT (%d, \'%s\', \'%s\', %d) INTO sensor;",
			id, name, unit, scale);
	rc = check_db_result(NULL, &result);
	if (rc != 0)
		return -4;

	return 0;
}

/**
 * parse_samples_since
 */
static int
parse_samples_since(long since_time)
{
	int rc;
	db_handle_t handle;
	db_result_t result;
	attribute_value_t sensor_id;
	attribute_value_t time;
	attribute_value_t value;

	sensor_id.u.int_value = 0;
	time.u.long_value = 0;
	value.u.long_value = 0;

	/* Check if sensor exists. */
	dprintf("DB get last sample\n");
	result = db_query(&handle, "SELECT sensor_id, time, value "
			"FROM sample WHERE time > %ld;", since_time);
	rc = check_db_result(&handle, &result);
	if (rc != 0)
		return -1;

	while (db_processing(&handle)) {
		result = db_process(&handle);
		switch (result) {
		case DB_OK:
			continue;
		case DB_GOT_ROW:
			result = db_get_value(&sensor_id, &handle, 0);
			rc = check_db_result(&handle, &result);
			if (rc != 0)
				return -2;

			result = db_get_value(&time, &handle, 1);
			rc = check_db_result(&handle, &result);
			if (rc != 0)
				return -2;

			result = db_get_value(&value, &handle, 2);
			rc = check_db_result(&handle, &result);
			if (rc != 0)
				return -3;

			printf_P(PSTR("sensor/value/time:%d,%ld,%ld\n"),
					(int)VALUE_INT(&sensor_id), (long)VALUE_LONG(&value),
					(long)VALUE_LONG(&time));
			// TODO implement printing with scale as follows:
			//printf_P(PSTR("    Humidity %2d.%02d, Temperature %2d.%02d, Temperature %2d.%02d\n"),
			//		hu / 100, hu % 100, te / 100, te % 100, te_rtc / 100, te_rtc % 100);
			continue;
		case DB_FINISHED:
		default:
			rc = check_db_result(&handle, &result);
			if (rc != 0)
				return -4;
			db_free(&handle);
			break;
		}
	}

	return 0;
}

/**
 * add_sample
 */
static void
add_sample(int sensor_id, long value)
{
	db_result_t result;
	unsigned long time = clock_seconds();

	dprintf("DB add sample %d %ld %ld\n", sensor_id, time, value);
	result = db_query(NULL, "INSERT (%d, %ld, %ld) INTO sample;",
			sensor_id, time, value);
	check_db_result(NULL, &result);
}

PROCESS(sensor_app_monitor_process, "Sensor App");
PROCESS_THREAD(sensor_app_monitor_process, ev, data)
{
	PROCESS_BEGIN();

	dprintf("Sensor App ready to read ...\n");

	/* Enable XMega on-chip sensors. */
	SENSORS_ACTIVATE(xmega_sensor);
	SENSORS_ACTIVATE(sht21_sensor);
	SENSORS_ACTIVATE(ds3231_sensor);
	SENSORS_ACTIVATE(ds1820_sensor);

	/* Initialise storage database. */
	if (create_sensor(SENSOR_XMEGA_TEMP, "XMEGA_TEMP", "degC", 10) != 0)
		goto process_end;
	if (create_sensor(SENSOR_XMEGA_VCC, "XMEGA_VCC", "mV", 1) != 0)
		goto process_end;
	if (create_sensor(SENSOR_SHT_TEMP, "SHT_TEMP", "degC", 100) != 0)
		goto process_end;
	if (create_sensor(SENSOR_SHT_HUMIDITY, "SHT_HUM", "RH%", 100) != 0)
		goto process_end;
	if (create_sensor(SENSOR_RTC_TEMP, "RTC_TEMP", "degC", 100) != 0)
		goto process_end;
	if (create_sensor(SENSOR_DSOW_TEMP, "DSOW_TEMP", "degC", 100) != 0)
		goto process_end;

	/* Start the measurement loop, each 10 seconds. */
	etimer_set(&timer_monitor, CLOCK_SECOND * 10);

#ifdef SENSOR_APP_DEBUG
static int i = 3;
	while (i) {
#else
	while (1) {
#endif
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		etimer_reset(&timer_monitor);

		add_sample(SENSOR_XMEGA_TEMP, xmega_sensor.value(XMEGA_SENSOR_TEMP));
		add_sample(SENSOR_XMEGA_VCC, xmega_sensor.value(XMEGA_SENSOR_VCC));
		add_sample(SENSOR_SHT_TEMP, sht21_sensor.value(SHT21_SENSOR_TEMP));
		add_sample(SENSOR_SHT_HUMIDITY, sht21_sensor.value(SHT21_SENSOR_HUMIDITY));
		add_sample(SENSOR_RTC_TEMP, ds3231_sensor.value(DS3231_SENSOR_TEMP));
		add_sample(SENSOR_DSOW_TEMP, ds1820_sensor.value(DS1820_SENSOR_TEMP));

#ifdef SENSOR_APP_DEBUG
		i--;
#endif
	}

process_end:
	; /* Stops compile error. */
	PROCESS_END();
}

PROCESS(sensor_app_display_process, "Sensor App Display");
PROCESS_THREAD(sensor_app_display_process, ev, data)
{
	PROCESS_BEGIN();

	/* Start the display loop, each 10 seconds. */
	etimer_set(&timer_display, CLOCK_SECOND * 10);

#ifdef SENSOR_APP_DEBUG
static int i = 3;
	while (i) {
#else
	while (1) {
#endif
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		etimer_reset(&timer_display);
		parse_samples_since(clock_seconds() - 10);

#ifdef SENSOR_APP_DEBUG
		i--;
#endif
	}

	PROCESS_END();
}
