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
 */

#ifndef __DS1820_SENSOR__
#define __DS1820_SENSOR__

#include <lib/sensors.h>

/**
 * DS1820 Sensor Command List.
 */
#define DS1820_SENSOR_TEMP						0
#define DS1820_SENSOR_ROM_CODE					1

/**
 * Export the DS1820 sensor object.
 *
 * Can be called as follows:
 *
 * 		#include <dev/ds1820-sensor.h>
 *
 * 		SENSORS_ACTIVATE(ds1820_sensor);
 *		ds1820_sensor.value(DS1820_SENSOR_TEMP);
 *		ds1820_sensor.value(DS1820_SENSOR_ROM_CODE);
 */
extern const struct sensors_sensor ds1820_sensor;

#endif
