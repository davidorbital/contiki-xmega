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

/**
 * SHT21 Overview.
 *
 * Pin	Function
 * ==================
 * 1	SDA
 * 2	VSS, Ground
 * 3	NC
 * 4	NC
 * 5	VDD, 2.1-3.6v
 * 6	SCL
 *
 * I2C address = 0b1000 000x (x = 1 read, = 0 write)
 *
 * Commands:
 * 		T measure with hold		0xE3
 * 		RH measure with hold	0xE5
 * 		T measure no hold		0xF3
 * 		RH measure no hold		0xF5
 * 		Write user reg			0xE6
 * 		Read user reg			0xE7
 * 		Soft reset				0xFE
 *
 * 	Hold Mode:
 * 		80(w) ack E5 ack 81(r) ack ... MSB ack LSB ack CSUM nack
 *
 * 	No Hold Mode:
 * 		80 ack F5 ack DATA nack DATA nack MSB ack LSB ack CSUM nack
 *
 *  Lower two bits of LSB represent status;
 *  	bit 1: 0 = T, 1 = RH
 *  	bit 0: not used
 *
 *  User Register:
 *  	bit 7,0		Resolution:	b00 = 12/14 (RH/T) (default)
 *  							b01 = 8/12
 *  							b10 = 10/13
 *  							b11 = 11/11
 *  	6			Status battery: 1 = VDD < 2.25V
 *  	3,4,5		Reserved
 *  	2			On chip heater: 1 = on
 *  	1			Disable OTP reload: 1 = disabled (default)
 *
 *  	Read:
 *  		80 ack E7 ack 81 ack REG nack
 *  	Write
 *  		80 ack E6 ack REG ack
 *
 *  Serial Number:
 *  	64bit, byte format:	SNA_1 SNA_0 SNB_3 SNB_2 SNB_1 SNB_0 SNC_1 SNC_0
 *
 *  	80(w) ack FA ack 0F ack 81(r) ack SNB_3 ack CRC ack SNB_2 ack CRC ack
 *  		SNB_1 ack CRC ack SNB_0 ack CRC nack
 *  	80(w) ack FC ack 0F ack 81(r) ack SNC_1 ack SNC_0 ack CRC ack
 *  		SNA_1 ack SNA_0 ack CRC nack
 */

#ifndef __SHT21_SENSOR__
#define __SHT21_SENSOR__

#include <lib/sensors.h>

/**
 * SHT21 Sensor List.
 */
#define SHT21_SENSOR_TEMP		0
#define SHT21_SENSOR_HUMIDITY	1
#define SHT21_SENSOR_HEAT_ON	2
#define SHT21_SENSOR_HEAT_OFF	3
#define SHT21_SENSOR_SN			4
#define SHT21_SENSOR_RESET		5

/**
 * Export the SHT21 sensor object.
 *
 * Can be called as follows:
 *
 * 		#include <dev/sht21-sensor.h>
 *
 * 		sht21_sensor.value(SHT21_SENSOR_TEMP);		// Temperature in dC
 * 		sht21_sensor.value(SHT21_SENSOR_HUMIDITY);	// Humidity
 */
extern const struct sensors_sensor sht21_sensor;

#endif
