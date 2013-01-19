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
 * 		Alarm app for AL-XSLED-EXT using DS3231 connected to Port C with INT
 * 		on Pin 2.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

#include <stdio.h>
#include <contiki.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <dev/ds3231-sensor.h>


//#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif

PROCESS(ds3231_alarm_process, "DS3231 alarm process");
PROCESS_THREAD(ds3231_alarm_process, ev, data)
{
	PROCESS_BEGIN();

	/* Set the RTC time, clear any existing alarm condition. */
	tm t;
	t.tm_year = 112;
	t.tm_mon = 7;
	t.tm_mday = 11;
	t.tm_hour = 20;
	t.tm_min = 4;
	t.tm_sec = 2;
	ds3231_sensor.configure(DS3231_CONFIG_SET_TIME, (int)&t);
	ds3231_sensor.configure(DS3231_CONFIG_CLEAR_ALARM, 0);

	/* Set the alarm to occur in 20 seconds. */
	printf_P(PSTR("Alarm set for 20 seconds.\n"));
	t.tm_sec += 20;
	ds3231_sensor.configure(DS3231_CONFIG_SET_ALARM, (int)&t);

	/* Configure the interrupt line from the RTC, connected on Port C Pin2. */
	PORTC.DIRCLR = PIN2_bm;
	PORTC.PIN2CTRL = (PORTC.PIN2CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc;
	PORTC.INTCTRL = (PORTC.INTCTRL & ~PORT_INT0LVL_gm) | PORT_INT0LVL_LO_gc;
	PORTC.INT0MASK = PIN2_bm;

	/* Enable processor handling of low level/priority interrupts. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm; // enable low level interrupts

	while (1) {
		/* Clear the interrupt. */
		PORTC.INTFLAGS = PORT_INT0IF_bm;

		/* Wait for the interrupt to fire. */
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);

		if (PORTC.INTFLAGS & PORT_INT0IF_bm) {
			printf_P(PSTR("Alarm from RTC fired.\n"));
			break;
		}

		dprintf("Polled for some other reason...\n");
	}

	PROCESS_END();
}

ISR(PORTC_INT0_vect)
{
	process_poll(&ds3231_alarm_process);
}


