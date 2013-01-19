/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 *
 * This file is part of the Contiki operating system.
 *
 */

/* Watchdog routines for the AVR */

/* The default timeout of 2 seconds works on most MCUs.
 * It should be disabled during sleep (unless used for wakeup) since
 * it draws significant current (~5 uamp on 1284p, 20x the MCU consumption).
 *
 * Note the wdt is not properly simulated in AVR Studio 4 Simulator 1:
 *   On many devices calls to wdt_reset will have no effect, and a wdt reboot will occur.
 *   The MCUSR will not show the cause of a wdt reboot.
 *   A 1MHz clock is assumed; at 8MHz timeout occurs 8x faster than it should.
 * Simulator 2 is supposed to work on supported devices (not atmega128rfa1),
 * but neither it nor Studio 5 beta do any resets on the 1284p.
 *
 * Setting WATCHDOG_CONF_TIMEOUT -1 will disable the WDT.
 */

#include <contiki-conf.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <dev/watchdog.h>

#ifndef WATCHDOG_CONF_TIMEOUT
#ifdef __XMEGA__
#define WATCHDOG_CONF_TIMEOUT WDT_PER_2KCLK_gc
#else
	#define WATCHDOG_CONF_TIMEOUT WDTO_2S
#endif
#endif

#ifdef __XMEGA__
#ifndef wdt_disable
#define wdt_disable() \
	uint8_t temp = (WDT.CTRL & ~WDT_ENABLE_bm) | WDT_CEN_bm; \
	CCP = CCP_IOREG_gc; \
	WDT.CTRL = temp
#endif
#endif

 /* While balancing start and stop calls is a good idea, an imbalance will cause
  * resets that can take a lot of time to track down.
  * Some low power protocols may cause this.
  * The default is no balance; define WATCHDOG_CONF_BALANCE 1 to override.
  */
#ifndef WATCHDOG_CONF_BALANCE
#define WATCHDOG_CONF_BALANCE 0
#endif

#if WATCHDOG_CONF_BALANCE && WATCHDOG_CONF_TIMEOUT >= 0
static int stopped = 0;
#endif

/*---------------------------------------------------------------------------*/
void
watchdog_init(void)
{
/*  Clear startup bit and disable the wdt, whether or not it will be used.
    Random code may have caused the last reset.
 */
#ifdef __XMEGA__
	RST.STATUS |= RST_WDRF_bm;
#else
	MCUSR&=~(1<<WDRF);
#endif
	wdt_disable();
#if WATCHDOG_CONF_BALANCE && WATCHDOG_CONF_TIMEOUT >= 0
	stopped = 1;
#endif
}
/*---------------------------------------------------------------------------*/
void
watchdog_start(void)
{
#if WATCHDOG_CONF_TIMEOUT >= 0
#if WATCHDOG_CONF_BALANCE
	stopped--;
	if (stopped)
		return;
#endif
	wdt_enable(WATCHDOG_CONF_TIMEOUT);
#ifdef __XMEGA__
	while (WDT.STATUS & WDT_SYNCBUSY_bm);
#endif
#endif  
}
/*---------------------------------------------------------------------------*/
void
watchdog_periodic(void)
{
#if WATCHDOG_CONF_TIMEOUT >= 0
#if WATCHDOG_CONF_BALANCE
	if(!stopped)
#endif
		wdt_reset();
#endif
}
/*---------------------------------------------------------------------------*/
void
watchdog_stop(void)
{
#if WATCHDOG_CONF_TIMEOUT >= 0
#if WATCHDOG_CONF_BALANCE
	stopped++;
#endif
	wdt_disable();
#endif
}
/*---------------------------------------------------------------------------*/
void
watchdog_reboot(void)
{
	cli();
	wdt_enable(0); /* 0 is the shortest time for all AVR CPU families. */
	while(1);
}
/*---------------------------------------------------------------------------*/
#if 0
/* Not all AVRs implement the wdt interrupt */
ISR(WDT_vect)
{
}
#endif
