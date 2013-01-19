/**
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
 * 		XMEGA Soft 1 Wire Master.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 *
 * @note
 * 		Each 1 Wire Master pin requires a 4.7K pull-up resistor.
 */

#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <avrdef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "ow-master.h"


#define NO_OW_DEVICES		0
#define SOME_OW_DEVICES		1
#define COMPLETED			0
#define RESET_NO_RESPONSE	-1


//#define DEBUG
#ifdef DEBUG
#define dprintf(FORMAT, args...) printf_P(PSTR(FORMAT), ##args)
#else
#define dprintf(...)
#endif


/**
 * ow_write_bit
 */
inline void
ow_write_bit(ow_xmega_master_t *master, char bit)
{
	if (bit) {
		master->port->OUTCLR = master->pin_bm;
		_delay_us(6);
		master->port->OUTSET = master->pin_bm;
		_delay_us(64);
	} else {
		master->port->OUTCLR = master->pin_bm;
		_delay_us(60);
		master->port->OUTSET = master->pin_bm;
		_delay_us(10);
	}
}

/**
 * ow_read_bit
 */
inline char
ow_read_bit(ow_xmega_master_t *master)
{
	int sample;

	master->port->OUTCLR = master->pin_bm;
	_delay_us(6);
	master->port->OUTSET = master->pin_bm;
	_delay_us(9);
	sample = master->port->IN & master->pin_bm;
	_delay_us(55);

	return sample;
}

/**
 * ow_reset
 */
inline int
ow_reset(ow_xmega_master_t *master)
{
	int sample;

	master->port->OUTCLR = master->pin_bm;
	_delay_us(480);
	master->port->OUTSET = master->pin_bm;
	_delay_us(70);
	sample = master->port->IN & master->pin_bm;
	_delay_us(410);

	if (sample) {
		dprintf("OW - reset no devices!\n");
	}

	return sample ? NO_OW_DEVICES : SOME_OW_DEVICES;
}

/**
 * ow_init
 */
int
ow_init(ow_xmega_master_t *master)
{
	master->port->OUTSET = master->pin_bm;
	master->port->DIRSET = master->pin_bm;

	return ow_reset(master);
}

/**
 * ow_transaction
 */
int
ow_transaction(ow_xmega_master_t *master, uint8_t* write, uint8_t write_len,
		uint8_t* read, uint8_t read_len)
{
	int bits;
	uint8_t byte = 0;

	dprintf("OW - transaction on port 0x%04x, pin 0x%02x\n",
			(uint16_t)master->port, master->pin_bm);

	if (ow_reset(master) == 0)
		return RESET_NO_RESPONSE;

	/* Write part of transaction. */
	if (write_len) {
		/* Send data. */
		while (write_len--) {
			byte = *write;
			dprintf("OW -     0x%02x\n", byte);

			bits = 8;
			while (bits--) {
				ow_write_bit(master, byte & 0x01);
				byte >>= 1;
			}

			write++;
		}
	}

	/* Read part of transaction. */
	if (read_len) {
		/* Read data. */
		while (read_len--) {
			bits = 8;
			while (bits--) {
				byte >>= 1;
				if (ow_read_bit(master))
					byte |= 0x80;
			}

			dprintf("OW -           0x%02x\n", byte);
			*read++ = byte;
		}
	}

	return COMPLETED;
}

/**
 *
 */
void
ow_wait_on_read(ow_xmega_master_t *master, uint8_t value)
{
	while ((ow_read_bit(master) & 0x01) != (value & 0x01));
}
