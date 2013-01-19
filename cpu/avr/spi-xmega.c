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
 * 		SPI Multi Slave support on AVR XMEGA.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

/**
 * Instructions for use.
 *
 * In platform code:
 * 	1/ Include <spi-xmega.h>.
 * 	2/ Define array of spi_xmega_slave_t for each slave.
 * 	3/ Call spi_init and spi_open, pass returned "fd" to library init func's.
 *
 * In cpu code:
 * 	1/ Add AVR += spi-xmega.c to avr/Makefile.xmega.
 *
 * In library code:
 * 	1/ Using "fd" from init, call spi_lock, spi_write, spi_read & spi_unlock.
 */

#include <stdint.h>
#include <avr/io.h>
#include <spi-xmega.h>

#define IDLE	0
#define OPEN	0x01
#define BUSY	0x02


static spi_xmega_slave_t *slave_array;
static uint8_t slave_count;


/**
 * spi_init_multi
 *
 * @brief		Initialise internal data structures.
 * @param desc	Pointer to array of spi_xmega_slave_t devices.
 * @param count	Number of items in spi_xmega_slave_t array.
 * @return		0 = success, -ve = error.
 */
int spi_init_multi(void *desc, uint8_t count)
{
	slave_array = (spi_xmega_slave_t *)desc;
	slave_count = count;

	return 0;
}

/**
 * spi_open
 *
 * @brief		Open the slave.
 * @param desc	Pointer to spi_xmega_slave_t which should be in slave_array.
 * @return		Index of slave in internal slave_array, -ve = error.
 */
int8_t spi_open(void *desc)
{
	int i;
	int8_t fd = -1;
	spi_xmega_slave_t *p;
	spi_xmega_slave_t *d = (spi_xmega_slave_t *)desc;

	/* Search for the slave in slave_array. */
	for (i = 0, p = slave_array; i < slave_count; i++,p++) {
		if (p == d) {
			fd = i; /* Save the return value. */
		}
	}
	if (fd < 0) {
		return fd;
	}

	/* Enable the master and open this slave. */
	if (!(*d->ctrl & SPI_ENABLE_bm)) {
		d->port->DIRSET = PIN5_bm | PIN7_bm;
		*d->ctrl = SPI_ENABLE_bm | SPI_MASTER_bm;
	}
	d->ss_port->DIRSET = d->ss_bm;
	d->ss_port->OUTSET = d->ss_bm;
	d->state = OPEN;

	return fd;
}

/**
 * spi_close
 *
 * @brief		Close the slave and disable master, if no longer in use.
 * @param fd	Index to internal slave_array.
 */
void spi_close(int8_t fd)
{
	int i;
	spi_xmega_slave_t *p;
	spi_xmega_slave_t *d = &slave_array[fd];

	/* Set the slave to idle. */
	d->state = IDLE;
	d->ss_port->DIRCLR = d->ss_bm;

	/* Search for a non idle slave on this master. */
	for (i = 0, p = slave_array; i < slave_count; i++,p++) {
		if ((p->port == d->port) && (p->state != IDLE)) {
			return; /* Master is still in use by another slave. */
		}
	}

	/* Disable the master (of this slave). */
	*d->ctrl = 0;
	d->port->DIRCLR = PIN5_bm | PIN7_bm;
}

/**
 * spi_lock
 *
 * @brief		Release /SS and set slave state to busy, but only if master is
 * 				not already in use.
 * @param fd	Index to internal slave_array.
 * @return		0 = successful, -ve = error.
 */
int spi_lock(int8_t fd)
{
	int i;
	spi_xmega_slave_t *d = &slave_array[fd];
	spi_xmega_slave_t *p;

	/* Search for a busy slave on this master (d). */
	for (i = 0, p = slave_array; i < slave_count; i++,p++) {
		if (p == d) {
			continue;
		}
		if ((p->port == d->port) && (p->state & BUSY)) {
			return -1; /* Master is busy. */
		}
	}

	/* Lock the master for this slave. */
	d->state |= BUSY;
	*d->ctrl = SPI_ENABLE_bm | SPI_MASTER_bm | d->ctrl_bm;
	d->ss_port->OUTCLR = d->ss_bm;

	return 0;
}

/**
 * spi_unlock
 *
 * @brief		Release /SS and set slave state to not busy.
 * @param fd	Index to internal slave_array.
 */
void spi_unlock(int8_t fd)
{
	spi_xmega_slave_t *d = &slave_array[fd];

	d->ss_port->OUTSET = d->ss_bm;
	d->state ^= BUSY;
}

/**
 * spi_write
 *
 * @brief		Write byte stream to slave device, ignores response bytes.
 * @param fd	Index to internal slave_array.
 * @param data	Pointer to array of bytes to be written to slave.
 * @param site	Number of bytes in array that should be written to slave.
 */
void spi_write(int8_t fd, uint8_t *data, int size)
{
	spi_xmega_slave_t *d = &slave_array[fd];

	while (size--) {
		*d->data = *data;
		while (!(*d->status & SPI_IF_bm));
		data++;
	}
}

/**
 * spi_read
 *
 * @brief		Read byte stream from slave device.
 * @param fd	Index to internal slave_array.
 * @param data	Pointer to array of bytes to be read into from slave.
 * @param site	Number of bytes in array that should be read into from slave.
 */
void spi_read(int8_t fd, uint8_t *data, int size)
{
	spi_xmega_slave_t *d = &slave_array[fd];

	while (size--) {
		*d->data = 0;
		while (!(*d->status & SPI_IF_bm));
		*data = *d->data;
		data++;
	}
}
