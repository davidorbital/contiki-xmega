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
 * 		SD functions specific to AL-XSLED-EXT.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

#include <stdint.h>
#include <avr/io.h>
#include <spi-xmega.h>

extern int8_t sd_fd;

int sd_arch_init(void *desc)
{
	spi_xmega_slave_t *d = (spi_xmega_slave_t *)desc;

	if (sd_fd < 0) {
		return sd_fd;
	}

	spi_lock(sd_fd);
	d->ss_port->OUTSET = d->ss_bm;

	int i = 10;
	uint8_t data = 0;
	while (--i) {
		spi_write(sd_fd, &data, 1);
	}

	spi_unlock(sd_fd);

	return sd_fd;
}

void sd_arch_spi_write(uint8_t c)
{
	watchdog_periodic();
	spi_write(sd_fd, &c, 1);
}

void sd_arch_spi_write_block(uint8_t *bytes, int amount)
{
	watchdog_periodic();
	spi_write(sd_fd, bytes, amount);
}

unsigned sd_arch_spi_read(void)
{
	uint8_t r;

	watchdog_periodic();
	spi_read(sd_fd, &r, 1);

	return r;
}
