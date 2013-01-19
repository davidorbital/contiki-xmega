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
 * 		SD settings specific to AL-XSLED-EXT.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 */

#ifndef SD_ARCH_H
#define SD_ARCH_H

#include <stdint.h>
#include <spi-xmega.h>


extern int8_t sd_fd;

#ifndef U1IFG
#define U1IFG		IFG2
#endif /* U1IFG */

#define MS_DELAY(x) clock_delay(354 * (x))

/* Machine-dependent macros. */
#define LOCK_SPI()		spi_lock(sd_fd)
#define UNLOCK_SPI()	spi_unlock(sd_fd)
#define SD_CONNECTED()	!(PORTC.IN & PIN3_bm)
#define LOWER_CS()
#define RAISE_CS()

/* Configuration parameters. */
#define SD_TRANSACTION_ATTEMPTS		512
#define SD_READ_RESPONSE_ATTEMPTS	8
#define SD_READ_BLOCK_ATTEMPTS		2
#define SD_MSK_OCR_33				0xc0
#define SD_VCC						SD_MSK_OCR_33

int sd_arch_init(void *desc);
void sd_arch_spi_write(uint8_t c);
void sd_arch_spi_write_block(uint8_t *bytes, int amount);
unsigned sd_arch_spi_read(void);

#endif /* !SD_ARCH_H */
