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
 * 		Platform for AL-XSLED-EXT.
 * @author
 * 		Timothy Rule <trule.github@nym.hush.com>
 * 		based on work from:
 * 		Nicolas Tsiftes <nvt@sics.se>
 */

#include <string.h>
#include <sd.h>
#include "cfs-coffee-arch.h"


/**
 * cfs_coffee_arch_erase
 *
 * Note, this erases by writing 0's. Depending on media writing 1's may be
 * more appropriate.
 */
int cfs_coffee_arch_erase(unsigned sector)
{
	char buf[SD_DEFAULT_BLOCK_SIZE];
	sd_offset_t start_offset;
	sd_offset_t end_offset;
	sd_offset_t offset;

	memset(buf, 0, sizeof(buf));

	start_offset = COFFEE_START + sector * COFFEE_SECTOR_SIZE;
	end_offset = start_offset + COFFEE_SECTOR_SIZE;

	for (offset = start_offset; offset < end_offset;
			offset += SD_DEFAULT_BLOCK_SIZE) {
		if (sd_write(offset, (unsigned char *)buf, sizeof(buf)) < 0) {
			return -1;
		}
		watchdog_periodic();
	}
	return 0;
}
