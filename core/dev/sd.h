/*
 * Copyright (c) 2009, Swedish Institute of Computer Science
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

/**
 * \file
 *	SD driver interface.
 * \author
 * 	Nicolas Tsiftes <nvt@sics.se>
 * 	Timothy Rule <trule.github@nym.hush.com>
 */

#ifndef SD_H
#define SD_H

#include "sd-arch.h"

#define SD_DEFAULT_BLOCK_SIZE			512
#define SD_REGISTER_SIZE				16

/* API return codes. */
#define SD_OK					 		0

#define SD_INIT_ERROR_NO_CARD			-1
#define SD_INIT_ERROR_ARCH				-2
#define SD_INIT_ERROR_NO_IF_COND		-3
#define SD_INIT_ERROR_NO_BLOCK_SIZE    	-4
#define SD_INIT_ERROR_NO_IDLE	    	-5
#define SD_INIT_ERROR_NO_OCR	    	-6
#define SD_INIT_ERROR_NO_OP_IDLE	    -7


#define SD_WRITE_ERROR_NO_CMD_RESPONSE		-20
#define SD_WRITE_ERROR_NO_BLOCK_RESPONSE	-21
#define SD_WRITE_ERROR_PROGRAMMING		-22
#define SD_WRITE_ERROR_TOKEN			-23
#define SD_WRITE_ERROR_NO_TOKEN			-24

#define SD_READ_ERROR_NO_CMD_RESPONSE	-40
#define SD_READ_ERROR_INVALID_SIZE		-41
#define SD_READ_ERROR_TOKEN				-42
#define SD_READ_ERROR_NO_TOKEN			-43


/* Type definition. */
typedef uint32_t sd_offset_t;

/* API */
int sd_initialize(void *desc);
int sd_write(sd_offset_t, unsigned char *, unsigned);
int sd_read(sd_offset_t, unsigned char *, unsigned);
int sd_write_block(sd_offset_t, unsigned char *);
int sd_read_block(sd_offset_t, unsigned char *);
sd_offset_t sd_get_capacity(void);

#endif /* !SD_H */
