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
 *	SD driver implementation using SPI.
 * \author
 * 	Nicolas Tsiftes <nvt@sics.se>
 * 	Timothy Rule <trule.github@nym.hush.com>
 */

#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "sd.h"
#include "sd-arch.h"


//#define DEBUG
#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...)
#endif

#ifndef MIN
#define MIN(a, b)	((a) < (b) ? (a) : (b))
#endif


/* SD commands */
#define GO_IDLE_STATE		0
#define SEND_OP_COND		1
#define SWITCH_FUNC			6
#define SEND_IF_COND		8
#define SEND_CSD			9
#define SEND_CID			10
#define STOP_TRANSMISSION	12
#define SEND_STATUS			13
#define READ_SINGLE_BLOCK	17
#define WRITE_BLOCK			24
#define READ_OCR			58

/* SD response lengths. */
#define R1			1
#define R2			2
#define R3			5
#define R7			5

#define START_BLOCK_TOKEN	0xfe
#define SPI_IDLE			0xff
#define SD_R1_ILLEGAL_CMD	0x04

/* Status codes returned after writing a block. */
#define DATA_ACCEPTED		2
#define DATA_CRC_ERROR		5
#define DATA_WRITE_ERROR	6

static uint16_t rw_block_size;
static uint16_t block_size;

static int read_register(int register_cmd, unsigned char *buf, int register_size);
/*---------------------------------------------------------------------------*/
static int
send_command(uint8_t cmd, uint32_t argument)
{
  uint8_t req[8];

  req[0] = SPI_IDLE;
  req[1] = 0x40 | cmd;
  req[2] = argument >> 24;
  req[3] = argument >> 16;
  req[4] = argument >> 8;
  req[5] = argument;
  req[6] = 0x95; /* The CRC hard-wired to 0x95, used for GO_IDLE_STATE only. */
  req[7] = SPI_IDLE;

  sd_arch_spi_write_block(req, sizeof(req));

  return 0;
}
/*---------------------------------------------------------------------------*/
static uint8_t *
get_response(int length)
{
  int i;
  int x;
  static uint8_t r[R7];

  for(i = 0; i < SD_READ_RESPONSE_ATTEMPTS; i++) {
    x = sd_arch_spi_read();
    if((x & 0x80) == 0) {
      /* A get_response byte is indicated by the MSB being 0. */
      r[0] = x;
      break;
    }
  }

  if(i == SD_READ_RESPONSE_ATTEMPTS) {
    return NULL;
  }

  for(i = 1; i < length; i++) {
    r[i] = sd_arch_spi_read();
  }

  return r;
}
/*---------------------------------------------------------------------------*/
static unsigned char *
transaction(int command, unsigned long argument,
	int response_type, unsigned attempts)
{
  unsigned char *r;

  LOCK_SPI();
  r = NULL;
  while (attempts--) {
    LOWER_CS();
    send_command(command, argument);
    r = get_response(response_type);
    RAISE_CS();
    if(r != NULL) {
      break;
    }
  }
  UNLOCK_SPI();

  return r;
}
/*---------------------------------------------------------------------------*/
int
sd_initialize(void *desc)
{
  unsigned char reg[16];
  int retry;
  uint8_t *r, read_bl_len;
  int sd_card_version;

  /* Check if the SD card is connected. */
  if(SD_CONNECTED() < 0) {
    return SD_INIT_ERROR_NO_CARD;
  }

  /* Initialise the SD-SPI bus. The call to sd_arch_init should set the
   * clock speed to 400KHz, lock access to the device if necessary, set
   * /SS high and then send 80 clocks to allow the SD card to initialise. */
  if(sd_arch_init(desc) < 0) {
    return SD_INIT_ERROR_ARCH;
  }

  /* Tell card to enter idle state. */
  r = transaction(GO_IDLE_STATE, 0, R1, SD_TRANSACTION_ATTEMPTS);
  if((r == NULL) || !(r[0] & 0x01)) {
	  dprintf("Failed to get go-idle response\n");
	  return SD_INIT_ERROR_NO_IDLE;
  }
  dprintf("Go idle: %02x\n", r[0]);


  /* Send interface condition to SD card, check version and voltage. */
  r = transaction(SEND_IF_COND, 0, R7, SD_TRANSACTION_ATTEMPTS);
  if(r == NULL) {
	  dprintf("Failed to get IF cond response\n");
	  return SD_INIT_ERROR_NO_IF_COND;
  }
  dprintf("IF cond: %d %d %d %d %d\n", r[0], r[1], r[2], r[3], r[4]);
  sd_card_version = (r[0] & SD_R1_ILLEGAL_CMD) ? 1 : 2;
  dprintf("SD card version is %d\n", sd_card_version);
  if(sd_card_version == 2) {
	  // TODO check pattern error r[1]

	  // TODO check voltage r[2]

  }

  /* Read OCR. */
  r = transaction(READ_OCR, 0, R3, SD_TRANSACTION_ATTEMPTS);
  if(r == NULL) {
    dprintf("Failed to get OCR response\n");
    return SD_INIT_ERROR_NO_OCR;
  }
  dprintf("OCR: %d %d %d %d %d\n", r[0], r[1], r[2], r[3], r[4]);
  if(r[0] & SD_R1_ILLEGAL_CMD) {
    dprintf("OCR indicated not an SD card, ignore\n");
    /* Ignore error condition and continue. */
  }
  if ((r[2] & SD_VCC) != SD_VCC) {
    dprintf("OCR indicated not a supported voltage, ignore\n");
    /* Ignore error condition and continue. */
  }

  /* Send OP with HCS 0. */
  retry = SD_TRANSACTION_ATTEMPTS;
  while (--retry) {
    r = transaction(SEND_OP_COND, 0, R1, 1);
    if (r) {
    	  dprintf("OP cond: %02x\n", r[0]);
    }
    if(r && !(r[0] & 1)) {
    	break;
    }
  }
  if(retry == 0) {
	  dprintf("Failed to get OP cond not idle response\n");
	  return SD_INIT_ERROR_NO_OP_IDLE;
  }
  if(r && (r[0] & SD_R1_ILLEGAL_CMD)) {
    dprintf("OP indicated not an SD card, ignore\n");
    /* Ignore error condition and continue. */
  }

  /* Read CSD register, set block size. */
  if(read_register(SEND_CSD, reg, sizeof(reg)) < 0) {
    dprintf("Failed to get block size of SD card\n");
    return SD_INIT_ERROR_NO_BLOCK_SIZE;
  }
  dprintf("CSD: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		  reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7]);
  dprintf("     %02x %02x %02x %02x %02x %02x %02x %02x\n",
		  reg[8], reg[9], reg[10], reg[11], reg[12], reg[13], reg[14], reg[15]);

  read_bl_len = reg[5] & 0x0f;
  block_size = 1 << read_bl_len;
  rw_block_size = (block_size > SD_DEFAULT_BLOCK_SIZE) ?
			SD_DEFAULT_BLOCK_SIZE : block_size;
  dprintf("Found block size %d, using block size %d\n", block_size, rw_block_size);

// TODO send CID (10) card identification data

// TODO check if SD is read only.

  /* FIXME Arbitrary wait time here. Need to investigate why this is needed. */
  MS_DELAY(5);

  return SD_OK;
}
/*---------------------------------------------------------------------------*/
int
sd_write_block(sd_offset_t offset, unsigned char *buf)
{
  unsigned char *r;
  int retval;
  int i;
  unsigned char data_response;
  unsigned char status_code;

  LOCK_SPI();
  r = NULL;
  retval = SD_WRITE_ERROR_NO_CMD_RESPONSE;
  for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
    LOWER_CS();
    send_command(WRITE_BLOCK, offset);
    r = get_response(R1);
    if(r != NULL) {
      break;
    }
    RAISE_CS();
  }

  if(r != NULL && r[0] == 0) {
    /* We received an R1 response with no errors.
       Send a start block token to the card now. */
    sd_arch_spi_write(START_BLOCK_TOKEN);

    /* Write the data block. */
    sd_arch_spi_write_block(buf, rw_block_size);

    /* Get a response from the card. */
    retval = SD_WRITE_ERROR_NO_BLOCK_RESPONSE;
    for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
      data_response = sd_arch_spi_read();
      if((data_response & 0x11) == 1) {
        /* Data response token received. */
        status_code = (data_response >> 1) & 0x7;
        if(status_code == DATA_ACCEPTED) {
          retval = rw_block_size;
        } else {
          retval = SD_WRITE_ERROR_PROGRAMMING;
        }
	break;
      }
    }
  }

  RAISE_CS();
  UNLOCK_SPI();

  return retval;
}
/*---------------------------------------------------------------------------*/
static int
read_block(unsigned read_cmd, sd_offset_t offset, unsigned char *buf, int len)
{
  unsigned char *r;
  int i;
  int token;
  int retval;

  LOCK_SPI();

  r = NULL;
  for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
    LOWER_CS();
    send_command(read_cmd, offset);
    r = get_response(R1);
    if(r != NULL) {
      break;
    }
    RAISE_CS();
  }

  if(r != NULL && r[0] == 0) {
    /* We received an R1 response with no errors.
       Get a token from the card now. */
    for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
      token = sd_arch_spi_read();
      if(token == START_BLOCK_TOKEN || (token > 0 && token <= 8)) {
        break;
      }
    }

    if(token == START_BLOCK_TOKEN) {
      /* A start block token has been received. Read the block now. */
      for(i = 0; i < len; i++) {
        buf[i] = sd_arch_spi_read();
      }

      /* Consume CRC. TODO: Validate the block. */
      sd_arch_spi_read();
      sd_arch_spi_read();

      retval = len;
    } else if(token > 0 && token <= 8) {
      /* The card returned a data error token. */
      retval = SD_READ_ERROR_TOKEN;
    } else {
      /* The card never returned a token after our read attempts. */
      retval = SD_READ_ERROR_NO_TOKEN;
    }

    RAISE_CS();
    UNLOCK_SPI();
    return retval;
  }

  RAISE_CS();
  UNLOCK_SPI();

  if(r != NULL) {
    dprintf("status during read: %d\n", r[0]);
  }

  return SD_READ_ERROR_NO_CMD_RESPONSE;
}
/*---------------------------------------------------------------------------*/
int
sd_read_block(sd_offset_t offset, unsigned char *buf)
{
  return read_block(READ_SINGLE_BLOCK, offset, buf, rw_block_size);
}
/*---------------------------------------------------------------------------*/
static int
read_register(int register_cmd, unsigned char *buf, int register_size)
{
  return read_block(register_cmd, 0, buf, register_size);
}
/*---------------------------------------------------------------------------*/
sd_offset_t
sd_get_capacity(void)
{
  unsigned char reg[16];
  int r;
  uint16_t c_size;
  uint8_t c_size_mult;
  sd_offset_t block_nr;
  sd_offset_t mult;

  r = read_register(SEND_CSD, reg, sizeof(reg));
  if(r < 0) {
    return r;
  }

  c_size = ((reg[6] & 3) << 10) + (reg[7] << 2) + ((reg[8] >> 6) & 3);
  c_size_mult = ((reg[9] & 3) << 1) + ((reg[10] & 0x80) >> 7);
  mult = 1 << (c_size_mult + 2);
  block_nr = (c_size + 1) * mult;

  return block_nr * block_size;
}
/*---------------------------------------------------------------------------*/
int
sd_write(sd_offset_t offset, unsigned char *buf, size_t size)
{
  sd_offset_t address;
  uint16_t offset_in_block;
  int r, i;
  size_t written;
  size_t to_write;
  unsigned char sd_buf[rw_block_size];

  /* Emulation of data writing using arbitrary offsets and chunk sizes. */
  memset(sd_buf, 0, sizeof(sd_buf));
  written = 0;
  offset_in_block = offset & (rw_block_size - 1);

  do {
    to_write = MIN(size - written, rw_block_size - offset_in_block);
    address = (offset + written) & ~(rw_block_size - 1);

    for(i = 0; i < SD_READ_BLOCK_ATTEMPTS; i++) {
      r = sd_read_block(address, sd_buf);
      if(r == sizeof(sd_buf)) {
	break;
      }
    }
    if(r != sizeof(sd_buf)) {
      return r;
    }

    memcpy(&sd_buf[offset_in_block], &buf[written], to_write);
    r = sd_write_block(address, sd_buf);
    if(r != sizeof(sd_buf)) {
      return r;
    }
    written += to_write;
    offset_in_block = 0;
  } while(written < size);

  return written;
}
/*---------------------------------------------------------------------------*/
int
sd_read(sd_offset_t offset, unsigned char *buf, size_t size)
{
  size_t total_read;
  size_t to_read;
  unsigned char sd_buf[rw_block_size];
  uint16_t offset_in_block;
  int r, i;

  /* Emulation of data reading using arbitrary offsets and chunk sizes. */
  total_read = 0;
  offset_in_block = offset & (rw_block_size - 1);

  do {
    to_read = MIN(size - total_read, rw_block_size - offset_in_block);
    for(i = 0; i < SD_READ_BLOCK_ATTEMPTS; i++) {
      r = sd_read_block((offset + total_read) & ~(rw_block_size - 1), sd_buf);
      if(r == sizeof(sd_buf)) {
	break;
      }
    }
    if(r != sizeof(sd_buf)) {
      return r;
    }

    memcpy(&buf[total_read], &sd_buf[offset_in_block], to_read);
    total_read += to_read;
    offset_in_block = 0;
  } while(total_read < size);

  return size;
}
/*---------------------------------------------------------------------------*/
