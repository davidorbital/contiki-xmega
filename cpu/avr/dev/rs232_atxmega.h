/*
 * Copyright (c) 2006, Technical University of Munich
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
 * @(#)$$
 */
/**
 * \file
 *         AVR atxmega specific definitions for the UART ports.
 *         This header is much more lighter than mega's one, since
 *         io.h provides enum and constant for all the funcionalities.
 * \author
 *         jacopo mondi <mondi@cs.unibo.it>
 */

#ifndef __RS232_ATXMEGA__
#define __RS232_ATXMEGA__
/******************************************************************************/
/***   Includes                                                               */
/******************************************************************************/
#include <avr/io.h>

/******************************************************************************
***   PORTS OFFSETS ***
******************************************************************************/
/*this is PORTC, the first port with USART*/
#define BASE_USART_PORT   0x0640
#define USART_PORT_OFFSET 0x0020
#define USARTn0_TXD_bm (1<<3)
#define USARTn0_RXD_bm (1<<2)
#define USARTn1_TXD_bm (1<<7)
#define USARTn1_RXD_bm (1<<6)
//XXX MISSING BAUDRATES



/******************************************************************************/
/***   USART ports                                                            */
/******************************************************************************/

#if defined(__AVR_ATxmega32A4__)

#if defined (RS232_0)
#error already defined!
#endif

#define RS232_0 USARTC0
#define RS232_1 USARTC1
#define RS232_2 USARTD0
#define RS232_3 USARTD1
#define RS232_4 USARTE0

#define RS232_USARTC0 0
#define RS232_USARTC1 1
#define RS232_USARTD0 2
#define RS232_USARTD1 3
#define RS232_USARTE0 4

#define RS232_COUNT 5

#elif defined (__AVR_ATxmega256A3__)

#define RS232_0 USARTC0
#define RS232_1 USARTC1
#define RS232_2 USARTD0
#define RS232_3 USARTD1
#define RS232_4 USARTE0
#define RS232_5 USARTE1
#define RS232_6 USARTF0
#define RS232_7 USARTF1

#define RS232_USARTC0 0
#define RS232_USARTC1 1
#define RS232_USARTD0 2
#define RS232_USARTD1 3
#define RS232_USARTE0 4
#define RS232_USARTE1 5
#define RS232_USARTF0 6
#define RS232_USARTF1 7

#define RS232_COUNT 8

#else
#error "Please add RS232 port definitions for Xmega variant."
#endif

/*
 * RS232_PORT_0 is platform specific. Either define it in platform code OR use
 * a call to rs232_redirect_stdout() in early platform init code. For platforms
 * using the latter approach, set a value to satisfy rs232.c.
 */
#ifndef RS232_PORT_0
#define RS232_PORT_0 RS232_USARTC0
#endif

/*
 * XMEGA baud rates. The lower byte goes to BAUDL(BAUDCTRLA) and the higher
 * byte goes to BAUDH(BAUDCTRLB).
 */
#define XMEGA_BAUD_ASYNC_9600			0x000C
#define XMEGA_BAUD_ASYNC_115200			0xA005
#define XMEGA_BAUD_ASYNC_115200_2M		XMEGA_BAUD_ASYNC_115200
#define XMEGA_BAUD_ASYNC_115200_32M		0xA417
#define XMEGA_BAUD_ASYNC_256000_32M		0xA1B4


/******************************************************************************/
/***   Interrupt settings                                                     */
/******************************************************************************/
#define USART_INTERRUPT_RX_COMPLETE USART_RXCINTLVL1_bm
#define USART_INTERRUPT_TX_COMPLETE USART_TXCINTLVL1_bm
#define USART_INTERRUPT_DATA_REG_EMPTY USART_DREINTLVL0_bm

/******************************************************************************/
/***   Receiver / transmitter                                                 */
/******************************************************************************/
#define USART_RECEIVER_ENABLE USART_RXEN_bm
#define USART_TRANSMITTER_ENABLE USART_TXEN_bm

/******************************************************************************/
/***   Mode select                                                            */
/******************************************************************************/
#define USART_MODE_ASYNC USART_CMODE_ASYNCHRONOUS_gc
#define USART_MODE_SYNC USART_CMODE_SYNCHRONOUS_gc

/******************************************************************************/
/***   Parity                                                                 */
/******************************************************************************/
#define USART_PARITY_NONE USART_PMODE_DISABLED_gc
#define USART_PARITY_EVEN USART_PMODE_EVEN_gc
#define USART_PARITY_ODD  USART_PMODE_ODD_gc

/******************************************************************************/
/***   Stop bits                                                              */
/******************************************************************************/
#define USART_STOP_BITS_1 0x00
#define USART_STOP_BITS_2 USART_SBMODE_bm
typedef enum USART_SMODE_enum
{
	USART_SMODE_1BIT_gc = 0x00,
	USART_SMODE_2BIT_gc = (0x01<<3),
} USART_SMODE_t;

/******************************************************************************/
/***   Character size                                                         */
/******************************************************************************/
#define USART_DATA_BITS_5 USART_CHSIZE_5BIT_gc
#define USART_DATA_BITS_6 USART_CHSIZE_6BIT_gc
#define USART_DATA_BITS_7 USART_CHSIZE_7BIT_gc
#define USART_DATA_BITS_8 USART_CHSIZE_8BIT_gc
#define USART_DATA_BITS_9 USART_CHSIZE_9BIT_gc

/******************************************************************************/
/***   Clock polarity                                                         */
/******************************************************************************/
#define USART_RISING_XCKN_EDGE 0x00
#define USART_FALLING_XCKN_EDGE 0x01

#endif /* #ifndef __RS232_ATXMEGA__ */
