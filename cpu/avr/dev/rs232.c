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
 * @(#)$Id: rs232.c,v 1.7 2010/10/27 14:51:20 dak664 Exp $
 * modified on 2011/05/17 jacopo mondi mondi@cs.unibo.it
 */

/* \file
 * 				RS232 implementation for various AVR (currently
 * 				ATmega, ATxmega and AV90USB).
 * 				Features the rs232_port structure where each device maps
 * 				its register in order to provide an uniform point
 * 				of view
 * \author
 * 				2010/10/27 dak664
 * 				2011/5/17 jacopo mondi <mondi@cs.unibo.it>	
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "contiki-conf.h"
#include "contiki.h"

#include "dev/slip.h"
#include "dev/rs232.h"

#include "leds.h"

#ifdef RS232_CONF_PRINTF_BUFFER_LENGTH
#define RS232_PRINTF_BUFFER_LENGTH RS232_CONF_PRINTF_BUFFER_LENGTH
#else
#define RS232_PRINTF_BUFFER_LENGTH 64
#endif

#ifndef ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
#define ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT 1
#endif


/* This is a generic UART structure.
 * Each platform maps corresponding registers into
 * structure elements.
 * This convention (hopefully) works for atmega, atxmega, and at90usb
 */
typedef struct {
  volatile uint8_t * DATA;
  volatile uint8_t * BAUDH;
  volatile uint8_t * BAUDL;
  volatile uint8_t * FUNCTION;
  volatile uint8_t * INTERRUPT;
  volatile uint8_t * FORMAT;
  volatile uint8_t txwait;
  int (* input_handler)(unsigned char);
} rs232_t;

#if defined (__AVR_ATmega128__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega128RFA1__)

static rs232_t rs232_ports[2] = {
  {   // UART0
    &UDR0,
    &UBRR0H,
    &UBRR0L,
    &UCSR0B,
    &UCSR0B,
    &UCSR0C,
    0,
    NULL
  },

  {  // UART1
    &UDR1,
    &UBRR1H,
    &UBRR1L,
    &UCSR1B,
    &UCSR1B,
    &UCSR1C,
    0,
    NULL
  }
};
/*---------------------------------------------------------------------------*/
ISR(USART0_TX_vect)
{
  rs232_ports[RS232_PORT_0].txwait = 0;
}

/*---------------------------------------------------------------------------*/
ISR(USART0_RX_vect)
{
  unsigned char c;

  c = *(rs232_ports[RS232_PORT_0].DATA);

  if(rs232_ports[RS232_PORT_0].input_handler != NULL) {
    rs232_ports[RS232_PORT_0].input_handler(c);
  }
}
/*---------------------------------------------------------------------------*/
ISR(USART1_TX_vect)
{
  rs232_ports[RS232_PORT_1].txwait = 0;
}

/*---------------------------------------------------------------------------*/
ISR(USART1_RX_vect)
{
  unsigned char c;

  c = *(rs232_ports[RS232_PORT_1].DATA);

  if(rs232_ports[RS232_PORT_1].input_handler != NULL) {
    rs232_ports[RS232_PORT_1].input_handler(c);
  }
}

#elif defined (__AVR_AT90USB1287__)
/* Has only UART1, map it to port 0 */
static rs232_t rs232_ports[1] = {
  {  // UART1
    &UDR1,
    &UBRR1H,
    &UBRR1L,
    &UCSR1B,
    &UCSR1B,
    &UCSR1C,
    0,
    NULL
  }
};
/*---------------------------------------------------------------------------*/
ISR(USART1_TX_vect)
{
  rs232_ports[RS232_PORT_0].txwait = 0;
}

/*---------------------------------------------------------------------------*/
ISR(USART1_RX_vect)
{
  unsigned char c;

  c = *(rs232_ports[RS232_PORT_0].DATA);

  if(rs232_ports[RS232_PORT_0].input_handler != NULL) {
    rs232_ports[RS232_PORT_0].input_handler(c);
  }
}
/* 
 * check if we have some type of Xmega, as they should all have the same
 * serial hardware in the same places.
 */
#elif defined (__RS232_ATXMEGA__)
  /* The Xmega header file already contains the USART_t
   * type definition*/
  #define XMEGA_SERIAL_PORT_COUNT 0

  static rs232_t rs232_ports[] = {

  #if defined(RS232_0)
  {
    &RS232_0.DATA,
    &RS232_0.BAUDCTRLB,
    &RS232_0.BAUDCTRLA,
    &RS232_0.CTRLB,
    &RS232_0.CTRLA,
    &RS232_0.CTRLC,
    0,
    NULL
  },
  #endif

  #if defined(RS232_1)
  {
    &RS232_1.DATA,
    &RS232_1.BAUDCTRLB,
    &RS232_1.BAUDCTRLA,
    &RS232_1.CTRLB,
    &RS232_1.CTRLA,
    &RS232_1.CTRLC,
    0,
    NULL
  },
  #endif
  #if defined(RS232_2)
  {
    &RS232_2.DATA,
    &RS232_2.BAUDCTRLB,
    &RS232_2.BAUDCTRLA,
    &RS232_2.CTRLB,
    &RS232_2.CTRLA,
    &RS232_2.CTRLC,
    0,
    NULL
  },
  #endif
  #if defined(RS232_3)
  {
    &RS232_3.DATA,
    &RS232_3.BAUDCTRLB,
    &RS232_3.BAUDCTRLA,
    &RS232_3.CTRLB,
    &RS232_3.CTRLA,
    &RS232_3.CTRLC,
    0,
    NULL
  },
  #endif
  #if defined(RS232_4)
  {
    &RS232_4.DATA,
    &RS232_4.BAUDCTRLB,
    &RS232_4.BAUDCTRLA,
    &RS232_4.CTRLB,
    &RS232_4.CTRLA,
    &RS232_4.CTRLC,
    0,
    NULL
  },
  #endif
  #if defined(RS232_5)
  {
    &RS232_5.DATA,
    &RS232_5.BAUDCTRLB,
    &RS232_5.BAUDCTRLA,
    &RS232_5.CTRLB,
    &RS232_5.CTRLA,
    &RS232_5.CTRLC,
    0,
    NULL
  },
  #endif
  #if defined(RS232_6)
  {
    &RS232_6.DATA,
    &RS232_6.BAUDCTRLB,
    &RS232_6.BAUDCTRLA,
    &RS232_6.CTRLB,
    &RS232_6.CTRLA,
    &RS232_6.CTRLC,
    0,
    NULL
  },
  #endif
  #if defined(RS232_7)
  {
    &RS232_7.DATA,
    &RS232_7.BAUDCTRLB,
    &RS232_7.BAUDCTRLA,
    &RS232_7.CTRLB,
    &RS232_7.CTRLA,
    &RS232_7.CTRLC,
    0,
    NULL
  },
  #endif
};



/*-- interrupt level HIGH for transmit, empty and receive --*/
// USART_RXCINTLVL_t USART_INTERRUPT_RX_COMPLETE = 
// 						USART_RXCINTLVL_HI_gc;
// USART_TXCINTLVL_t USART_INTERRUPT_TX_COMPLETE =
// 						USART_TXCINTLVL_HI_gc;

/*=== Interrupt Routines ===*/

/* generic ISR versions */
void inline usart_generic_tx_vect(uint8_t portnum){
   //PORTE.OUT ^= (1<<1);
  rs232_ports[portnum].txwait = 0;

}

void inline usart_generic_rx_vect(uint8_t portnum){

  unsigned char c;

  c = *(rs232_ports[portnum].DATA);

  if(rs232_ports[portnum].input_handler != NULL) {
    rs232_ports[portnum].input_handler(c);
  }
}


/* ISR declatations pointing at the general version */
/*---------------------------------------------------------------------------*/

ISR(USARTE0_DRE_vect) { usart_generic_tx_vect(RS232_USARTE0);}
ISR(USARTE0_TXC_vect) { usart_generic_tx_vect(RS232_USARTE0);}

ISR(USARTE0_RXC_vect) { usart_generic_rx_vect(RS232_USARTE0); }
ISR(USARTD0_TXC_vect) { usart_generic_tx_vect(RS232_USARTD0); }
ISR(USARTD0_RXC_vect) { usart_generic_rx_vect(RS232_USARTD0); }
ISR(USARTD1_TXC_vect) { usart_generic_tx_vect(RS232_USARTD1); }
ISR(USARTD1_RXC_vect) { usart_generic_rx_vect(RS232_USARTD1); }
ISR(USARTC0_TXC_vect) { usart_generic_tx_vect(RS232_USARTC0); }
ISR(USARTC0_RXC_vect) { usart_generic_rx_vect(RS232_USARTC0); }
ISR(USARTC1_TXC_vect) { usart_generic_tx_vect(RS232_USARTC1); }
ISR(USARTC1_RXC_vect) { usart_generic_rx_vect(RS232_USARTC1); }


#else  /* end xmega*/
#error Please define the UART registers for your MCU!
#endif

/*---------------------------------------------------------------------------*/
void
rs232_init (uint8_t port, uint16_t bd, uint8_t ffmt)
{
	rs232_t *rs232 = &rs232_ports[port];

#if defined (__RS232_ATXMEGA__)
	/* 
	 * we need same index for even and odd 
	 * port numbers, so we clear the LSB.
	 * This number must then be divide by 2 to get the distance
	 * (in USART_PORT_OFFSET) from first USART
	 * port (BASE_USART_PORT)
	 */
	uint8_t port_index = ((port&0xfe)/2);
	PORT_t *USART_port = (PORT_t *) (BASE_USART_PORT + 
			(USART_PORT_OFFSET*port_index));
	/*
	 * We need to set TX pin as output, and set it 
	 * high before setting direction.
	 * Rx pin is input
	 */
	if (port&0x01){
		USART_port->OUT |= USARTn1_TXD_bm;
		USART_port->DIR |= USARTn1_TXD_bm;
		USART_port->DIRCLR = USARTn1_RXD_bm;
	}else{
		USART_port->OUT |= USARTn0_TXD_bm;
		USART_port->DIR |= USARTn0_TXD_bm;
		USART_port->DIRCLR = USARTn0_RXD_bm;
	}
#endif /*__AVR_ATxmega256A3__*/

	/*
	 * Setup baudrate
	 */
	(*(*rs232).BAUDH) &= 0xff00;/*clear 4 LSB*/
  (*(*rs232).BAUDH) |= (uint8_t)(0x00ff & (bd>>8));
  (*(*rs232).BAUDL) = (uint8_t)bd;

  /*
   * - Enable receiver and transmitter
	 */
	(*(*rs232).FUNCTION) |= 
					USART_RECEIVER_ENABLE |
					USART_TRANSMITTER_ENABLE;

  /* - Enable interrupts for receiver and transmitter
	 *   ( medium priority interrupts for xmega)
   */
	(*(*rs232).INTERRUPT) |= 
	 				USART_INTERRUPT_RX_COMPLETE |
					USART_INTERRUPT_TX_COMPLETE;

  /*
   * - mode (sync. / async)
   * - Parity
   * - Stop bits
   * - charater size (9 bits are currently not supported)
   * - clock polarity
  */
	(*(*rs232).FORMAT) = ffmt;

	/*setup flag and input handler*/
  (*rs232).txwait = 0;
  (*rs232).input_handler = NULL;
}

void
rs232_print_p(uint8_t port, const char *buf)
{
  while(pgm_read_byte(buf)) {
    rs232_send(port, pgm_read_byte(buf));
    ++buf;
  }
}
/*---------------------------------------------------------------------------*/
void
rs232_print(uint8_t port, char *buf)
{
  while(*buf) {
#if ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
    if(*buf=='\n') rs232_send(port, '\r');
	if(*buf=='\r') buf++; else rs232_send(port, *buf++);
#else
    rs232_send(port, *buf++);
#endif
  }
}
/*---------------------------------------------------------------------------*/
void
rs232_printf(uint8_t port, const char *fmt, ...)
{
  va_list ap;
  static char buf[RS232_PRINTF_BUFFER_LENGTH];

  va_start (ap, fmt);
  vsnprintf (buf, RS232_PRINTF_BUFFER_LENGTH, fmt, ap);
  va_end(ap);

  rs232_print (port, buf);
}
/*---------------------------------------------------------------------------*/
void
rs232_send(uint8_t port, unsigned char c)
{
  rs232_ports[port].txwait = 1;
  *(rs232_ports[port].DATA) = c;
	while(rs232_ports[port].txwait);
}
/*---------------------------------------------------------------------------*/
void
rs232_set_input(uint8_t port, int (*f)(unsigned char))
{
  rs232_ports[port].input_handler = f;
}
/*---------------------------------------------------------------------------*/
#if defined(SLIP_PORT)
void
slip_arch_writeb(unsigned char c)
{
  rs232_send(SLIP_PORT, c);
}
#endif
/*---------------------------------------------------------------------------*/
int rs232_stdout_putchar(char c, FILE *stream);
static uint8_t stdout_rs232_port=RS232_PORT_0;
static FILE rs232_stdout = FDEV_SETUP_STREAM(rs232_stdout_putchar,
					     NULL,
					     _FDEV_SETUP_WRITE);
int rs232_stdout_putchar(char c, FILE *stream)
{
#if ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
  if(c=='\n') rs232_send(stdout_rs232_port, '\r');
  if(c!='\r') rs232_send (stdout_rs232_port, c);
#else
  rs232_send (stdout_rs232_port, c);
#endif
  return 0;
}
/*---------------------------------------------------------------------------*/
void rs232_redirect_stdout (uint8_t port) {
  stdout_rs232_port = port;
  stdout = &rs232_stdout;
}
