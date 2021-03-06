/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
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
 * @(#)$Id: avrdef.h,v 1.2 2007/12/11 17:21:14 joxe Exp $
 */

#ifndef AVRDEF_H
#define AVRDEF_H

#include <avr/io.h>
#include <avr/interrupt.h>


/* Include stdint.h before avrdef.h, best option, otherwise do it this way. */
#ifndef __STDINT_H_
#ifdef HAVE_STDINT_H
#include <stdint.h>
#else
typedef unsigned char   uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;
typedef   signed char    int8_t;
typedef          short  int16_t;
typedef          long   int32_t;
#endif /* !HAVE_STDINT_H */
#endif

/* These names are deprecated, use C99 names. */
typedef  uint8_t    u8_t;
typedef uint16_t   u16_t;
typedef uint32_t   u32_t;
typedef  int32_t   s32_t;
typedef unsigned short clock_time_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

/* Common function prototypes. */
void cpu_init(void);
void clock_delay(unsigned int us2);
void clock_wait(int ms10);
void clock_set_seconds(unsigned long s);
unsigned long clock_seconds(void);
void *sbrk(int);
uint8_t xmega_read_calibration_byte(uint8_t index);

/* ?, uses SREG from avr/io.h. */
typedef unsigned char spl_t;
static inline void splx(spl_t s) { SREG = s; }
static inline spl_t splhigh(void) { spl_t s = SREG; cli(); return s; }

/* AVR XMega family defines, to control family specific behavior. */
#if defined(__AVR_ATxmega16A4__) \
|| defined(__AVR_ATxmega32A4__) \
|| defined(__AVR_ATxmega64A1U__) \
|| defined(__AVR_ATxmega64A3__) \
|| defined(__AVR_ATxmega128A1__) \
|| defined(__AVR_ATxmega128A1U__) \
|| defined(__AVR_ATxmega128A3__) \
|| defined(__AVR_ATxmega192A3__) \
|| defined(__AVR_ATxmega256A3__) \
|| defined(__AVR_ATxmega256A3B__)
#define __XMEGA__
#define __XMEGA_A__
#endif

#if defined(__AVR_ATxmega16D4__) \
|| defined(__AVR_ATxmega32D4__) \
|| defined(__AVR_ATxmega64D3__) \
|| defined(__AVR_ATxmega128D3__) \
|| defined(__AVR_ATxmega192D3__) \
|| defined(__AVR_ATxmega256D3__)
#define __XMEGA__
#define __XMEGA_D__
#endif

#endif /* AVRDEF_H */
