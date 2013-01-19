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


#define ANNOUNCE_BOOT 1    //adds about 600 bytes to program size

#define DEBUG 0
#if DEBUG
#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#define PRINTSHORT(FORMAT,args...) printf_P(PSTR(FORMAT),##args)

#else
#define PRINTF(...)
#define PRINTSHORT(...)
#endif

#include <avr/pgmspace.h>
#include <avr/fuse.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <string.h>


#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "dev/rs232.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/watchdog.h"
#include <autostart.h>
#include "interrupt.h"

#include <util/delay.h>

#ifdef POE_LCD_TEST
#include "lcd-test.h"
#endif


void setup_clock(void) {
    //set to run at 8MHz
//   CLKPR = _BV(CLKPCE);
//   CLKPR = 0x80;
//   CLKPR = 0x00;

    //CPU_CCP = 0x9D; //set change enabled

    //turn on external osc.
    OSC.CTRL |= OSC_XOSCEN_bm   ; //also PLLEN

    // external osc clock source, 20x multiplier
    OSC_PLLCTRL = 0b11001000;
    //OSC.PLLCTRL = 0b11000011;



    _delay_ms(100);
    OSC.CTRL |= OSC_PLLEN_bm;

    _delay_ms(100);
    //CLK.CTRL |=  //(SCLKSEL) use the pll
    CPU_CCP = CCP_IOREG_gc;
    //CLK.CTRL = CLK_SCLKSEL_XOSC_gc;
    CLK.CTRL = CLK_SCLKSEL_PLL_gc;

    CPU_CCP = CCP_IOREG_gc;
    //CLK.PSCTRL = (CLK.PSCTRL & ~(CLK_PSBCDIV_gm) )| CLK_PSBCDIV_2_2_gc;
    CLK.PSCTRL = 0b00000011;
    
    /* Lock the clock so that clock_init does not adjust later. */
    CCP = CCP_IOREG_gc;
    CLK.LOCK |= CLK_LOCK_bm;
}

void setup_board(void) {
    //enable relay output
    PORTE.DIR |= (1<<1);

}


/*-------------------------Low level initialization------------------------*/
/*------Done in a subroutine to keep main routine stack usage small--------*/
void initialize(void)
{

  //calibrate_rc_osc_32k(); //CO: Had to comment this out
  watchdog_init();
  setup_board();

//       
//
  interrupt_init( PMIC_CTRL_HML_gm);
  interrupt_start();
  
  rs232_init(RS232_USARTE0, XMEGA_BAUD_ASYNC_9600 ,USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);


  rs232_redirect_stdout(RS232_USARTE0);

    
  setup_clock();
  clock_init();

  //watchdog_start();

  //enable interupts

    _delay_ms(300);
  
  /* zeroth rs232 port for debugging */
  PORTE.DIRSET = PIN3_bm;
  PORTE.OUTSET= PIN3_bm;
  PORTE.DIRCLR = PIN2_bm;



  /* Redirect stdout to zeroth port */


  

//#if ANNOUNCE_BOOT
  printf_P(PSTR("\n*******Booting %s*******\n"),CONTIKI_VERSION_STRING);
//#endif


/* rtimers needed for radio cycling */
//  rtimer_init();

 /* Initialize process subsystem */
  process_init();
  
 /* etimers must be started before ctimer_init */
  process_start(&etimer_process, NULL);

  //process_start(&hello_world_process, NULL);
// #ifdef POE_LCD_TEST
//   process_start(&lcd_test_process, NULL);
// #endif
/* mac process must be started before tcpip process! */
//   process_start(&mac_process, NULL);
//   process_start(&tcpip_process, NULL);

  /* Autostart other processes */
  autostart_start(autostart_processes);



}

/*---------------------------------------------------------------------------*/
void log_message(char *m1, char *m2)
{
  printf_P(PSTR("%s%s\n"), m1, m2);
}


/*-------------------------------------------------------------------------*/
/*------------------------- Main Scheduler loop----------------------------*/
/*-------------------------------------------------------------------------*/
int
main(void)
{

  initialize();
  while(1) {
    watchdog_periodic();
    process_run();

  }
  return 0;
}



