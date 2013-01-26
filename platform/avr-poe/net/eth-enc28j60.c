/*********************************************
 * vim:sw=8:ts=8:si:et
 * To use the above modeline in vim you must have "set modeline" in your .vimrc
 * Author: Guido Socher 
 * Copyright: GPL V2
 *
 * Based on the enc28j60.c file from the AVRlib library by Pascal Stang.
 * For AVRlib See http://www.procyonengineering.com/
 *********************************************/
#include <avr/io.h>
#include <stdio.h>
#include <string.h>

//just so we can send stupid \r.
#include "dev/rs232.h"

#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "eth-enc28j60.h"

#include "xmega-signatures.h"

#include <spi-xmega.h>
#include <enc28j60pins.h>

#ifndef ALIBC_OLD
#include <util/delay.h>
#else
#include <avr/delay.h>
#endif

static uint8_t Enc28j60Bank;
static uint16_t NextPacketPtr;

uint16_t totalPacketSize;

#if !defined(_ENC28J60_PINS)
    #error "you need to define the ENC28J60 pins for your board. You'll need an enc28j60pins.h file in your project.'"
#endif


uint8_t spfd = -1;

void enc28j60Reset(void) {
//     printf("Eth reset\n");
    _delay_ms(100);
    //perform a software reset
    if (spi_lock(spfd)!= 0) {
        printf("could not lock\n");
    }
//      printf("locked\n");
    uint8_t send = 0xff;
    spi_write(spfd, &send, sizeof(send));
    spi_unlock(spfd);
    _delay_ms(100);
    enc28j60PhyWrite(PHCON1, 0x8000);
    _delay_ms(1800);
//     printf("ready to go\n");

    
}

uint8_t enc28j60ReadOp(uint8_t op, uint8_t address)
{
        if (spi_lock(spfd)!= 0) {
            printf("could not lock\n");
        }
        // issue read command
        uint8_t send = op | (address & ADDR_MASK);
        spi_write(spfd, &send, sizeof(send));
        // read data
        spi_read(spfd, &send, sizeof(send));
        // do dummy read if needed (for mac and mii, see datasheet page 29)
        if(address & 0x80)
        {
            spi_read(spfd, &send, sizeof(send));
        }
        // release CS
        spi_unlock(spfd);
        return(send);
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data)
{
        if (spi_lock(spfd)!= 0) {
            printf("could not lock\n");
        }  
        address = op | (address & ADDR_MASK);
        // issue write command
        spi_write(spfd, &address, sizeof(address));
        // write data
        spi_write(spfd, &data, sizeof(data));
        spi_unlock(spfd);
}

void enc28j60ReadBuffer(uint16_t len, uint8_t* data)
{
        spi_lock(spfd);
        // issue read command
        uint8_t send = ENC28J60_READ_BUF_MEM;
        spi_write(spfd, &send, sizeof(send));
        spi_read(spfd, data,len);
        *data='\0';
        spi_unlock(spfd);
}

uint8_t enc28j60ReadBufferByte(void)
{
        uint8_t ret;
        //just a call to the multi-byte version
        enc28j60ReadBuffer(1,&ret);
        return ret;
}

void enc28j60WriteBuffer(uint16_t len, uint8_t* data)
{
        spi_lock(spfd);
        // issue write command
        uint8_t send = ENC28J60_WRITE_BUF_MEM;
        spi_write(spfd, &send, sizeof(send));
        spi_write(spfd,data,len);
        spi_unlock(spfd);
}

void enc28j60SetBank(uint8_t address)
{
        // set the bank (if needed)
        if((address & BANK_MASK) != Enc28j60Bank)
        {
                // set the bank
                enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
                enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
                Enc28j60Bank = (address & BANK_MASK);
        }
}

uint8_t enc28j60Read(uint8_t address)
{
        // set the bank
        enc28j60SetBank(address);
        // do the read
        return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

void enc28j60Write(uint8_t address, uint8_t data)
{
        // set the bank
        enc28j60SetBank(address);
        // do the write
        enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void enc28j60PhyWrite(uint8_t address, uint16_t data)
{
    // set the PHY register address
    enc28j60Write(MIREGADR, address);
    // write the PHY data
    enc28j60Write(MIWRL, data);
    enc28j60Write(MIWRH, data>>8);
    // wait until the PHY write completes
    while(enc28j60Read(MISTAT) & MISTAT_BUSY){
        _delay_us(15);
    }
}

// ----------------------------- begin higher-level access ------------------------------------


void enc28j60Init(uip_eth_addr* macaddr)
{
    printf ( "eth: Init\n" );
    //init the multi-SPI interface with just one SPI device
    spi_init_multi ( enc28j60_spi,1 );
    //we're going to open this and just leave it open for now.
    spfd = spi_open ( &enc28j60_spi[0] );
//     printf ( "spfd = %d\n", spfd );
    
    // perform system reset
    enc28j60Reset();
    // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
    _delay_ms(1);

    enc28j60SetBank ( ECON1 );
    //change LED settings
    //enc28j60PhyWrite(PHLCON, 0b0000110110100000);
    //flash both
//   enc28j60PhyWrite(PHLCON, 0b0000101010100010);
    //red for rx, green for tx, min stretching
    enc28j60PhyWrite ( PHLCON, 0b0000000100100010 );
    //green for tx, red for link/rx, min stretching
    //enc28j60PhyWrite(PHLCON, 0b0000000111000010);
    
    NextPacketPtr = RXSTART_INIT;
    // Rx start
    enc28j60Write ( ERXSTL, RXSTART_INIT&0xFF );
    enc28j60Write ( ERXSTH, RXSTART_INIT>>8 );
    // set receive pointer address
    enc28j60Write ( ERXRDPTL, RXSTART_INIT&0xFF );
    enc28j60Write ( ERXRDPTH, RXSTART_INIT>>8 );
    // RX end
    enc28j60Write ( ERXNDL, RXSTOP_INIT&0xFF );
    enc28j60Write ( ERXNDH, RXSTOP_INIT>>8 );
    // TX start
    enc28j60Write ( ETXSTL, TXSTART_INIT&0xFF );
    enc28j60Write ( ETXSTH, TXSTART_INIT>>8 );
    // TX end
    enc28j60Write ( ETXNDL, TXSTOP_INIT&0xFF );
    enc28j60Write ( ETXNDH, TXSTOP_INIT>>8 );

    //accept broadcast packets (to FF:FF:FF:FF:FF:FF) and accept direct packets
    enc28j60Write ( ERXFCON, ERXFCON_UCEN|ERXFCON_BCEN );
    
    // do bank 2 stuff
    // enable MAC receive
    enc28j60Write ( MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS );
    if ( enc28j60Read ( MACON1 ) != ( MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS ) )
    {
        PRINTF ( "MAC init failed\n" );
    }
    // bring MAC out of reset
    enc28j60Write ( MACON2, 0x00 );
    // enable automatic padding to 60bytes and CRC operations
    enc28j60WriteOp ( ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN );
    // set inter-frame gap (non-back-to-back)
    enc28j60Write ( MAIPGL, 0x12 );
    enc28j60Write ( MAIPGH, 0x0C );
    // set inter-frame gap (back-to-back)
    enc28j60Write ( MABBIPG, 0x12 );
    // Set the maximum packet size which the controller will accept
    // Do not send packets longer than UIP_BUFSIZE:
    enc28j60Write ( MAMXFLL, UIP_BUFSIZE&0xFF );
    enc28j60Write ( MAMXFLH, UIP_BUFSIZE>>8 );
    
    // do bank 3 stuff
    // write MAC address
    // NOTE: MAC address in ENC28J60 is byte-backward
    enc28j60Write ( MAADR5, macaddr->addr[0] );
    enc28j60Write ( MAADR4, macaddr->addr[1] );
    enc28j60Write ( MAADR3, macaddr->addr[2] );
    enc28j60Write ( MAADR2, macaddr->addr[3] );
    enc28j60Write ( MAADR1, macaddr->addr[4] );
    enc28j60Write ( MAADR0, macaddr->addr[5] );
    // no loopback of transmitted frames
    enc28j60PhyWrite ( PHCON2, PHCON2_HDLDIS );

    // switch to bank 0
    enc28j60SetBank ( ECON1 );

    // enable interrutps
//     //enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
    enc28j60WriteOp ( ENC28J60_BIT_FIELD_SET, EIE, EIE_PKTIE ); //cannot enable interrupts if INT is wired to SS - will fix on next version of board

    // enable packet reception
    enc28j60WriteOp ( ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN );
}

// read the revision of the chip:
uint8_t enc28j60getrev(void)
{
	return(enc28j60Read(EREVID));
}

void enc28j60PacketSend(uint16_t len, uint8_t* packet)
{
   //printf("sending from card - if this doesn't hit the network, we have an error here.\n");
    printf("Es\n");
//   uint16_t b;
//   for (b = 0; b<len; b++) {
//     printf("%0X ", packet[b]);
//   }
//   printf("\n\n");

  //check to ensure previous packet made it
  //uint8_t prev = enc28j60Read(ECON1);
  //printf("pps: %x\n", prev&ECON1_TXRTS);
  //wait for RTS

    if( (enc28j60Read(EIR) & (EIR_TXERIF)) ){
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST );
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST );

        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
        printf("TX reset!\n");
        return;
    }
  
  uint8_t cnt=255;
  while (enc28j60Read(ECON1)&ECON1_TXRTS){
    printf("tx nrts\n");
    if (!cnt--){
        printf("drop the packet and keep going\n");
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST );
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST );

        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
        return;
    }
    
  }
  
  
	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);
	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
	enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);
	// write per-packet control byte (0x00 means use macon3 settings)
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(len, packet);
	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
        // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
	if( (enc28j60Read(EIR) & EIR_TXERIF) ){
                enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
        }
}

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet)
{
	uint16_t rxstat;
	uint16_t len;
	// check if a packet has been received and buffered
	//if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
        // The above does not work. See Rev. B4 Silicon Errata point 6.
  uint8_t wait = enc28j60Read(EPKTCNT);
  
	if( wait ==0 ){
      return(0);
  }

  if (wait>0){
    if (wait>2){
        printf("eth: %d pkts waiting\n", wait );
    }
    
    if( (enc28j60Read(EIR) & (EIR_RXERIF)) ){
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXRST);
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXRST);

        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_RXERIF);
        printf("RX overflow!\n");
        return 0;
    }
    while (wait > 6) {
        printf("dropping packet\n");
        // Set the read pointer to the start of the received packet
        enc28j60Write(ERDPTL, (NextPacketPtr));
        enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
        // read the next packet pointer
        NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
        NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
        enc28j60Write(ERXRDPTL, (NextPacketPtr));
        enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);
        // decrement the packet counter indicate we are done with this packet
        enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
        wait = enc28j60Read(EPKTCNT);
    }
  }

	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (NextPacketPtr));
	enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
	// read the next packet pointer
	NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the packet length (see datasheet page 43)
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
        len-=4; //remove the CRC count
	// read the receive status (see datasheet page 43)
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// limit retrieve length
        if (len>maxlen-1){
                len=maxlen-1;
        }
        // check CRC and symbol errors (see datasheet page 44, table 7-3):
        // The ERXFCON.CRCEN is set by default. Normally we should not
        // need to check this.
        if ((rxstat & 0x80)==0){
                // invalid
                len=0;
        }else{
                // copy the packet from the receive buffer
                enc28j60ReadBuffer(len, packet);
        }
	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	enc28j60Write(ERXRDPTL, (NextPacketPtr));
	enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);
	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	return(len);
}


//scott added
/*

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
// this also leaves the read buffer pointer pointed at the start of the packet.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t enc28j60GetPacketLength()
{
  uint16_t rxstat;
  uint16_t len;
  uint8_t numPackets;


    
  
  // check if a packet has been received and buffered
  //if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
    // The above does not work. See Rev. B4 Silicon Errata point 6.
    numPackets = enc28j60Read(EPKTCNT);
    if( numPackets == 0 ){
      return(0);
    }



    // Set the read pointer to the start of the received packet
    enc28j60Write(ERDPTL, (NextPacketPtr));
    enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
    // read the next packet pointer
    NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

    // read the packet length (see datasheet page 43)
    len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
    len-=4; //remove the CRC count
    // read the receive status (see datasheet page 43)
    rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
    rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

    // check CRC and symbol errors (see datasheet page 44, table 7-3):
    // The ERXFCON.CRCEN is set by default. Normally we should not
    // need to check this.
    if ((rxstat & 0x80)==0 || numPackets >= 3){
      // invalid
      len=0;
      //have to advance anyway to skip the invalid packet.
      enc28j60FinishPacket();
    }
    
    return(len);
}

// Finishes with the current packet. 
// moves the ERXRDP pointer to the next packet.
void enc28j60FinishPacket()
{
// Move the RX read pointer to the start of the next received packet
// This frees the memory we just read out
    enc28j60Write(ERXRDPTL, (NextPacketPtr));
    enc28j60Write(ERXRDPTH, (NextPacketPtr)>>8);
    // decrement the packet counter indicate we are done with this packet
    enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

    if( (enc28j60Read(EIR) & (EIR_RXERIF | EIR_TXERIF)) ){
        //we missed RXing  one or more packets due to lack of space
        //or failed to tx
        //use Bit Field Clear to reset
        enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, EIR , EIR_RXERIF | EIR_TXERIF);

        //we could reset here, but we don't have the MAC addr handy..

        //enc28j60Reset();
        //enc28j60Init();
    }
    

}*/


/*---------------------------------------------------------------------------*/
/*
 * This is the end of the code that interacts with the hardware, and the begining of the code that implements the uIP driver.
 */


#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
#define BUFNP ((struct uip_eth_hdr)uip_buf)


//prints whatever is in the uip_buf
void print_packet(void) {
    uint16_t i = 0;
    PRINTF("packet: ");
    for (i=0;i <uip_len; i++) {
        PRINTF("%x ",uip_buf[i]);
    }
    PRINTF("\n");
}

/*---------------------------------------------------------------------------*/
/*
 * We declare the process that we use to register with the TCP/IP stack,
 * and to check for incoming packets.
 */
PROCESS(enc28j60_process, "enc28j60 driver");
/*---------------------------------------------------------------------------*/
/*
 * Next, we define the function that transmits packets. This function
 * is called from the TCP/IP stack when a packet is to be transmitted.
 * The packet is located in the uip_buf[] buffer, and the length of the
 * packet is in the uip_len variable.
 */
u8_t
enc28j60_driver_output(void)
{
    
    //PRINTF("in enc28j60_driver_output, len is %d\n ",uip_len);
    
    memmove((((void*)&uip_buf)) + 14, (&uip_buf), uip_len);
    
    //PRINTF("packet to send, len: %d\n", uip_len);
    //print_packet();
    
    uip_arp_out();
    //PRINTF("in enc28j60_driver_output, post arp, len is %d\n ",uip_len);
    //print_packet();
    
    enc28j60PacketSend(uip_len, uip_buf);
    // A network device driver returns always zero.
    return 0;
}

/* another version, but doesn't do ARP*/
u8_t
enc28j60_driver_output_noarp(void)
{
    
//     PRINTF("packet to send, already with arp, len: %d\n", uip_len);
    //print_packet();
    enc28j60PacketSend(uip_len, uip_buf);
    return 0;
}

static void
pollhandler(void)
{
    uint8_t i;
    for(i = 0; i < UIP_CONNS; i++) {
        //PRINTF("poll UIP %d\n", i);
        uip_periodic(i);
        /* If the above function invocation resulted in data that
         *            should be sent out on the network, the global variable
         *            uip_len is set to a value > 0. */
        if(uip_len > 0) {
            PRINTF("con caused send\n");
            enc28j60_driver_output();
        }
    }
    
    for(i = 0; i < UIP_UDP_CONNS; i++) {
        uip_udp_periodic(i);
        /* If the above function invocation resulted in data that
         *            should be sent out on the network, the global variable
         *            uip_len is set to a value > 0. */
        if(uip_len > 0) {
            PRINTF("UDP con caused send\n");
            enc28j60_driver_output();
        }
    }
    
    //copy the packet into our buffer
    uip_len = enc28j60PacketReceive(UIP_BUFSIZE, uip_buf);//check_and_copy_packet();
    
    if(uip_len > 0) {
        //print_packet();
        struct uip_eth_hdr * hdrp = BUF;
        if(hdrp->type == uip_htons(UIP_ETHTYPE_IP)) {
            //PRINTF("RXed an IP packet, len: %d\n", uip_len);
            uip_arp_ipin();
            //PRINTF("RXed an IP packet, post ARP, len: %d, seq %lx\n", uip_len, uip_htonl(*(uint32_t*)(uip_buf + 38)));
            //print_packet();
            
            //tcpip_input will expect an IP packet, not an ethernet frame
            memmove((&uip_buf), ((void*)&uip_buf) + 14, uip_len);
            uip_len -= 14;
            //PRINTF("RX an IP packet START\n");
            tcpip_input();
            //PRINTF("RX an IP packet STOP\n");
        } else if(hdrp->type == uip_htons(UIP_ETHTYPE_ARP)) {
            //PRINTF("RXed an ARP packet, len: %d\n", uip_len);
            //print_packet();
            uip_arp_arpin();
            //PRINTF("RXed an ARP packet\n");
            /* If the above function invocation resulted in data that
             *         should be sent out on the network, the global variable
             *         uip_len is set to a value > 0. */
            if (uip_len > 0) {
                //PRINTF("RX of ARP packet causes more send\n");
                enc28j60_driver_output_noarp();
            }
        }
    }
    
    process_poll(&enc28j60_process);
}
/*---------------------------------------------------------------------------*/
/*
 * Finally, we define the process that does the work.
 */
PROCESS_THREAD(enc28j60_process, ev, data)
{
    //setup the poll handler
    PROCESS_POLLHANDLER(pollhandler());
    
    uip_eth_addr our_uip_ethaddr;
    PROCESS_BEGIN();
    
    uip_init();
    uip_arp_init();
    tcpip_set_outputfunc(enc28j60_driver_output);
    
    //set the mac address to be our serial number
    uip_80211_addr tempMAC;
    get_serial_number((char*)(uip_ethaddr.addr), sizeof(our_uip_ethaddr));
    get_serial_number((char*)(&(tempMAC.addr[0])), 6);
    uip_setethaddr(tempMAC);
    
    PRINTF("eth: MAC addr ");
    
    uint8_t i = 0;
    for (i=0; i<sizeof(uip_ethaddr.addr); i++){
        PRINTF("%2x ", uip_ethaddr.addr[i] );
    }
    PRINTF("\n");
    
    enc28j60Init(&uip_ethaddr);
    
    
    process_poll(&enc28j60_process);
    printf("eth: network initialized, rev %d\n", enc28j60getrev());
    //process_start(&my_dhcp_process, "dhcp_now");
    
    
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXIT);
    
    //shutdown_the_hardware();
//     PRINTF("shut down ethernet hardware.\n");
    
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/




