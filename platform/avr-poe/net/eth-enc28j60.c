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
#include "eth-enc28j60.h"
#include <enc28j60pins.h>

//

#ifndef ALIBC_OLD
#include <util/delay.h>
#else
#include <avr/delay.h>
#endif

#define XAP_PORT 3865


uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x44};

static uint8_t Enc28j60Bank;
static uint16_t NextPacketPtr;

uint16_t totalPacketSize;

#if !defined(_ENC28J60_PINS)
    #error "you need to define the ENC28J60 pins for your board. You'll need an enc28j60pins.h file in your project.'"
#endif



// set CS to 0 = active
//#define CSACTIVE ENC28J60_CONTROL_PORT&=~(1<<ENC28J60_CONTROL_CS)
#define CSACTIVE ENC28J60_CONTROL_CS_PORT.OUTCLR = (1<<ENC28J60_CONTROL_CS)
// set CS to 1 = passive
//#define CSPASSIVE ENC28J60_CONTROL_PORT|=(1<<ENC28J60_CONTROL_CS)
#define CSPASSIVE ENC28J60_CONTROL_CS_PORT.OUTSET = (1<<ENC28J60_CONTROL_CS)
//
#define waitspi() while(!(SPID.STATUS & SPI_IF_bm))


void enc28j60Reset(void) {
            /* enable PB0, reset as output */
       // ENC28J60_CONTROL_DDR |= (1<<ENC28J60_CONTROL_RESET);

        /* set output to gnd, reset the ethernet chip */
        //ENC28J60_CONTROL_PORT &= ~(1<<ENC28J60_CONTROL_RESET);
       // _delay_ms(20);
        
        /* set output to Vcc, reset inactive */
       // ENC28J60_CONTROL_PORT |= (1<<ENC28J60_CONTROL_RESET);

    _delay_ms(100);
    //perform a software reset
    CSACTIVE;
    // issue write command
    SPIPORT.DATA = 0xff;

    waitspi();


    CSPASSIVE;

    _delay_ms(100);

    //enc28j60PhyWrite(PHLCON, 0b0000110110100000);
    enc28j60PhyWrite(PHCON1, 0x8000);
    _delay_ms(1800);
    printf("ready to go\n");
//     while (enc28j60Read(ESTAT)&ESTAT_CLKRDY) {
//         _delay_ms(1);
//         printf("clock not ready\n");
//     }

    
}

uint8_t enc28j60ReadOp(uint8_t op, uint8_t address)
{
        CSACTIVE;
        // issue read command
        SPIPORT.DATA = op | (address & ADDR_MASK);
        waitspi();
        // read data
        SPIPORT.DATA = 0x00;
        waitspi();
        // do dummy read if needed (for mac and mii, see datasheet page 29)
        if(address & 0x80)
        {
                SPIPORT.DATA = 0x00;
                waitspi();
        }
        // release CS
        CSPASSIVE;
        return(SPIPORT.DATA);
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data)
{
        CSACTIVE;
        // issue write command
        SPIPORT.DATA = op | (address & ADDR_MASK);

        waitspi();

        // write data
        SPIPORT.DATA = data;

        waitspi();

        CSPASSIVE;
}

void enc28j60ReadBuffer(uint16_t len, uint8_t* data)
{
        CSACTIVE;
        // issue read command
        SPIPORT.DATA = ENC28J60_READ_BUF_MEM;
        waitspi();
        while(len)
        {
                len--;
                // read data
                SPIPORT.DATA = 0x00;
                waitspi();
                *data = SPIPORT.DATA;
                data++;
        }
        *data='\0';
        CSPASSIVE;
}

uint8_t enc28j60ReadBufferByte(void)
{
        uint8_t ret;
        CSACTIVE;
        // issue read command
        SPIPORT.DATA = ENC28J60_READ_BUF_MEM;
        waitspi();
        SPIPORT.DATA = 0x00;
        waitspi();
        ret = SPIPORT.DATA;
        CSPASSIVE;
        return ret;
}

void enc28j60WriteBuffer(uint16_t len, uint8_t* data)
{
        CSACTIVE;
        // issue write command
        SPIPORT.DATA = ENC28J60_WRITE_BUF_MEM;
        waitspi();
        while(len)
        {
                len--;
                // write data
                SPIPORT.DATA = *data;
                data++;
                waitspi();
        }
        CSPASSIVE;
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
	// initialize I/O
  

	ENC28J60_CONTROL_CS_PORT.DIR |= 1<<ENC28J60_CONTROL_CS;
	CSPASSIVE;
        //

  //this is very specific!
  SPIPINPORT.DIRSET = (1<<7) | (1<<5) | (1<<4); //mosi, sck, ss output
  SPIPINPORT.DIRCLR = (1<<6); //MISO as input
	//DDRB  |= 1<<PB3 |1<<PB5 | 1<<PB2; // mosi, sck, ss output
	//cbi(DDRB,PINB4); // MISO is input
        //
	CSPASSIVE;
  //      cbi(PORTB,PB3); // MOSI low
  //      cbi(PORTB,PB5); // SCK low
  SPIPINPORT.OUTCLR = (1<<5) | (1<<7);

  
	//
	// initialize SPI interface
	// master mode and Fosc/2 clock:
  //      SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
  SPIPORT.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_CLK2X_bm;
  //SPIPORT.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm;
  //      SPSR |= (1<<SPI2X);
        

  
  // perform system reset
  enc28j60Reset();

  enc28j60SetBank(ECON1);
  //change LED settings
  //enc28j60PhyWrite(PHLCON, 0b0000110110100000);

  //red for rx, green for tx, min stretching
  enc28j60PhyWrite(PHLCON, 0b0000000100100010);
  //green for tx, red for link/rx, min stretching
  //enc28j60PhyWrite(PHLCON, 0b0000000111000010);
	// check CLKRDY bit to see if reset is complete
        // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
	//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// set receive buffer start address
	NextPacketPtr = RXSTART_INIT;
        // Rx start
	enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXSTH, RXSTART_INIT>>8);
	// set receive pointer address
	enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
	// RX end
	enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
	// TX start
	enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60Write(ETXSTH, TXSTART_INIT>>8);
	// TX end
	enc28j60Write(ETXNDL, TXSTOP_INIT&0xFF);
	enc28j60Write(ETXNDH, TXSTOP_INIT>>8);
	// do bank 1 stuff, packet filter:
        // For broadcast packets we allow only ARP packtets
        // All other packets should be unicast only for our mac (MAADR)
        //
        // The pattern to match on is therefore
        // Type     ETH.DST
        // ARP      BROADCAST
        // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
        // in binary these poitions are:11 0000 0011 1111
        // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
	//enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
  //changed to allow all broadbast UDP packets (!)
  //another option would be to not do ARP at all (breaking ping, for example), and use the pattern match to match the UDP port number and broadcast address
  
//   enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN);
// 	enc28j60Write(EPMM0, 0x3f);
// 	enc28j60Write(EPMM1, 0x30);
// 	enc28j60Write(EPMCSL, 0xf9);
// 	enc28j60Write(EPMCSH, 0xf7);

  //accept broadcast packets (to FF:FF:FF:FF:FF:FF) and accept direct packets
  enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_BCEN);
  //enc28j60Write(ERXFCON, 0);
        
        //
	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	enc28j60Write(MACON2, 0x00);
	// enable automatic padding to 60bytes and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
        // Do not send packets longer than MAX_FRAMELEN:
	enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);	
	enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);
	// do bank 3 stuff
        // write MAC address
        // NOTE: MAC address in ENC28J60 is byte-backward
        printf("setting mac: %x to %x", macaddr->addr[5], macaddr->addr[0]);
        enc28j60Write(MAADR5, macaddr->addr[0]);
        enc28j60Write(MAADR4, macaddr->addr[1]);
        enc28j60Write(MAADR3, macaddr->addr[2]);
        enc28j60Write(MAADR2, macaddr->addr[3]);
        enc28j60Write(MAADR1, macaddr->addr[4]);
        enc28j60Write(MAADR0, macaddr->addr[5]);
	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);

  
	// switch to bank 0
	enc28j60SetBank(ECON1);
	// enable interrutps
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

// read the revision of the chip:
uint8_t enc28j60getrev(void)
{
	return(enc28j60Read(EREVID));
}

void enc28j60PacketSend(uint16_t len, uint8_t* packet)
{
//   printf("sending from card - if this doesn't hit the network, we have an error here.\n");
    //printf("card send\n");
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
    printf(" %d pkt wait\n", wait );
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
    

}


