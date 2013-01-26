/*****************************************************************************
* vim:sw=8:ts=8:si:et
*
* Title        : Microchip ENC28J60 Ethernet Interface Driver
* Author        : Pascal Stang (c)2005
* Modified by Guido Socher
* Copyright: GPL V2
*
*This driver provides initialization and transmit/receive
*functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
*This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
*chip, using an SPI interface to the host processor.
*****************************************************************************/
//@{


extern uint8_t mymac[6];

#ifndef ETH_ENC28J60_H
#define ETH_ENC28J60_H


#include "eth-enc38j60-regs.h"
#include "uip.h"
#include <inttypes.h>
#include <avr/pgmspace.h>

// ENC28J60 Control Registers
// Control register definitions are a combination of address,
// bank number, and Ethernet/MAC/PHY indicator bits.
// - Register address        (bits 0-4)
// - Bank number        (bits 5-6)
// - MAC/PHY indicator        (bit 7)
#define ADDR_MASK        0x1F
#define BANK_MASK        0x60
#define SPRD_MASK        0x80


//globals
extern uint16_t totalPacketSize;

// functions
extern void enc28j60Reset(void);
extern uint8_t enc28j60ReadOp(uint8_t op, uint8_t address);
extern void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data);
extern void enc28j60ReadBuffer(uint16_t len, uint8_t* data);
extern uint8_t enc28j60ReadBufferByte(void);
extern void enc28j60WriteBuffer(uint16_t len, uint8_t* data);
extern void enc28j60SetBank(uint8_t address);
extern uint8_t enc28j60Read(uint8_t address);
extern void enc28j60Write(uint8_t address, uint8_t data);
extern void enc28j60PhyWrite(uint8_t address, uint16_t data);
extern void enc28j60Init(struct uip_eth_addr* macaddr);
extern void enc28j60PacketSend(uint16_t len, uint8_t* packet);



extern uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet);
extern uint8_t enc28j60getrev(void);

extern uint16_t enc28j60GetPacketLength(void);
extern void enc28j60FinishPacket(void);


#endif
//@}
