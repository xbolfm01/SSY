/**
 * @file eeprom.c
 *
 * @brief  Implementation of the native EEPROM interface.
 *
 * Implements methods to get access to EEPROM hardware and
 * to read and write bytes to and from EEPROM
 * Available on following platforms:
 *    - Sensor Terminal Board
 *    - deRFnode
 *
 *
 * IMPORTANT: THIS FEATURE ONLY WORKS WITH deRFmega128 REVISION 2 AND HIGHER!!
 *
 * $Id: eeprom.c,v 1.1 2011/04/08 14:23:36 dam Exp $
 *
 * @author    dresden elektronik: http://www.dresden-elektronik.de
 * @author    Support email: support@dresden-elektronik.de
 *
 * Copyright (c) 2010, Dresden Elektronik All rights reserved.
 *
 * Licensed under Dresden Elektronik Limited License Agreement --> deEULA.txt
 */


/* === INCLUDES ============================================================ */
#include <stdio.h>            // include standard io definitions
#include <stdbool.h>          // include bool definition
#include <avr/io.h>           // include io definitions
#include <util/twi.h>         // include 2-wire definitions
#include <util/delay.h>

#include "config.h"           // include configuration
#include "eeprom.h"           // include EEPROM specific definitions
#include "twi_master.h"       // TWI master library

/* === TYPES =============================================================== */

/* === MACROS / DEFINES ==================================================== */

/* === PROTOTYPES ========================================================== */

/* === GLOBALS ============================================================= */

/* === IMPLEMENTATION ====================================================== */

/**
 * @brief Initialize the EEPROM hardware
 *
 * Does all the startup-time peripheral initializations: TWI clock, ...
 *
 */
void eeprom_init(void)
{
    /* initialize the MCU to be TWI master, run with 100kHz clock */
    twi_init(100000ULL);
}


/**
 * @brief Read "len" bytes from EEPROM starting at "eeaddr" into "buf".
 *
 * This requires two bus cycles: during the first cycle, the device
 * will be selected (master transmitter mode), and the address
 * transfered.
 *
 * The second bus cycle will reselect the device (repeated start
 * condition, going into master receiver mode), and transfer the data
 * from the device to the TWI master. Multiple bytes can be transfered by
 * ACKing the client's transfer. The last transfer will  be NACKed, which
 * the client will take as an indication to not initiate further transfers.
 *
 * @param eeaddr  the EEPROM address where to read data from
 * @param len     the length of data
 * @param buf     pointer to data which should be read
 *
 * @return        number of read bytes, or -1 if any error occured
 */
int32_t eeprom_read_bytes(uint32_t eeaddr, uint16_t len, uint8_t *buf)
{
  uint8_t slave_addr;
  int32_t rv = 0;

  /* set slave address */
  slave_addr = TWI_SLA_24CXX;

  /* modify device address, if upper memory area requested */
  if(eeaddr & MAX_ADDRESS_MASK_24CXX)
  {
    slave_addr |= ADDR_HIGH_24CXX;
  }

  /*
   * First cycle: master transmitter mode
   */

  /* send start condition */
  if(twi_start(slave_addr, TWI_WRITE)) goto fail;

  /* send address MSB followed by LSB */
  if(twi_write((eeaddr >> 8) & 0xFF)) goto fail;
  if(twi_write(eeaddr & 0xFF)) goto fail;

  /*
   * Next cycle(s): master receiver mode
   */
  if(twi_start(slave_addr, TWI_READ)) goto fail;

  for( ; len>0; len--)
  {
    if (len==1)
       *buf = twi_readNack();
    else
       *buf = twi_readAck();

    buf++;
    rv++;
  }

quit:
  twi_stop();
  return rv;

fail:
  rv = -1;
  goto quit;
}


/**
 * @brief Write "len" bytes into EEPROM starting at "eeaddr" from "buf".
 *
 * This is a bit simpler than the previous function since both, the
 * address and the data bytes will be transfered in master transmitter
 * mode, thus no reselection of the device is necessary.  However, the
 * EEPROMs are only capable of writing one "page" simultaneously, so
 * care must be taken to not cross a page boundary within one write
 * cycle.  The amount of data one page consists of varies from
 * manufacturer to manufacturer: some vendors only use 8-byte pages
 * for the smaller devices, and 16-byte pages for the larger devices,
 * while other vendors generally use 16-byte pages.  We thus use the
 * smallest common denominator of 8 bytes per page, declared by the
 * macro PAGE_SIZE above.
 *
 * The function simply returns after writing one page, returning the
 * actual number of data byte written.  It is up to the caller to
 * re-invoke it in order to write further data.
 *
 * @param eeaddr  the EEPROM address where to write data
 * @param len     the length of data
 * @param buf     pointer to data which should be written
 *
 * @return        number of written bytes, or -1 if any error occured
 */
int16_t eeprom_write_page(uint32_t eeaddr, uint16_t len, uint8_t *buf)
{
  uint8_t slave_addr;
  int16_t rv = 0;
  uint32_t endaddr;

  /* optionally truncate length */
  if (eeaddr + len < (eeaddr | (PAGE_SIZE - 1)))
  {
    /* matches into one page */
    endaddr = eeaddr + len;
  }
  else
  {
    /* does not match */
    endaddr = (eeaddr | (PAGE_SIZE - 1)) + 1;
  }
  len = endaddr - eeaddr;

  /* set slave address */
  slave_addr = TWI_SLA_24CXX;

  /* modify device address, if upper memory area requested */
  if(eeaddr & MAX_ADDRESS_MASK_24CXX)
  {
    slave_addr |= ADDR_HIGH_24CXX;
  }

  /* send start condition */
  if(twi_start(slave_addr, TWI_WRITE)) goto fail;

  /* send address MSB followed by LSB */
  if(twi_write((eeaddr >> 8) & 0xFF )) goto fail;
  if(twi_write(eeaddr & 0xFF)) goto fail;

  for (; len > 0; len--)
  {
    if(twi_write(*buf)) goto fail;

    buf++;
    rv++;
  }

quit:
  twi_stop();
  return rv;

fail:
  rv=-1;
  goto quit;
}


/**
 * @brief Write 'len' bytes to EEPROM
 *
 * Wrapper around eeprom_write_page() that repeats calling this function until
 * either an error has been returned, or all bytes have been written.
 *
 * @param eeaddr  the EEPROM address where to write data
 * @param len     the length of data
 * @param buf     pointer to data which should be written
 *
 * @return        number of written bytes, or -1 if any error occured
 *
**/
int16_t eeprom_write_bytes(uint32_t eeaddr, uint16_t len, uint8_t *buf)
{
  int16_t rv;
  uint16_t total = 0;

  do
  {
    rv = eeprom_write_page(eeaddr, len, buf);
    if (rv == -1)
      return -1;
    eeaddr += rv;
    len -= rv;
    buf += rv;
    total += rv;
  }
  while (len > 0);

  return total;
}

