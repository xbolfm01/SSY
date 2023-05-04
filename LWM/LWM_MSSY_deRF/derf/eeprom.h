/**
 * @file eeprom.h
 *
 * @brief Application header file for eeprom.c.
 *
 * Implements methods to get native access to EEPROM hardware.
 * You can read and write bytes to and from EEPROM.
 *
 * $Id: eeprom.h,v 1.1 2011/04/08 14:23:37 dam Exp $
 *
 * @author    dresden elektronik: http://www.dresden-elektronik.de
 * @author    Support email: support@dresden-elektronik.de
 *
 * Copyright (c) 2010, Dresden Elektronik All rights reserved.
 *
 * Licensed under Dresden Elektronik Limited License Agreement --> deEULA.txt
 */


/* Prevent double inclusion */
#ifndef EEPROM_H_
#define EEPROM_H_

/* === INCLUDES ============================================================ */
#include <util/delay.h>
#include "config.h"     		// include configuration

/* === MACROS / DEFINES ==================================================== */

#if !defined(__AVR_ATmega128RFA1__)
#error "EEPROM does not work with this MCU - only with deRFmega128 Rev. 2 (ATmega128RFA1)"
#endif // !defined(__AVR_ATmega128RFA1__)

/** @brief Macro to easy enable EEPROM, delay since Vcc is switched  */
#define EEPROM_ENABLE()       (PORTD &= ~_BV(PD6), DDRD |= _BV(PD6), _delay_us(5))
/** @brief Macro to easy disable EEPROM */
#define EEPROM_DISABLE()      (_delay_us(5), PORTD |= _BV(PD6),  DDRD |= _BV(PD6))

/**
 * TWI address for AT24C1024 EEPROM:
 * <br>---------------------------------------
 * <br>| 1 | 0 | 1 | 0 | A2 | A1 | P0 | R/~W |
 * <br>---------------------------------------
 * <br>
 * A2/A1 = address selection <br>
 * P0    = highest address bit (17th bit of address) <br>
 * R/~W  = Read / Write <br>
 *
 * Address is one bit shifted since library expects this.
 */
#define TWI_SLA_24CXX               ((0xA0)>>1)

/** maximum address number mask (AT24C1024 = 17 Bit address space -> masks 17. bit) */
#define MAX_ADDRESS_MASK_24CXX      ((0x10000)>>1)

/** highest address bit position at SLA ("P0") */
#define ADDR_HIGH_24CXX             (0x02>>1)


/**
 * Number of bytes that can be written in a row, see comments for
 * eeprom_write_page() below.  Some vendor's devices would accept 16,
 * but 8 seems to be the lowest common denominator.
 *
 * Note that the page size must be a power of two, this simplifies the
 * page boundary calculations below.
 *
 */
//#define PAGE_SIZE                   (8)         /* 24C01 .. 24C16 */
//#define PAGE_SIZE                   (64)        /* 24C128 */
#define PAGE_SIZE                   (256UL)     /* AT24C1024 */

/** Number of pages (on AT24C1024) */
#define NUMBER_PAGES                (512UL)     /* AT24C1024 */

/** Last page address (on AT24C1024) */
#define LAST_PAGE_ADDRESS           ((PAGE_SIZE * NUMBER_PAGES) - PAGE_SIZE)     /* AT24C1024 */

/** First page address (on AT24C1024) - dummy value */
#define FIRST_PAGE_ADDRESS 0UL     /* AT24C1024 */

/* === TYPES =============================================================== */

/* === PROTOTYPES ========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

void eeprom_init(void);
int32_t eeprom_read_bytes(uint32_t eeaddr, uint16_t len, uint8_t *buf);
int16_t eeprom_write_bytes(uint32_t eeaddr, uint16_t len, uint8_t *buf);

#ifdef __cplusplus
} /* extern "C" */
#endif

/* === INLINE FUNCTIONS ==================================================== */


#endif /* EEPROM_H_ */

/* EOF */
