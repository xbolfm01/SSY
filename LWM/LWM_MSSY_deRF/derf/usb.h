/**
 * @file usb.h
 *
 * @brief Application header file for usb.c.
 * 
 * Implements methods to get native access to USB hardware (FTDI chip).
 * You can read and write bytes over USB interface.  
 * 
 * $Id: usb.h,v 1.1 2011/04/08 14:23:36 dam Exp $
 *
 * @author    dresden elektronik: http://www.dresden-elektronik.de
 * @author    Support email: support@dresden-elektronik.de
 */
/*
 * Copyright (c) 2010, Dresden Elektronik All rights reserved.
 *
 * Licensed under Dresden Elektronik Limited License Agreement --> deEULA.txt
 */


/* Prevent double inclusion */
#ifndef USB_H_
#define USB_H_

/* === INCLUDES ============================================================ */
#include <stdio.h>      // FILE* type
#include "config_derf.h"     // include configuration

/* === MACROS / DEFINES ==================================================== */
#define PLATFORM DE_RF_NODE

/******************************************************************************
 * USB SPECIFIC DEFINITIONS FOR EACH PLATFORM
 ******************************************************************************/


#if (PLATFORM == SENS_TERM_BOARD)

#ifdef __AVR_ATmega1281__

/* USB Memory Mapped Address, which is only available on ATmega1281 since it have a XMEM interface */
#define USB_ADDRESS              (0x2200)

#endif

/* Port definition of USB data port */
#define USB_DATA_PORT         (PORTB)
#define USB_DATA_DDR          (DDRB)
#define USB_DATA_PINS         (PINB)

/* Port definition of USB address decoding port */
// PC6,7 -> PDx on RFA1
#define USB_DECODE_PORT       (PORTD)
#define USB_DECODE_DDR        (DDRD)
/* ChipSelect bits
 *  CS_usb  = 1 (CS1 = 0, CS0 = 0)
 *  CS_io   = 1 (CS1 = 0, CS0 = 1)
 *  CS_sram = 1 (CS1 = 1, CS0 = 0)
 */
#define USB_CS0_PIN           (PD6)
#define USB_CS1_PIN           (PD7)

/* Port definitions which gives indication of reception of byte */
#define USB_RXF_PIN           (PE7)
#define USB_RXF_PORT          (PORTE)
#define USB_RXF_DDR           (DDRE)
#define USB_RXF_PINS          (PINE)

/* Port definitions which gives indication of transmission of byte */
#define USB_TXE_PIN           (PE6)
#define USB_TXE_PORT          (PORTE)
#define USB_TXE_DDR           (DDRE)
#define USB_TXE_PINS          (PINE)

/* Port definitions for write and read access on USB */
#ifdef __AVR_ATmega1281__
#define USB_WR_PIN            (PG1)
#define USB_WR_PORT           (PORTG)
#define USB_WR_DDR            (DDRG)
#define USB_WR_PINS           (PING)
#else // __AVR_ATmega128RFA1__
#define USB_WR_PIN            (PE4)
#define USB_WR_PORT           (PORTE)
#define USB_WR_DDR            (DDRE)
#define USB_WR_PINS           (PINE)
#endif

#ifdef __AVR_ATmega1281__
#define USB_RD_PIN            (PD4)
#define USB_RD_PORT           (PORTD)
#define USB_RD_DDR            (DDRD)
#define USB_RD_PINS           (PIND)
#else // __AVR_ATmega128RFA1__
#define USB_RD_PIN            (PE5)
#define USB_RD_PORT           (PORTE)
#define USB_RD_DDR            (DDRE)
#define USB_RD_PINS           (PINE)
#endif

/*
 * The following definitions are for deRFnode platform
 *
 * Due to backward compability of radio modules the USB pins are not located at one single port
 * They are connected to different pins on radio modules, therefore another definition is required
 */
#elif (PLATFORM == DE_RF_NODE)

/* Port definitions which gives indication of reception of byte */
#define USB_RXF_PIN                 (PE2)
#define USB_RXF_PORT                (PORTE)
#define USB_RXF_DDR                 (DDRE)
#define USB_RXF_PINS                (PINE)

/* Port definitions which gives indication of transmission of byte */
#define USB_TXE_PIN                 (PB5)
#define USB_TXE_PORT                (PORTB)
#define USB_TXE_DDR                 (DDRB)
#define USB_TXE_PINS                (PINB)

/* Port definitions for write and read access on USB */

#if (!(defined REV01) | (defined REV00))
#define USB_WR_PIN                  (PD3)
#define USB_WR_PORT                 (PORTD)
#define USB_WR_DDR                  (DDRD)
#define USB_WR_PINS                 (PIND)

#define USB_RD_PIN                  (PD2)
#define USB_RD_PORT                 (PORTD)
#define USB_RD_DDR                  (DDRD)
#define USB_RD_PINS                 (PIND)
#else
#define USB_WR_PIN                  (PG1)
#define USB_WR_PORT                 (PORTG)
#define USB_WR_DDR                  (DDRG)
#define USB_WR_PINS                 (PING)

#define USB_RD_PIN                  (PD4)
#define USB_RD_PORT                 (PORTD)
#define USB_RD_DDR                  (DDRD)
#define USB_RD_PINS                 (PIND)
#endif

/* Port definitions of data port from USB interface */
#define USB_D0_BIT                  (0)
#define USB_D0_PIN                  (PB0)
#define USB_D0_PORT                 (PORTB)
#define USB_D0_DDR                  (DDRB)
#define USB_D0_PINS                 (PINB)

#define USB_D1_BIT                  (1)
#define USB_D1_PIN                  (PF2)
#define USB_D1_PORT                 (PORTF)
#define USB_D1_DDR                  (DDRF)
#define USB_D1_PINS                 (PINF)

#define USB_D2_BIT                  (2)
#define USB_D2_PIN                  (PD5)
#define USB_D2_PORT                 (PORTD)
#define USB_D2_DDR                  (DDRD)
#define USB_D2_PINS                 (PIND)

#define USB_D3_BIT                  (3)
#define USB_D3_PIN                  (PG2)
#define USB_D3_PORT                 (PORTG)
#define USB_D3_DDR                  (DDRG)
#define USB_D3_PINS                 (PING)

#define USB_D4_BIT                  (4)
#define USB_D4_PIN                  (PE6)
#define USB_D4_PORT                 (PORTE)
#define USB_D4_DDR                  (DDRE)
#define USB_D4_PINS                 (PINE)

#define USB_D5_BIT                  (5)
#define USB_D5_PIN                  (PB4)
#define USB_D5_PORT                 (PORTB)
#define USB_D5_DDR                  (DDRB)
#define USB_D5_PINS                 (PINB)

#define USB_D6_BIT                  (6)
#define USB_D6_PIN                  (PE7)
#define USB_D6_PORT                 (PORTE)
#define USB_D6_DDR                  (DDRE)
#define USB_D6_PINS                 (PINE)

#define USB_D7_BIT                  (7)
#define USB_D7_PIN                  (PB6)
#define USB_D7_PORT                 (PORTB)
#define USB_D7_DDR                  (DDRB)
#define USB_D7_PINS                 (PINB)

#endif // (PLATFORM == SENS_TERM_BOARD)

/* === TYPES =============================================================== */

/* === PROTOTYPES ========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

void usb_init(void);
uint8_t usb_keypressed(void);
int usb_putc_std (char c, FILE* dummy_file);
int usb_getc_std (FILE* dummy_file);

#ifdef __cplusplus
} /* extern "C" */
#endif

/* === INLINE FUNCTIONS ==================================================== */


#endif /* USB_H_ */

/* EOF */
