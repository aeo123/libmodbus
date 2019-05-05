/*
 * @Description: 
 * @Author: zpw
 * @LastEditors: zpw
 * @Date: 2019-04-19 21:06:54
 * @LastEditTime: 2019-04-30 16:57:46
 */
/*
 * Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#ifndef MODBUS_RTU_PRIVATE_H
#define MODBUS_RTU_PRIVATE_H


#include <stdint.h>


#define _MODBUS_RTU_HEADER_LENGTH      1
#define _MODBUS_RTU_PRESET_REQ_LENGTH  6
#define _MODBUS_RTU_PRESET_RSP_LENGTH  2

#define _MODBUS_RTU_CHECKSUM_LENGTH    2


typedef struct _modbus_rtu {
    /* Device: rt device */
    void *device;
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    uint32_t baud;
    /* Data bit */
    uint32_t data_bit;
    /* Stop bit */
    uint32_t stop_bit;
    /* Parity: 'N', 'O', 'E' */
    uint32_t parity;

#if HAVE_DECL_TIOCSRS485
    int serial_mode;
#endif
#if HAVE_DECL_TIOCM_RTS
    int rts;
    int rts_delay;
    int onebyte_time;
    long rts_pin;
    void (*set_rts) (modbus_t *ctx, int on);
#endif
    /* To handle many slaves on the same link */
    int confirmation_to_ignore;
} modbus_rtu_t;

#endif /* MODBUS_RTU_PRIVATE_H */
