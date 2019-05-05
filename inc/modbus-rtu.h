/*
 * @Description: 
 * @Author: zpw
 * @LastEditors: zpw
 * @Date: 2019-04-30 15:00:45
 * @LastEditTime: 2019-04-30 16:57:55
 */
/*
 * Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "modbus.h"
#include <rtdevice.h>

MODBUS_BEGIN_DECLS

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * RS232 / RS485 ADU = 253 bytes + slave (1 byte) + CRC (2 bytes) = 256 bytes
 */
#define MODBUS_RTU_MAX_ADU_LENGTH  256

MODBUS_API modbus_t *modbus_new_rtu(struct rt_serial_device *serial, struct serial_configure *config);

#define MODBUS_RTU_RS232 0
#define MODBUS_RTU_RS485 1

MODBUS_API int modbus_rtu_set_serial_mode(modbus_t *ctx, int mode);
MODBUS_API int modbus_rtu_get_serial_mode(modbus_t *ctx);

#define MODBUS_RTU_RTS_NONE   0
#define MODBUS_RTU_RTS_UP     1
#define MODBUS_RTU_RTS_DOWN   2

MODBUS_API int modbus_rtu_set_rts(modbus_t *ctx, long rts_pin, int mode);
MODBUS_API int modbus_rtu_get_rts(modbus_t *ctx);

MODBUS_API int modbus_rtu_set_custom_rts(modbus_t *ctx, void (*set_rts) (modbus_t *ctx, int on));

MODBUS_API int modbus_rtu_set_rts_delay(modbus_t *ctx, int us);
MODBUS_API int modbus_rtu_get_rts_delay(modbus_t *ctx);

MODBUS_END_DECLS

#endif /* MODBUS_RTU_H */
