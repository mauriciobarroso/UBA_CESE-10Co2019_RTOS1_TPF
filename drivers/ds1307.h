/* Copyright 2019, Mauricio Barroso
 * All rights reserved.
 *
 * This file is part of EMMON.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Date: 10/12/19 */

#ifndef _DS1307_H_
#define _DS1307_H_

/*==================[inclusions]=============================================*/

#include <stdint.h>
#include "driver/i2c.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

#ifndef I2C_MASTER_SCL_IO
#define I2C_MASTER_SCL_IO			2                /* gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           14               /* gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /* I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0                /* I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                /* I2C master do not need buffer */
#endif

#ifndef WRITE_BIT
#define WRITE_BIT                   I2C_MASTER_WRITE /* I2C master write */
#define READ_BIT                    I2C_MASTER_READ  /* I2C master read */
#define ACK_CHECK_EN                0x1              /* I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0              /* I2C master will not check ack from slave */
#define ACK_VAL                     0x0              /* I2C ack value */
#define NACK_VAL                    0x1              /* I2C nack value */
#define LAST_NACK_VAL               0x2              /* I2C last_nack value */
#endif

#define DS1307_ADDR					0x68             /* slave address for DS1307 RTC */
#define DS1307_SECONDS_ADDR			0x0				 /* SECONDS address for DS1307 RTC */
#define DS1307_MINUTES_ADDR			0x1				 /* MINUTES address for DS1307 RTC */
#define DS1307_HOURS_ADDR			0x2				 /* HOURS address for DS1307 RTC */
#define DS1307_DAY_ADDR				0x3				 /* DAY address for DS1307 RTC */
#define DS1307_DATE_ADDR 			0x4				 /* DATE address for DS1307 RTC */
#define DS1307_MONTH_ADDR			0x5				 /* MONTH address for DS1307 RTC */
#define DS1307_YEAR_ADDR			0x6				 /* YEAR address for DS1307 RTC */
#define DS1307_CONTROL_ADDR			0x7				 /* CONTROL address for DS1307 RTC */

#define OUTPUT_1HZ					0x10			 /*set SQW/OUT output to 1Hz */
#define OUTPUT_4096KHZ				0x11			 /*set SQW/OUT output to 4096KHz */
#define OUTPUT_8192KHZ				0x12			 /*set SQW/OUT output to 8192KHz */
#define OUTPUT_32768KHZ				0x13			 /*set SQW/OUT output to 32768KHz */
#define OUTPUT_LOW					0x0				 /*set SQW/OUT output to 0 */
#define OUTPUT_HIGH					0x90			 /*set SQW/OUT output to 1 */

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern esp_err_t ds1307_read_datetime( uint8_t * datetime );
extern esp_err_t ds1307_set_datetime( uint8_t * datetime );
extern esp_err_t ds1307_read_control( uint8_t control );
extern esp_err_t ds1307_set_control( uint8_t control );
extern esp_err_t ds1307_init( i2c_port_t i2c_num );
/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/

#endif /* #ifndef _DS1307_H_ */
