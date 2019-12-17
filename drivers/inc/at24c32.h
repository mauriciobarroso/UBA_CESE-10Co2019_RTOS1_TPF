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

#ifndef _AT24C32_H_
#define _AT24C32_H_

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

#define AT24C32_ADDR				0x50             /* slave address for 24C32 EEPROM */

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern esp_err_t at24c32_write_byte( uint16_t reg_address, uint8_t * data );
extern esp_err_t at24c32_write_page( uint16_t reg_address, uint8_t * data, size_t page_size );
extern esp_err_t at24c32_read_current( uint8_t * data );
extern esp_err_t at24c32_read_random( uint16_t reg_address, uint8_t * data );
//extern esp_err_t at24c32_read_sequential( uint16_t reg_address, uint8_t * data );
extern esp_err_t at24c32_init( i2c_port_t i2c_num );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/

#endif /* #ifndef _AT24C32_H_ */
