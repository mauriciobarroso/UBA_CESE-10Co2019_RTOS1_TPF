/* Copyright 2019, Mauricio Barroso
 * All rights reserved.
 *
 * This file is part of arquitecturaDeMicroprocesadores.
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

/*==================[inlcusions]============================================*/

#include "driver/at24c32.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[internal data declaration]==============================*/

/*==================[external data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
static esp_err_t i2c_init();
static esp_err_t at24c32_write( i2c_port_t i2c_num, uint8_t reg_address_high, uint8_t reg_address_low, uint8_t * data, size_t data_len );
static esp_err_t at24c32_read( i2c_port_t i2c_num, uint8_t * data );

/*==================[external functions definition]=========================*/

extern esp_err_t at24c32_write_byte( uint16_t reg_address, uint8_t * data )
{
	int32_t ret;
	uint8_t reg_address_low, reg_address_high;

	reg_address_low = ( uint8_t )( reg_address & 0x00FF );
	reg_address_high = ( uint8_t )( ( reg_address & 0xFF00 ) >> 8 );

	ret = at24c32_write( I2C_MASTER_NUM, reg_address_high, reg_address_low, data, 1 );

	return ret;
}

extern esp_err_t at24c32_write_page( uint16_t reg_address, uint8_t * data, size_t page_size )
{
	int32_t ret;
	uint8_t reg_address_low, reg_address_high;

	reg_address_low = ( uint8_t )( reg_address & 0x00FF );
	reg_address_high = ( uint8_t )( ( reg_address & 0xFF00 ) >> 8 );

	ret = at24c32_write( I2C_MASTER_NUM, reg_address_high, reg_address_low, data, page_size );

	return ret;
}

extern esp_err_t at24c32_read_current( uint8_t * data )
{
	int32_t ret;

	ret = at24c32_read( I2C_MASTER_NUM, data );

	return ret;
}

extern esp_err_t at24c32_read_random( uint16_t reg_address, uint8_t * data )
{
	uint32_t ret;
	uint8_t reg_address_low, reg_address_high;

	reg_address_low = ( uint8_t )( reg_address & 0x00FF );
	reg_address_high = ( uint8_t )( ( reg_address & 0xFF00 ) >> 8 );

	ret = at24c32_write( I2C_MASTER_NUM, reg_address_high, reg_address_low, data, 0 );

	if(ret != ESP_OK)
	{
		return ret;
	}

	ret = at24c32_read( I2C_MASTER_NUM, data );

	return ret;
}

//extern esp_err_t at24c32_read_sequential( uint16_t reg_address, uint8_t * data )

extern esp_err_t at24c32_init( i2c_port_t i2c_num )
{
	i2c_init();

	return ESP_OK;
}

/*==================[internal functions definition]==========================*/

static esp_err_t i2c_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK( i2c_driver_install( i2c_master_port, conf.mode ));
    ESP_ERROR_CHECK( i2c_param_config( i2c_master_port, &conf ));

    return ESP_OK;
}

static esp_err_t at24c32_write( i2c_port_t i2c_num, uint8_t reg_address_high, uint8_t reg_address_low, uint8_t * data, size_t data_len )
{
	int ret;

	if( data_len > 32)
	{
		ret = ESP_FAIL;
		return ret;
	}

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start( cmd ); // start condition
    i2c_master_write_byte( cmd, AT24C32_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN ); // device address
    i2c_master_write_byte( cmd, reg_address_high, ACK_CHECK_EN ); // first word address
    i2c_master_write_byte( cmd, reg_address_low, ACK_CHECK_EN ); // second word address
    for( uint8_t index = 0; index < data_len; index++ )
    {
    	i2c_master_write_byte( cmd, data[index], ACK_CHECK_EN ); // data
    }
    //i2c_master_write( cmd, data, data_len, ACK_CHECK_EN ); // data
    i2c_master_stop( cmd ); // stop condition
    ret = i2c_master_cmd_begin( i2c_num, cmd, 1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete( cmd );

    return ret;
}

static esp_err_t at24c32_read( i2c_port_t i2c_num, uint8_t * data )
{
	int ret;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start( cmd ); // start condition
	i2c_master_write_byte( cmd, AT24C32_ADDR << 1 | READ_BIT, ACK_CHECK_EN ); // device address
	i2c_master_read_byte( cmd, data, LAST_NACK_VAL ); // data
	i2c_master_stop( cmd ); // stop condition
	ret = i2c_master_cmd_begin( i2c_num, cmd, 1000 / portTICK_RATE_MS );
	i2c_cmd_link_delete( cmd );

	return ret;
}

/*==================[end of file]============================================*/
