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

#include "driver/ds1307.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[internal data declaration]==============================*/

/*==================[external data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static esp_err_t i2c_init();
static esp_err_t ds1307_write( i2c_port_t i2c_num, uint8_t reg_address, uint8_t * data, size_t data_len );
static esp_err_t ds1307_read( i2c_port_t i2c_num, uint8_t reg_address, uint8_t * data, size_t data_len );

/*==================[external functions definition]=========================*/

extern esp_err_t ds1307_read_datetime( uint8_t * datetime )
{
	int32_t ret;

	ret = ds1307_read( I2C_MASTER_NUM, DS1307_SECONDS_ADDR, datetime, 7 );

	return ret;
}

extern esp_err_t ds1307_set_datetime( uint8_t * datetime )
{
	int32_t ret;

	ret = ds1307_write( I2C_MASTER_NUM, DS1307_SECONDS_ADDR, datetime, 7 );

	return ret;
}

extern esp_err_t ds1307_read_control( uint8_t * control )
{
	int32_t ret;

	ret = ds1307_read( I2C_MASTER_NUM, DS1307_CONTROL_ADDR, control, 1 );

	return ret;
}

extern esp_err_t ds1307_set_control( uint8_t control )
{
	int32_t ret;

	ret = ds1307_write( I2C_MASTER_NUM, DS1307_CONTROL_ADDR, &control, 1 );

	return ret;
}

extern esp_err_t ds1307_init( i2c_port_t i2c_num )
{
	uint8_t rtc_datetime[7];

	//TickType_t ticks_to_wait = 1000 / portTICK_RATE_MS;
	i2c_init();

	ds1307_read_datetime(rtc_datetime);

	if(rtc_datetime[DS1307_SECONDS_ADDR] == 0x80)
	{
		for( uint8_t index = 0; index < 7; index++)
		{
			rtc_datetime[index] = 0x01;
		}

		ds1307_set_datetime( rtc_datetime );
	}

	ds1307_set_control( OUTPUT_1HZ );

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

static esp_err_t ds1307_write( i2c_port_t i2c_num, uint8_t reg_address, uint8_t * data, size_t data_len )
{
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DS1307_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t ds1307_read( i2c_port_t i2c_num, uint8_t reg_address, uint8_t * data, size_t data_len )
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, DS1307_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (ret != ESP_OK)
	{
		return ret;
	}

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte( cmd, DS1307_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}

/*==================[end of file]============================================*/
