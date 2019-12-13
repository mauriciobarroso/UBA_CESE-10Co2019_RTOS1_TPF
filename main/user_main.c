/*
 * user_main.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Mauricio Barroso
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/ds1307.h"
#include "driver/at24c32.h"

#define ESP_INTR_FLAG_DEFAULT	0
#define PIN_REFRESH				5
#define PIN_BUTTON				0

#define EEPROM_PULSE_COUNT		0x000
#define EEPROM_DATE				0x200
#define EEPROM_MONTH			0x202
#define EEPROM_YEAR				0x204
#define EEPROM_EEPROM_INDEX		0x500
#define EEPROM_USER_ID			0x510
#define EEPROM_DEVICE_SETTINGS	0x204
/*
static uint16_t PULSE_COUNT = 0;
static uint8_t DATE = 0x00;
static uint8_t MONTH = 0x00;
static uint8_t YEAR = 0x00;
static uint16_t EEPROM_INDEX = 0x000;
static uint32_t USER_ID = 0x00000000;
static uint16_t DEVICE_SETTINGS = 0;
*/

typedef enum
{
	dRtcTimeDate,
	dEepromData
} dData_t;

/*==================[typedef]================================================*/

typedef struct PERIPHERAL_DATA
{
	dData_t dDataType;
	void *pvData;
} PeripheralData_t;

/*==================[data declaration]================================================*/

static const char *TAG = "main";
SemaphoreHandle_t xSemaphoreRefresh = NULL;
SemaphoreHandle_t xSemaphoreButton = NULL;
QueueHandle_t xQueue;

/*==================[function declaration ]================================================*/

void IRAM_ATTR refresh_isr_handler( void *arg )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xSemaphoreRefresh, xHigherPriorityTaskWoken );

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void IRAM_ATTR button_isr_handler( void *arg )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xSemaphoreButton, xHigherPriorityTaskWoken );

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void vInitGPIO( )
{
	/* configura PIN_REFRESH como entrada y habilita la interrupción */
	gpio_set_direction( PIN_REFRESH, GPIO_MODE_INPUT );
	gpio_install_isr_service( ESP_INTR_FLAG_DEFAULT );
	gpio_isr_handler_add( PIN_REFRESH, refresh_isr_handler, NULL );
	gpio_set_intr_type(PIN_REFRESH, GPIO_INTR_NEGEDGE);
	/* configura PIN_BUTTON como entrada y habilita la interrupción */
	gpio_set_direction( PIN_BUTTON, GPIO_MODE_INPUT );
	gpio_install_isr_service( ESP_INTR_FLAG_DEFAULT );
	gpio_isr_handler_add( PIN_BUTTON, button_isr_handler, NULL );
	gpio_set_intr_type(PIN_BUTTON, GPIO_INTR_NEGEDGE);
}

static void task_eeprom( void *arg )
{
	BaseType_t xStatus;
	PeripheralData_t xPeripheralStruct;
	uint8_t pulse_count = 0;

	xPeripheralStruct.dDataType = dEepromData;

	for( ;; )
	{
		ESP_LOGI( TAG, "task_eeprom\r\n" );
		/* lee una posisición de la memoria eeprom y la guarda en pulse_count */
		at24c32_read_random( EEPROM_PULSE_COUNT, &pulse_count );

		xStatus = xSemaphoreTake( xSemaphoreButton, portMAX_DELAY );

		if( xStatus == pdTRUE)
		{
			++pulse_count;
			at24c32_write_byte( EEPROM_PULSE_COUNT, &pulse_count );
			xPeripheralStruct.pvData = ( void * ) pulse_count;
			/* manda a la pulse_count a la cola */
			xQueueSendToBack( xQueue, &xPeripheralStruct, 0 );
		}

		else
		{
			ESP_LOGI( TAG, "No se pudo tomar el semáforo xSemaphoreButton" );
		}
	}
}

static void task_rtc( void *arg )
{
	uint8_t data[ 7 ];
	PeripheralData_t xPeripheralStruct;
	BaseType_t xStatus;

	xPeripheralStruct.dDataType = dRtcTimeDate;

	for( ;; )
	{
		ESP_LOGI( TAG, "task_rtc\r\n" );

		xStatus = xSemaphoreTake( xSemaphoreRefresh, portMAX_DELAY );

		if( xStatus == pdTRUE)
		{
			ds1307_read_datetime( data );

			xPeripheralStruct.pvData = ( void *) data;

			xQueueSendToBack( xQueue, &xPeripheralStruct, 0 );
		}

		else
		{
			ESP_LOGI( TAG, "No se pudo tomar el semáforo xSemaphoreButton" );
		}
	}
}

static void task_show( void *arg )
{
	PeripheralData_t xReceivedPeripheral;
	BaseType_t xStatus;
	uint8_t * data;

	for( ;; )
	{
		ESP_LOGI( TAG, "task_show\r\n" );

		xStatus = xQueueReceive( xQueue, &xReceivedPeripheral, portMAX_DELAY );

		if( xStatus == pdPASS )
		{
			data = xReceivedPeripheral.pvData;

			switch( xReceivedPeripheral.dDataType )
			{
				case dRtcTimeDate:
			 		ESP_LOGI( TAG, "Recibido del RTC\r" );
			 		ESP_LOGI( TAG, "Date:%02x/%02x/%02x\r", data[ DS1307_DATE_ADDR ], data[ DS1307_MONTH_ADDR ], data[ DS1307_YEAR_ADDR ] );
			 		ESP_LOGI( TAG, "Time:%02x:%02x:%02x\r\n", data[ DS1307_HOURS_ADDR ], data[ DS1307_MINUTES_ADDR ], data[ DS1307_SECONDS_ADDR ] );
			 		break;
			 	case dEepromData:
			 		ESP_LOGI( TAG, "Recibido de la EEPROM\r" );
			 		ESP_LOGI( TAG, "Conteo de pulsos:%d\r\n", (uint8_t) data);
			 		break;
			 }
		}
	}
}

void app_main()
{
	vInitGPIO();
	ds1307_init( I2C_MASTER_NUM );

	xQueue = xQueueCreate( 10, sizeof( PeripheralData_t ) );

	if( xQueue != NULL )
	{
		xSemaphoreRefresh = xSemaphoreCreateBinary();
		xSemaphoreButton = xSemaphoreCreateBinary();

		xTaskCreate( task_rtc, "rtc", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL );
		xTaskCreate( task_show, "show", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL );
		xTaskCreate( task_eeprom, "eeprom", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL );

	}
}
