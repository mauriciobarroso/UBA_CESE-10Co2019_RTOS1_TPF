/*
 * user_main.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Mauricio Barroso
 */

/*==================[inclusions]=============================================*/

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

/*==================[macros]=================================================*/
/* ESP8266 */
#define ESP_INTR_FLAG_DEFAULT	0
#define PIN_REFRESH				5
#define PIN_BUTTON				0
/* ubicación en la EEPROM */
#define EEPROM_PULSE_COUNT		0x000
#define EEPROM_DATE				0x200
#define EEPROM_MONTH			0x202
#define EEPROM_YEAR				0x204
#define EEPROM_EEPROM_INDEX		0x500
#define EEPROM_USER_ID			0x510
#define EEPROM_DEVICE_SETTINGS	0x204

/*==================[typedef]================================================*/
/* origen de inforamación */
typedef enum
{
	dRtcTimeDate,
	dEepromData
} dData_t;
/* información de los periféricos RTC y EEPROM */
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
	/* declaración de variables locales */
	BaseType_t xStatus;
	PeripheralData_t xPeripheralStruct;
	uint8_t pulse_count = 0;
	/* establece que la fuente de inforamción es la EEPROM */
	xPeripheralStruct.dDataType = dEepromData;
	/* lee la posisición de la EEPROM donde se almacena el conteo de pulsos y lo guarda en pulse_count */
	at24c32_read_random( EEPROM_PULSE_COUNT, &pulse_count );
	for( ;; )
	{
		/* se toma el semáforo xSemaphoreButton*/
		xStatus = xSemaphoreTake( xSemaphoreButton, portMAX_DELAY );
		/* manejo de errores si no se puede tomar el semáforox SemaphoreButton */
		if( xStatus == pdTRUE)
		{
			/* incrementa en 1 pulse_count, lo guarda en la EEPROM y lo manda a la cola xQueue */
			++pulse_count;
			at24c32_write_byte( EEPROM_PULSE_COUNT, &pulse_count );
			xPeripheralStruct.pvData = ( void * ) pulse_count;
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
	/* declaración de variables locales */
	uint8_t data[ 7 ];
	PeripheralData_t xPeripheralStruct;
	BaseType_t xStatus;
	/* establece que la fuente de inforamción es el RTC */
	xPeripheralStruct.dDataType = dRtcTimeDate;

	for( ;; )
	{
		/* se toma el semáforo xSemaphoreRefresh*/
		xStatus = xSemaphoreTake( xSemaphoreRefresh, portMAX_DELAY );
		/* manejo de errores si no se puede tomar el semáforox SemaphoreRefresh */
		if( xStatus == pdTRUE)
		{
			/* obtiene los datos del RTC y los manda a la cola xQueue */
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
	/* declaración de variables locales */
	PeripheralData_t xReceivedPeripheral;
	BaseType_t xStatus;
	uint8_t * data;
	/* ciclo infinito */
	for( ;; )
	{
		xStatus = xQueueReceive( xQueue, &xReceivedPeripheral, portMAX_DELAY );
		/* manejo de errores al recibir de la cola */
		if( xStatus == pdPASS )
		{
			data = xReceivedPeripheral.pvData;
			/* selecciona la acción en función de la fuente de información */
			switch( xReceivedPeripheral.dDataType )
			{
				case dRtcTimeDate:
					/* envía por la UART los datos del RTC*/
			 		ESP_LOGI( TAG, "Recibido del RTC\r" );
			 		ESP_LOGI( TAG, "Date:%02x/%02x/%02x\r", data[ DS1307_DATE_ADDR ], data[ DS1307_MONTH_ADDR ], data[ DS1307_YEAR_ADDR ] );
			 		ESP_LOGI( TAG, "Time:%02x:%02x:%02x\r\n", data[ DS1307_HOURS_ADDR ], data[ DS1307_MINUTES_ADDR ], data[ DS1307_SECONDS_ADDR ] );
			 		break;
			 	case dEepromData:
			 		/* envía por la UART el conteo de pulsos almacenado en la EEPROM */
			 		ESP_LOGI( TAG, "Recibido de la EEPROM\r" );
			 		ESP_LOGI( TAG, "Conteo de pulsos:%d\r\n", (uint8_t) data);
			 		break;
			 }
		}

		else
		{
			ESP_LOGI( TAG, "No se pudo recibir de la cola xQueue\r\n" );
		}
	}
}

/*========================[main]=============================================*/

void app_main()
{
	/* funciones de inicialización */
	vInitGPIO();
	ds1307_init( I2C_MASTER_NUM );
	/* creación de la cola xQueue*/
	xQueue = xQueueCreate( 2, sizeof( PeripheralData_t ) );
	/* manejo de errores en la creación de la cola */
	if( xQueue != NULL )
	{
		/* creación de los semáforos binarios */
		xSemaphoreRefresh = xSemaphoreCreateBinary();
		xSemaphoreButton = xSemaphoreCreateBinary();
		/* creación de las tareas */
		xTaskCreate( task_rtc, "rtc", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL );
		xTaskCreate( task_show, "show", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL );
		xTaskCreate( task_eeprom, "eeprom", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL );
		/* el scheduler es iniciado en las tareas de inicialización del esp8266 */
	}
}
