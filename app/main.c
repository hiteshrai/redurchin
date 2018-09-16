#include <string.h>
#include <stdio.h>

#include "usb_cdc.h"
#include "hardware.h"
#include "clock.h"
#include "analog.h"

/*******************************************************************************
* Definitions
******************************************************************************/


/*******************************************************************************
* Variables
******************************************************************************/
static volatile bool analog_ready_data = false;

/*******************************************************************************
* Prototypes
******************************************************************************/

/*******************************************************************************
* Code
******************************************************************************/
static void analog_ready_callback(void)
{
	// Set the flag indicating that an analog read is available
	analog_ready_data = true;
}

int main(void)
{
	uint32_t reading = 0;
	
	hardware_init();
	usb_cdc_init();
	clock_init();
	
	uint32_t last_usb_send_tick = clock_get_tick();
	uint8_t command = 0x01;
	analog_init();	
	
	analog_start(1000, analog_ready_callback);
	
	while (1)
	{
		usb_cdc_task();
		
		if (analog_ready_data)
		{
			reading = analog_get_reading();
			analog_ready_data = false;
			
			char buffer[50];
			int len = snprintf(buffer, sizeof(buffer) - 1, "Voltage: %d\r\n", (int)reading);
			usb_cdc_write((uint8_t *)buffer, len);
		}
		
		if (clock_get_elapsed_time_ms(clock_get_tick(), last_usb_send_tick) >= 1000)
		{
			usb_cdc_write((uint8_t *)"HELLO\r\n", 7);
			last_usb_send_tick = clock_get_tick();
		}
	}
}
