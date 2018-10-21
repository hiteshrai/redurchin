#include <string.h>
#include <stdio.h>

#include "usb_cdc.h"
#include "hardware.h"
#include "clock.h"
#include "analog.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define MCLK_FREQ              1600000U

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
	hardware_init();
	usb_cdc_init();
	clock_init();
	
	uint32_t last_usb_send_tick = clock_get_tick();
	analog_init();	
	
	analog_start(MCLK_FREQ, analog_ready_callback);
	uint32_t last_time = clock_get_tick();
	
	while (1)
	{
		usb_cdc_task();
		uint32_t current_time = clock_get_tick();
		
		if (analog_ready_data)
		{
			uint64_t reading_nV;
			if (analog_get_reading(&reading_nV))
			{
				if (clock_get_elapsed_time_ms(current_time, last_time) > 1000)
				{
					last_time = current_time;
					
					uint32_t integer_voltage = reading_nV / 1000000000;
					uint32_t fraction_voltage = reading_nV % 1000000000;
					char buffer[50];
					int len = snprintf(buffer, sizeof(buffer) - 1, "Voltage: %ld.%09ld\r\n", integer_voltage, fraction_voltage);
					usb_cdc_write((uint8_t *)buffer, len);	
				}				
			}
			analog_ready_data = false;
		}
	}
}
