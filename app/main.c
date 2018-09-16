#include <string.h>

#include "usb_cdc.h"
#include "hardware.h"
#include "clock.h"

static void rx_callback(uint8_t *data, uint32_t length)
{
	char * transmit_string = "HELLO HITESH HOW ARE YOU???\r\n";
	uint32_t transmit_string_length = strlen(transmit_string);	
}

int main(void)
{
	hardware_init();
	usb_cdc_init();
	clock_init();
	usb_cdc_set_receive_callback(rx_callback);
	
	char * transmit_string = "HELLLO HITESH HOW ARE YOU???\r\n";
	uint32_t transmit_string_length = strlen(transmit_string);
	
	uint32_t last_usb_send_tick = clock_get_tick();
	
	while (1)
	{
		if (clock_get_elapsed_time_ms(clock_get_tick(), last_usb_send_tick) >= 5000)
		{
			usb_cdc_write((uint8_t *)transmit_string, transmit_string_length);
			last_usb_send_tick = clock_get_tick();
		}
		
		usb_cdc_task();
	}
}
