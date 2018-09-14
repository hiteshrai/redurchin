#include "usb_cdc.h"
#include "hardware.h"
#include <string.h>

static void rx_callback(uint8_t *data, uint32_t length)
{
	char * transmit_string = "HELLO HITESH HOW ARE YOU???\r\n";
	uint32_t transmit_string_length = strlen(transmit_string);
	
	usb_cdc_write((uint8_t *)transmit_string, transmit_string_length);
}

int main(void)
{
	hardware_init();
	usb_cdc_init();
	usb_cdc_set_receive_callback(rx_callback);
	
	while (1)
	{
		usb_cdc_task();
	}
}
