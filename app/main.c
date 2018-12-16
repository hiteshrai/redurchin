#include <string.h>
#include <stdio.h>

#include "usb_cdc.h"
#include "hardware.h"
#include "clock.h"
#include "analog.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define MCLK_FREQ              100000U
#define HARDWARE_DOWNSAMPLING  32

/*******************************************************************************
* Variables
******************************************************************************/
static volatile bool analog_ready_data = false;
static uint32_t sample_frequency = 1;
static uint32_t transfer_sample_count;

/*******************************************************************************
* Prototypes
******************************************************************************/

/*******************************************************************************
* Code
******************************************************************************/
static uint32_t get_sample_count_from_frequency(uint32_t frequency)
{
	uint32_t max_frequency = MCLK_FREQ / HARDWARE_DOWNSAMPLING;
	if (frequency > max_frequency)
	{
		frequency = max_frequency;
	}
	uint32_t samples = MCLK_FREQ / HARDWARE_DOWNSAMPLING / frequency;
	return samples;
}

static void analog_ready_callback(void)
{
	// Set the flag indicating that an analog read is available
	analog_ready_data = true;
}

int main(void)
{
	uint64_t reading_nV_sum = 0;
	bool missed_reading = false;
	uint32_t sample_count = 0;

	hardware_init();
	usb_cdc_init();
	clock_init();
	
	analog_init();		
	analog_start(MCLK_FREQ, analog_ready_callback);
	
	transfer_sample_count = get_sample_count_from_frequency(sample_frequency);
	
	while (1)
	{
		usb_cdc_task();
		uint32_t current_time = clock_get_tick();
		int len;
		
		if (analog_ready_data)
		{
			char buffer[50];
			uint64_t reading_nV;
			sample_count++;
			if (analog_get_reading(&reading_nV))
			{
				reading_nV_sum += reading_nV;
			}
			else
			{
				missed_reading = true;
			}
			if (sample_count >= transfer_sample_count)
			{
				if (missed_reading)
				{
					len = snprintf(buffer, sizeof(buffer) - 1, "Missed a reading\r\n");
				}
				else
				{
					uint64_t reading_nV_avg = reading_nV_sum / transfer_sample_count;
					uint32_t integer_voltage = reading_nV / 1000000000;
					uint32_t fraction_voltage = reading_nV % 1000000000;
					len = snprintf(buffer, sizeof(buffer) - 1, "Voltage: %ld.%09ld\r\n", integer_voltage, fraction_voltage);
				}
				
				usb_cdc_write((uint8_t *)buffer, len);	
				reading_nV_sum = 0;
				sample_count = 0;
				missed_reading = false;
			}

			analog_ready_data = false;
		}
	}
}
