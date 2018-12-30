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
#define OUTPUT_BUFFER_LEN      200

#define SEND_TIMESTAMP  0x01
#define SEND_GAIN       0x02
#define SEND_RATE       0X04

#define ANALOG_GAIN     1

/*******************************************************************************
* Variables
******************************************************************************/
static volatile bool analog_ready_data = false;
static uint32_t sample_frequency = 1;
static uint32_t transfer_sample_count;
static char output_buffer[OUTPUT_BUFFER_LEN];
static int send_mask = SEND_TIMESTAMP;

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

static void send_reading_output(uint64_t reading_nV, int rate_hz, float gain, bool missed_reading)
{
    static bool missed_send = false;
    uint32_t integer_voltage = reading_nV / 1000000000;
    uint32_t fraction_voltage = reading_nV % 1000000000;

    // Data is formatted as JSON
    int output_len = 0;
    output_len += sprintf(&output_buffer[output_len], "{");
    output_len += sprintf(&output_buffer[output_len], "id:redurchin");
    output_len += sprintf(&output_buffer[output_len], ",voltage:%ld.%09ld", integer_voltage, fraction_voltage);
    if (missed_reading)
    {
        output_len += sprintf(&output_buffer[output_len], ",missed_reading:1");
    }
    if (missed_send)
    {
        output_len += sprintf(&output_buffer[output_len], ",missed_send:1");
    }
    if (send_mask & SEND_TIMESTAMP)
    {
        output_len += sprintf(&output_buffer[output_len],
            ",timestamp_ms:%ld",
            clock_get_time_ms());
    }
    if (send_mask & SEND_GAIN)
    {
        output_len += sprintf(&output_buffer[output_len], ",gain:%f", gain);
    }
    if (send_mask & SEND_RATE)
    {
        output_len += sprintf(&output_buffer[output_len], ",rate_hz:%d", rate_hz);
    }
    
    output_len += sprintf(&output_buffer[output_len], "}\r\n");

    missed_send = !usb_cdc_write((uint8_t *)output_buffer, output_len);
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
    			uint64_t reading_nV_avg = reading_nV_sum / transfer_sample_count;
    			send_reading_output(reading_nV_avg, sample_frequency, ANALOG_GAIN, missed_reading);
				reading_nV_sum = 0;
				sample_count = 0;
				missed_reading = false;
			}

			analog_ready_data = false;
		}
	}
}
