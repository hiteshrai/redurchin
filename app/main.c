#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "usb_cdc.h"
#include "hardware.h"
#include "clock.h"
#include "analog.h"
#include "ui.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define MCLK_FREQ              1000000U
#define HARDWARE_DOWNSAMPLING  256
#define OUTPUT_BUFFER_LEN      200

#define SEND_TIMESTAMP         0x01
#define SEND_GAIN              0x02
#define SEND_RATE              0X04

#define MAX_USER_CMD_LENGTH    50

/*******************************************************************************
* Variables
******************************************************************************/
static volatile bool analog_ready_data = false;
static uint32_t sample_frequency = 1;
static volatile uint32_t transfer_sample_count;
static char output_buffer[OUTPUT_BUFFER_LEN];
static int send_mask = SEND_TIMESTAMP | SEND_GAIN | SEND_RATE;

static int64_t reading_raw_sum = 0;
static bool missed_reading = false;
static uint32_t sample_count = 0;

static char user_cmd[MAX_USER_CMD_LENGTH + 1];
static int user_cmd_length = 0;
static bool user_cmd_received = false;

static bool dark_mode = false;

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

static void send_reading_output(int64_t reading_nV, int rate_hz, bool missed_reading)
{
    static bool missed_send = false;
    uint8_t num_shift;
    float analog_gain = analog_get_current_gain(&num_shift);
    
    // Adjust the reading voltage based of the gain
    if(analog_gain < 1.0f)
    {
        reading_nV <<= num_shift;
    }
    else
    {
        reading_nV >>= num_shift;
    }
    
    int32_t integer_voltage = reading_nV / 1000000000;
    uint32_t fraction_voltage = abs((int32_t)(reading_nV % 1000000000));

    // Data is formatted as JSON
    int output_len = 0;
    output_len += sprintf(&output_buffer[output_len], "{");
    output_len += sprintf(&output_buffer[output_len], "\"id\":\"redurchin\"");
    output_len += sprintf(&output_buffer[output_len], ",\"voltage\":%ld.%09ld", integer_voltage, fraction_voltage);
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
            ",\"timestamp_ms\":%ld",
            clock_get_time_ms());
    }
    if (send_mask & SEND_GAIN)
    {
        uint32_t int_gain = analog_gain * 1000;
        output_len += sprintf(&output_buffer[output_len], ",\"gain\":%ld.%ld", int_gain / 1000, int_gain % 1000);
    }
    if (send_mask & SEND_RATE)
    {
        output_len += sprintf(&output_buffer[output_len], ",\"rate_hz\":%d", rate_hz);
    }
    
    output_len += sprintf(&output_buffer[output_len], "}\r\n");

    missed_send = !usb_cdc_write((uint8_t *)output_buffer, output_len);
}

static void usb_data_callback(uint8_t *data, uint32_t length)
{
    uint8_t *end_data = data + length;
    while (data < end_data)
    {
        if (!user_cmd_received)
        {
            if (('\n' == *data) || ('\r' == *data))
            {
                // Null-terminate the command
                user_cmd[user_cmd_length] = '\0';
                user_cmd_length = 0;
                user_cmd_received = true;
                break;
            }
            else if (user_cmd_length < MAX_USER_CMD_LENGTH)
            {
                user_cmd[user_cmd_length++] = *data;
            }
            else
            {
                // Too long - start over
                user_cmd_length = 0;
            }
        }
        data++;
    }
}

void set_gain(float gain)
{
    analog_set_gain(gain);
}

void set_rate(int rate)
{
    sample_frequency = rate;
    transfer_sample_count = get_sample_count_from_frequency(sample_frequency);
    
    // Clear out variables so we get a clean sum
    reading_raw_sum = 0;
    sample_count = 0;
    missed_reading = false;
}

void set_dark_mode(void)
{
    dark_mode = true;
}

bool handle_user_command(void)
{
    bool command_handled = false;
    if (user_cmd_received)
    {
        char *cmd_data = NULL;
        char *colon_location = strchr(user_cmd, ':');
        if (colon_location)
        {
            // There is a data portion
            cmd_data = colon_location + 1;
            *colon_location = '\0';
        }
        if ((0 == strcmp("GAIN", user_cmd)) && cmd_data)
        {
            set_gain(atof(cmd_data));
            command_handled = true;
        }
        else if ((0 == strcmp("RATE", user_cmd)) && cmd_data)
        {
            set_rate(atoi(cmd_data));
            command_handled = true;
        }
        else if (0 == strcmp("DARK", user_cmd))
        {
            set_dark_mode();
            command_handled = true;
        }
        user_cmd_received = false;
    }
    return command_handled;
}

int main(void)
{
	hardware_init();
	usb_cdc_init();
    usb_cdc_set_receive_callback(usb_data_callback);
	clock_init();
	ui_init();
	
	analog_init();		
	analog_start(MCLK_FREQ, analog_ready_callback);
	
	transfer_sample_count = get_sample_count_from_frequency(sample_frequency);
	
	uint32_t last_led_change_tick = clock_get_tick();
	uint8_t last_led_state = 1;
	
	while (1)
	{
		usb_cdc_task();
		uint32_t current_time = clock_get_tick();
		int len;
		
		if (analog_ready_data)
		{
			char buffer[50];
			int64_t reading_raw;
			sample_count++;
    		if (analog_get_raw_reading(&reading_raw))
			{
				reading_raw_sum += reading_raw;
			}
			else
			{
				missed_reading = true;
			}
			if (sample_count >= transfer_sample_count)
			{
    			int64_t reading_raw_avg = reading_raw_sum / transfer_sample_count;
    			send_reading_output((reading_raw_avg / 1000000) * analog_get_raw_to_fV_factor(),
        			sample_frequency,
        			missed_reading);
				reading_raw_sum = 0;
				sample_count = 0;
				missed_reading = false;
			}

			analog_ready_data = false;
		}
    	
    	handle_user_command();
		
       	if (clock_get_elapsed_time_ms(clock_get_tick(), last_led_change_tick) >= 10)
        {
            if (dark_mode)
            {
                ui_darken();
            }
            else
            {
                ui_brighten();                
            }
            last_led_change_tick = clock_get_tick();
        }        	
	}
}
