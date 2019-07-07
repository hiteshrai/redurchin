/*temperature.c : Drivers for temperature measurements
 **/

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "custom_board.h"
#include "i2c.h"
#include "temperature.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TEMPERATURE_I2C_ADDRESS_1           0x18
#define TEMPERATURE_I2C_ADDRESS_2           0x1F

#define TEMPERATURE_CONFIG_REGISTER         0x00
#define TEMPERATURE_AMBIENT_REGISTER        0x05

// The temperature is in 1/16th of degree, and the upper 4 bits are flags
#define TEMPERATURE_RESOLUTION_SHIFT_DOWN   4
#define TEMPERATURE_FLAG_BIT_COUNT          4

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

 
/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/
static bool read_temperature(int16_t *temperature, uint8_t slave_address)
{
	bool success = false;
	uint8_t write_register = TEMPERATURE_AMBIENT_REGISTER;
	
	if (i2c_write(slave_address, &write_register, sizeof(uint8_t)))
	{
		if (i2c_read(slave_address, (uint8_t *)temperature, sizeof(uint16_t)))
		{
			success = true;
		}
	}
	
	return success;
}

bool temperature_init(void)
{
	i2c_init();
	
	return true;
}

bool temperature_get_reading(int *temp_c)
{
	int16_t raw_temp;
	bool success = false;
	
	if (read_temperature(&raw_temp, TEMPERATURE_I2C_ADDRESS_2))
	{
    	// Reverse the byte order
    	int16_t endian_fixed_temp = ((raw_temp >> 8) & 0xFF) | (raw_temp << 8);
    	// Shift up to clear the flags
    	endian_fixed_temp = endian_fixed_temp << TEMPERATURE_FLAG_BIT_COUNT;
    	
    	// Now shift it back down both the amount of flag bits
    	//  and the resolution
		*temp_c = endian_fixed_temp >> (TEMPERATURE_FLAG_BIT_COUNT + TEMPERATURE_RESOLUTION_SHIFT_DOWN);
		success = true;
	}
	
    return success;
}
