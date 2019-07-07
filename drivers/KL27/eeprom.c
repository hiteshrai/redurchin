/*eeprom.c : Drivers for EEPROM
 **/

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "custom_board.h"
#include "i2c.h"
#include "eeprom.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CALIBRATION_DATA_LENGTH    10
#define CALIBRATION_DATA_ADDRESS   0
#define EEPROM_I2C_ADDRESS         0x50

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

 
/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t calibration_data[CALIBRATION_DATA_LENGTH];

/*******************************************************************************
 * Code
 ******************************************************************************/
bool eeprom_read_calibration_data(void)
{
	bool result;
	bool success = false;
    
	// Setup the starting address to read from
	uint8_t reg_address = CALIBRATION_DATA_ADDRESS;
	result = i2c_write(EEPROM_I2C_ADDRESS, &reg_address, 1);
	if (result)
	{
		result = i2c_read(EEPROM_I2C_ADDRESS, calibration_data, CALIBRATION_DATA_LENGTH);
		success = result;
	}
}

bool eeprom_write_calibration_data(void)
{
	uint8_t calibration_address_and_data[sizeof(uint8_t) + CALIBRATION_DATA_LENGTH];
	calibration_address_and_data[0] = CALIBRATION_DATA_ADDRESS;
	memcpy(calibration_address_and_data + sizeof(uint8_t), calibration_data, CALIBRATION_DATA_LENGTH);
	return i2c_write(EEPROM_I2C_ADDRESS,
		calibration_address_and_data,
		sizeof(calibration_address_and_data));
}

bool eeprom_init(void)
{
	i2c_init();
	eeprom_read_calibration_data();
	calibration_data[0]++;
	eeprom_write_calibration_data();
	return true;
}
