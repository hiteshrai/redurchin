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
#define MAX_EEPROM_DATA_LENGTH          50
#define EEPROM_I2C_BASE_ADDRESS         0x50
#define TOTAL_EEPROM_SIZE               512

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

 
/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/
bool eeprom_read_data(uint32_t address, uint8_t *data, uint32_t length)
{
	bool result;
	bool success = false;
    
	// Setup the starting address to read from
	uint8_t reg_address = address & 0xFF;
	result = i2c_write(EEPROM_I2C_BASE_ADDRESS, &reg_address, sizeof(reg_address));
	if (result)
	{
    	result = i2c_read(EEPROM_I2C_BASE_ADDRESS, data, length);
		success = result;
	}
    return success;
}

bool eeprom_write_data(uint32_t address, uint8_t *data, uint32_t length)
{
    uint8_t calibration_address_and_data[sizeof(uint8_t) + MAX_EEPROM_DATA_LENGTH];
    
    if (length > MAX_EEPROM_DATA_LENGTH)
    {
        return false;
    }
	calibration_address_and_data[0] = address & 0xFF;
	memcpy(calibration_address_and_data + sizeof(uint8_t), data, length);
	return i2c_write(EEPROM_I2C_BASE_ADDRESS,
		calibration_address_and_data,
		sizeof(calibration_address_and_data));
}

bool eeprom_erase(void)
{
    enum
    {
        ERASE_CHUNK_BYTES = 0x10,
    };
    uint8_t erase_bytes[ERASE_CHUNK_BYTES] = { 0 };
    for (int erase_addr = 0; erase_addr < TOTAL_EEPROM_SIZE; erase_addr += ERASE_CHUNK_BYTES)
    {
        eeprom_write_data(erase_addr, erase_bytes, ERASE_CHUNK_BYTES);
    }
}

bool eeprom_init(void)
{
	i2c_init();
	return true;
}
