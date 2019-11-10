#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "temperature.h"
#include "eeprom.h"
#include "calibration.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define CALIBRATION_VERSION    0
#define CALIBRATION_SIGNATURE  0x52454455  // REDU
#define CALIBRATION_ROW_SIGNATURE 0x47524149 // HRAI

enum
{
    CALIBRATION_HEADER_START_ADDR = 0x0000,
    CALIBRATION_HEADER_LENGTH     = 0x0010,
    
    CALIBRATION_MISC_START_ADDR   = CALIBRATION_HEADER_START_ADDR + CALIBRATION_HEADER_LENGTH,
    CALIBRATION_MISC_LENGTH       = 0x0010,

    CALIBRATION_TEMP_START_ADDR   = CALIBRATION_MISC_START_ADDR + CALIBRATION_MISC_LENGTH,
    CALIBRATION_TEMP_LENGTH       = 0x0010,

    CALIBRATION_RESV_START_ADDR   = CALIBRATION_TEMP_START_ADDR + CALIBRATION_TEMP_LENGTH,
    CALIBRATION_RESV_LENGTH       = 0x0050,

    CALIBRATION_VOLT_START_ADDR   = CALIBRATION_RESV_START_ADDR + CALIBRATION_RESV_LENGTH,
    CALIBRATION_VOLT_LENGTH       = 0x0180,

    CALIBRATION_END_ADDR          = CALIBRATION_VOLT_START_ADDR + CALIBRATION_VOLT_LENGTH,
};

typedef struct
{
    uint32_t calibration_version;
    uint32_t calibration_signature;
    uint32_t reserved1;
    uint32_t reserved2;
} calibration_header_t;

#define MAX_CALIB_GAINS      (CALIBRATION_VOLT_LENGTH / sizeof(gain_calibration_t))

/*******************************************************************************
* Variables
******************************************************************************/
static gain_calibration_t gain_calibration[MAX_CALIB_GAINS];
static uint8_t gain_count = 0;

/*******************************************************************************
* Prototypes
******************************************************************************/


/*******************************************************************************
* Code
******************************************************************************/
bool calibration_init(void)
{
    eeprom_init();

    // Validate calibration data format
    calibration_header_t header;
    if (!eeprom_read_data(CALIBRATION_HEADER_START_ADDR,
        (uint8_t *)&header,
        sizeof(calibration_header_t)))
    {
        // Unable to read from EEPROM
        return false;
    }
    
    // Able to read the header - check it out
    if (header.calibration_version != CALIBRATION_VERSION ||
        header.calibration_signature != CALIBRATION_SIGNATURE)
    {
        // Invalid format - re-format
        eeprom_erase();
        
        memset(&header, 0, sizeof(header));
        header.calibration_version = CALIBRATION_VERSION;
        header.calibration_signature = CALIBRATION_SIGNATURE;
        if (!eeprom_write_data(CALIBRATION_HEADER_START_ADDR,
            (uint8_t *)&header,
            sizeof(calibration_header_t)))
        {
            // Why can't we write to EEPROM?
            return false;
        }
    }
    
    // Now look for calibration data
    uint32_t gain_calib_address = CALIBRATION_VOLT_START_ADDR;
    while (gain_calib_address < CALIBRATION_END_ADDR - sizeof(gain_calibration_t))
    {
        gain_calibration_t calibration = { 0 };
        if (!eeprom_read_data(gain_calib_address,
            (uint8_t *)&calibration,
            sizeof(gain_calibration_t)))
        {
            return false;
        }
        
        // Check validity of the row
        if(calibration.signature == CALIBRATION_ROW_SIGNATURE)
        {
            // Valid calibration - copy it
            gain_calibration[gain_count] = calibration;
            gain_count++;
        }
        else
        {
            // Invalid - get out
            break;
        }
    }    
    return gain_count > 0;
}

bool calibration_adjust_voltage_reading(float gain, int64_t input_reading, int64_t *output_reading)
{
    
}
