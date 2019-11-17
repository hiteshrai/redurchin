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

bool calibration_write_gain(gain_calibration_t calibration)
{
    bool success = false;
    int gain_index;
    
    // Is this an update to an existing gain, or a new gain?
    for(gain_index = 0 ; gain_index < gain_count ; gain_index++)
    {
        if (calibration.gain == gain_calibration[gain_index].gain)
        {
            // Match
            break;
        }
    }
    
    if (gain_index < MAX_CALIB_GAINS)
    {
        // Write to EEPROM and then update cache
        if(eeprom_write_data(CALIBRATION_VOLT_START_ADDR + gain_index * sizeof(gain_calibration_t),
            (uint8_t*)&calibration,
            sizeof(gain_calibration_t)))
        {
            gain_calibration[gain_index] = calibration;
            success = true;
            if (gain_index + 1 > gain_count)
            {
                gain_count = gain_index + 1;
            }
        }
    }
    return success;
}

bool calibration_read_gain(int index, gain_calibration_t *calibration)
{
    bool valid = index < gain_count;
    if (valid)
    {
        *calibration = gain_calibration[index];
    }
    return valid;
}

int calibration_get_num_calibrations(void)
{
    return gain_count;
}

int64_t interpolate_voltage_adjustment(int64_t input_reading, int64_t low_voltage, int64_t high_voltage, int64_t low_error, int64_t high_error)
{
    if (low_voltage == high_voltage)
    {
        // This is a step function. Average the errors and add it.
        return input_reading + (low_error + high_error) / 2;
    }
    
    int64_t adjustment = (input_reading - low_voltage) * (high_error - low_error) / (high_voltage - low_voltage);
    
    return input_reading + adjustment;
}

bool calibration_adjust_voltage_reading(float gain, int64_t input_reading, int64_t *output_reading)
{
    // Find the matching gain
    int gain_index;
    for (gain_index = 0; gain_index < gain_count; gain_index++)
    {
        if (gain_calibration[gain_index].gain == gain)
        {
            break;
        }
    }
    if (gain_index < gain_count)
    {
        // Found a matching table - interpolate/extrapolate to convert input to output
        gain_calibration_t *cal = &gain_calibration[gain_index];
        int section_index;
        for (section_index = 0; section_index < NUM_CALIB_POINTS_PER_GAIN; section_index++)
        {
            if (input_reading <= cal->cal_points[section_index].input_voltage)
            {
                break;
            }
        }
        if (section_index == 0)
        {
            // Extrapolating down
            *output_reading = interpolate_voltage_adjustment(input_reading, 
                cal->cal_points[0].input_voltage,
                cal->cal_points[1].input_voltage,
                cal->cal_points[0].error,
                cal->cal_points[1].error);

        }
        else if (section_index == NUM_CALIB_POINTS_PER_GAIN)
        {
            // Extrapolating up
            section_index--;
            *output_reading = interpolate_voltage_adjustment(input_reading, 
                cal->cal_points[section_index - 1].input_voltage,
                cal->cal_points[section_index].input_voltage,
                cal->cal_points[section_index - 1].error,
                cal->cal_points[section_index].error);

        }
        else
        {
            *output_reading = interpolate_voltage_adjustment(input_reading, 
                cal->cal_points[section_index - 1].input_voltage,
                cal->cal_points[section_index].input_voltage,
                cal->cal_points[section_index - 1].error,
                cal->cal_points[section_index].error);
        }
        return true;
    }
    else
    {
        *output_reading = input_reading;
        return false;
    }
}
