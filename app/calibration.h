/******************************************************************************
 calibration.h
 Interface for calibration routines
 ******************************************************************************/

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
enum
{
    NUM_CALIB_POINTS_PER_GAIN = 5,
};

typedef struct
{
    float gain;
    struct
    {
        int32_t input_voltage;
        int32_t error;
    } cal_points[NUM_CALIB_POINTS_PER_GAIN];
    uint32_t signature;
} gain_calibration_t;

/*******************************************************************************
* API
******************************************************************************/
bool calibration_init(void);
bool calibration_adjust_voltage_reading(float gain, int64_t input_reading, int64_t *output_reading);
bool calibration_write_gain(gain_calibration_t calibration);
bool calibration_read_gain(int index, gain_calibration_t *calibration);
int calibration_get_num_calibrations(void);

#endif /* _CALIBRATION_H_ */
