#ifndef _ANALOG_H_
#define _ANALOG_H_

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef void(*analog_ready_callback_t)(void);

/*******************************************************************************
* API
******************************************************************************/
void analog_init(void);
void analog_start(uint32_t freq, analog_ready_callback_t cb);
void analog_stop(void);
bool analog_get_raw_reading(int64_t *analog);
uint32_t analog_get_raw_to_fV_factor(void);
bool analog_set_gain(float gain);
float analog_get_current_gain(uint8_t *shift_value);

#endif /* _ANALOG_H_ */
