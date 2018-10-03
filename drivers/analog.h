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
bool analog_get_reading(uint64_t *analog);

#endif /* _ANALOG_H_ */
