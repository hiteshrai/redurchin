#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

 
/*******************************************************************************
* API
******************************************************************************/
bool temperature_init(void);
bool temperature_get_reading(int *temp_c);

#endif /* _TEMPERATURE_H_ */
