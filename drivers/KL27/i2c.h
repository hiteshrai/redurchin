#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
* API
******************************************************************************/
void i2c_init(void);
bool i2c_read(uint8_t i2c_address, uint8_t *data, uint8_t length);
bool i2c_write(uint8_t i2c_address, uint8_t *data, uint8_t length);

#endif /* _I2C_H_ */
