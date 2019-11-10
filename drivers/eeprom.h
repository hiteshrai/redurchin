#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

 
/*******************************************************************************
* API
******************************************************************************/
bool eeprom_read_data(uint32_t address, uint8_t *data, uint32_t length);
bool eeprom_write_data(uint32_t address, uint8_t *data, uint32_t length);
bool eeprom_init(void);
bool eeprom_erase(void);

#endif /* _EEPROM_H_ */
