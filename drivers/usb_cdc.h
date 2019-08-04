/*
 **/
#ifndef _USB_CDC_H_
#define _USB_CDC_H_

#include <stdint.h>
#include <stdbool.h>
/*******************************************************************************
 * DEFINITION
 ******************************************************************************/
typedef void(*usb_cdc_receive)(uint8_t *data, uint32_t length);

/*******************************************************************************
 * API
 ******************************************************************************/
void usb_cdc_init(void);
void usb_cdc_task(void);
bool usb_cdc_write(uint8_t * data, uint32_t length);
void usb_cdc_set_receive_callback(usb_cdc_receive rx);
bool usb_cdc_is_connected(void);

#endif /* USB_CDC_H_ */
