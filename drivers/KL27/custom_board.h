/* custom_board.h
*/

#ifndef _CUSTOM_BOARD_H_
#define _CUSTOM_BOARD_H_

#include "fsl_device_registers.h"
#include "fsl_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define UART_CLOCK              kCLOCK_PortB
#define UART_PORT               PORTB
#define UART_RX_PIN             16
#define UART_TX_PIN             17

#define ANALOG_SPI_CLOCK        kCLOCK_PortD
#define ANALOG_SPI_PORT         PORTD
#define ANALOG_SPI_GPIO         GPIOD
#define ANALOG_SPI_MISO         7
#define ANALOG_SPI_MOSI         6
#define ANALOG_SPI_CLK          5
#define ANALOG_SPI_CS_ADC       4

#define ANALOG_CLOCK            kCLOCK_PortC
#define ANALOG_PORT             PORTC
#define ANALOG_GPIO             GPIOC
#define ANALOG_MCLK_PIN         4
#define ANALOG_READY_PIN        7
#define ANALOG_ADC_SEL0_PIN     3
#define ANALOG_ADC_SEL1_PIN     2
#define ANALOG_PGA_CS_PIN       1

#define UI_LED_CLOCK            kCLOCK_PortA
#define UI_LED_PORT             PORTA
#define UI_LED_GPIO             GPIOA
#define UI_LED_PIN              1

#endif 
