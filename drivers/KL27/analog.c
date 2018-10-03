/*analog.c : Drivers for analog front end
 **/

#include "fsl_spi.h"
#include "spi.h"
#include "pwm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "custom_board.h"
#include "analog.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// PGA commands and registers
#define PGA_WRITE_COMMAND         0x40
#define PGA_READ_COMMAND          0x80

#define PGA_REG_GAIN              0x00
#define PGA_REG_MUX               0x06

#define PGA_REG_MUX_INIT          0x60
#define PGA_REG_GAIN_UNITY        0x18

// IRQ handling
#define ANALOG_READY_IRQ          PORTB_PORTC_PORTD_PORTE_IRQn
#define ANALOG_READY_IRQ_HANDLER  PORTBCDE_IRQHandler

#define SELECT_CHIP(x)            GPIO_PinWrite(ANALOG_SPI_GPIO, x, 0)
#define DESELECT_CHIP(x)          GPIO_PinWrite(ANALOG_SPI_GPIO, x, 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
analog_ready_callback_t analog_ready_callback = NULL;


/*******************************************************************************
 * Code
 ******************************************************************************/
void ANALOG_READY_IRQ_HANDLER(void)
{
	GPIO_PortClearInterruptFlags(ANALOG_GPIO, 1U << ANALOG_READY_PIN);
	if (analog_ready_callback)
	{
		analog_ready_callback();
	}
}


static void setup_rdy_pin(void)
{
	gpio_pin_config_t ready_pin_config = 
	{
		kGPIO_DigitalInput,
		0,
	};
	
	PORT_SetPinInterruptConfig(ANALOG_PORT, ANALOG_READY_PIN, kPORT_InterruptFallingEdge);
	GPIO_PinInit(ANALOG_GPIO, ANALOG_READY_PIN, &ready_pin_config);
	EnableIRQ(ANALOG_READY_IRQ);
}


static void setup_pga(void)
{
	spi_init();
	uint8_t mux_data[] = { PGA_WRITE_COMMAND | PGA_REG_MUX, PGA_REG_MUX_INIT };
	uint8_t gain_data[] = { PGA_WRITE_COMMAND | PGA_REG_GAIN, PGA_REG_GAIN_UNITY };
	
	SELECT_CHIP(ANALOG_SPI_CS_PGA);
	spi_transaction(mux_data, NULL, sizeof(mux_data));
	DESELECT_CHIP(ANALOG_SPI_CS_PGA);
	
	SELECT_CHIP(ANALOG_SPI_CS_PGA);
	spi_transaction(gain_data, NULL, sizeof(gain_data));
	DESELECT_CHIP(ANALOG_SPI_CS_PGA);
	
	spi_deinit();
}

static void analog_pin_init(void)
{
	/* Configure MCLK pin to use as PWM and READY pin as an GPIO */
	CLOCK_EnableClock(ANALOG_CLOCK);
	PORT_SetPinMux(ANALOG_PORT, ANALOG_MCLK_PIN, kPORT_MuxAlt3);
	PORT_SetPinMux(ANALOG_PORT, ANALOG_READY_PIN, kPORT_MuxAsGpio);
	
	/* Configure SPI bus*/
	CLOCK_EnableClock(ANALOG_SPI_CLOCK);
	PORT_SetPinMux(ANALOG_SPI_PORT, ANALOG_SPI_MISO, kPORT_MuxAlt2);
	PORT_SetPinMux(ANALOG_SPI_PORT, ANALOG_SPI_MOSI, kPORT_MuxAlt2);
	PORT_SetPinMux(ANALOG_SPI_PORT, ANALOG_SPI_CLK, kPORT_MuxAlt2);
	
	PORT_SetPinMux(ANALOG_SPI_PORT, ANALOG_SPI_CS_PGA, kPORT_MuxAsGpio);
	PORT_SetPinMux(ANALOG_SPI_PORT, ANALOG_SPI_CS_ADC, kPORT_MuxAsGpio);
	
	gpio_pin_config_t adc_cs_config = 
	{
		kGPIO_DigitalOutput,
		1,
	};
	
	GPIO_PinInit(ANALOG_SPI_GPIO, ANALOG_SPI_CS_ADC, &adc_cs_config);
	
	gpio_pin_config_t pga_cs_config = 
	{
		kGPIO_DigitalOutput,
		1,
	};
	
	GPIO_PinInit(ANALOG_SPI_GPIO, ANALOG_SPI_CS_PGA, &pga_cs_config);
	
}

void analog_init(void)
{
	analog_pin_init();
	setup_pga();
}

void analog_start(uint32_t freq, analog_ready_callback_t cb)
{
	// Setup Ready pin for 2512-24 ADC Chip.
    setup_rdy_pin();
	
	// Initialize MCLK pin for 2512-24 ADC chip.
	pwm_init(freq);	
	
	spi_init();
	
	analog_ready_callback = cb;
}

void analog_stop(void)
{
	pwm_deinit();
	analog_ready_callback = NULL;
}

bool analog_get_reading(uint64_t *analog)
{
	enum
	{
		ADC_CONFIG_WORD_DF32 = 0x56,
		ANALOG_nV_PER_LSB    = 596
	};
	bool success = false;
	uint8_t read_data[4];
	
	SELECT_CHIP(ANALOG_SPI_CS_ADC);
	spi_transaction(NULL, read_data, sizeof(read_data));
	DESELECT_CHIP(ANALOG_SPI_CS_ADC);
	
	// Separate the voltage value and the "configuration" word
	uint64_t analog_value_int = read_data[0] << 16
		| read_data[1] << 8
		| read_data[2];
	uint8_t config_word = read_data[3];
	
	if (config_word == ADC_CONFIG_WORD_DF32)
	{
		// This is a valid reading
		*analog = analog_value_int * ANALOG_nV_PER_LSB;
		success = true;
	}
	return success;
}
