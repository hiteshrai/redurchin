/*i2c.c : Drivers for I2C
 **/

#include "fsl_i2c.h"
#include "spi.h"
#include "pwm.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "custom_board.h"
#include "analog.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_BAUDRATE 100000U
#define I2C_BASEADDR I2C0
#define I2C_CLOCK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

 
/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/
static void init_i2c_pins(void)
{
	/* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(I2C_CLOCK);

    const port_pin_config_t scl_config = {/* Internal pull-up resistor is enabled */
                                                   kPORT_PullUp,
                                                   /* Fast slew rate is configured */
                                                   kPORT_FastSlewRate,
                                                   /* Passive filter is disabled */
                                                   kPORT_PassiveFilterDisable,
                                                   /* Low drive strength is configured */
                                                   kPORT_LowDriveStrength,
                                                   /* Pin is configured as I2C0_SDA */
                                                   kPORT_MuxAlt2};
    /* PORTB0 is configured as I2C0_SCL */
    PORT_SetPinConfig(I2C_PORT, I2C_SCL_PIN, &scl_config);

    const port_pin_config_t sda_config = {/* Internal pull-up resistor is enabled */
                                                   kPORT_PullUp,
                                                   /* Fast slew rate is configured */
                                                   kPORT_FastSlewRate,
                                                   /* Passive filter is disabled */
                                                   kPORT_PassiveFilterDisable,
                                                   /* Low drive strength is configured */
                                                   kPORT_LowDriveStrength,
                                                   /* Pin is configured as I2C0_SCL */
                                                   kPORT_MuxAlt2};
    /* PORTB1 is configured as I2C0_SDA */
    PORT_SetPinConfig(I2C_PORT, I2C_SDA_PIN, &sda_config);
    
    /* PORTB0 is configured as I2C0_SCL */
    PORT_SetPinMux(I2C_PORT, I2C_SCL_PIN, kPORT_MuxAlt2);

    /* PORTB1 is configured as I2C0_SDA */
    PORT_SetPinMux(I2C_PORT, I2C_SDA_PIN, kPORT_MuxAlt2);
}

void i2c_init(void)
{
	init_i2c_pins();

    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(I2C_BASEADDR, &masterConfig, I2C_CLOCK_FREQ);
}

bool i2c_read(uint8_t i2c_address, uint8_t *data, uint8_t length)
{
    i2c_master_transfer_t masterXfer;

    /* Prepare transfer structure. */
    masterXfer.slaveAddress = i2c_address;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = data;
    masterXfer.dataSize = length;
    masterXfer.direction = kI2C_Read;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status_t result = I2C_MasterTransferBlocking(I2C_BASEADDR, &masterXfer);
    return result == kStatus_Success;
}

bool i2c_write(uint8_t i2c_address, uint8_t *data, uint8_t length)
{
    i2c_master_transfer_t masterXfer;

    /* Prepare transfer structure. */
    masterXfer.slaveAddress = i2c_address;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = data;
    masterXfer.dataSize = length;
    masterXfer.direction = kI2C_Write;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status_t result = I2C_MasterTransferBlocking(I2C_BASEADDR, &masterXfer);
    return result == kStatus_Success;
}
