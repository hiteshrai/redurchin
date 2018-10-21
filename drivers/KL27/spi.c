#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "clock_config.h"

#include "fsl_spi.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPI_MASTER_BASEADDR                SPI0
#define SPI_MASTER_SOURCE_CLOCK            kCLOCK_BusClk
#define SPI_MASTER_CLK_FREQ                CLOCK_GetFreq(kCLOCK_BusClk)

#define TRANSFER_BAUDRATE                  4000000U 

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
bool spi_transaction(uint8_t * write_data, uint8_t * read_data, uint8_t data_size)
{
	spi_transfer_t master_xfer = { 0 };
	
	/* Start master transfer */
	master_xfer.txData = write_data;
	master_xfer.rxData = read_data;
	master_xfer.dataSize = data_size;

	return (kStatus_Success == SPI_MasterTransferBlocking(SPI_MASTER_BASEADDR, &master_xfer));
}

void spi_init(void)
{
	spi_master_config_t spi_config = { 0 };
	uint32_t spi_freq = 0;

	SPI_MasterGetDefaultConfig(&spi_config);
	spi_config.baudRate_Bps = TRANSFER_BAUDRATE;
	spi_config.polarity = kSPI_ClockPolarityActiveLow;
	spi_config.phase = kSPI_ClockPhaseFirstEdge;
	
	spi_freq = SPI_MASTER_CLK_FREQ;

	SPI_MasterInit(SPI_MASTER_BASEADDR, &spi_config, spi_freq);
}

void spi_deinit(void)
{
	SPI_Deinit(SPI_MASTER_BASEADDR);
}
