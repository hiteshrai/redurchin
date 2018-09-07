/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_debug_console.h"
#include "fsl_spi.h"
#include "fsl_flexio_spi.h"
#include "board.h"

#include "clock_config.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_FLEXIO_BASE FLEXIO
#define FLEXIO_SPI_MOSI_PIN 6U
#define FLEXIO_SPI_MISO_PIN 7U
#define FLEXIO_SPI_SCK_PIN 5U
#define FLEXIO_SPI_PCS0_PIN 4U

#define BOARD_SPI_MASTER_BASE SPI0
#define SPI_MASTER_CLK_SRC SPI0_CLK_SRC
#define SPI_MASTER_CLK_FREQ CLOCK_GetFreq(SPI0_CLK_SRC)
#define SLAVE_SPI_IRQ UART2_FLEXIO_IRQn
#define MASTER_SPI_IRQ SPI0_IRQn
#define TRANSFER_SIZE 256U        /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 500000U /*! Transfer baudrate - 500k */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* SPI user callback */
void SPI_MasterUserCallback(SPI_Type *base, spi_master_handle_t *handle, status_t status, void *userData);
void FLEXIO_SPI_SlaveUserCallback(FLEXIO_SPI_Type *base,
                                  flexio_spi_slave_handle_t *handle,
                                  status_t status,
                                  void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t masterRxData[TRANSFER_SIZE] = {0U};
uint8_t masterTxData[TRANSFER_SIZE] = {0U};
uint8_t slaveRxData[TRANSFER_SIZE] = {0U};
uint8_t slaveTxData[TRANSFER_SIZE] = {0U};

spi_master_handle_t g_m_handle;
FLEXIO_SPI_Type spiDev;
flexio_spi_slave_handle_t g_s_handle;

volatile bool isTransferCompleted = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SPI_MasterUserCallback(SPI_Type *base, spi_master_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        __NOP();
    }
}

void FLEXIO_SPI_SlaveUserCallback(FLEXIO_SPI_Type *base,
                                  flexio_spi_slave_handle_t *handle,
                                  status_t status,
                                  void *userData)
{
    if (status == kStatus_Success)
    {
        __NOP();
    }

    isTransferCompleted = true;

    PRINTF("This is FLEXIO SPI slave call back.\r\n");
}

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    CLOCK_SetFlexio0Clock(1U);

    PRINTF("SPI Master interrupt - FLEXIO SPI Slave interrupt example start.\r\n");
    PRINTF("This example use one spi instance as master and one flexio spi slave on one board.\r\n");
    PRINTF("Master and slave are both use interrupt way.\r\n");
    PRINTF("Please make sure you make the correct line connection. Basically, the connection is: \r\n");
    PRINTF("SPI_master -- FLEXIO_SPI_slave \r\n");
    PRINTF("   SCK      --    SCK  \r\n");
    PRINTF("   PCS0     --    PCS0 \r\n");
    PRINTF("   MOSI     --    MOSI \r\n");
    PRINTF("   MISO     --    MISO \r\n");

    uint32_t srcClock_Hz;
    uint32_t errorCount;
    uint32_t i;
    spi_master_config_t masterConfig;
    flexio_spi_slave_config_t slaveConfig;
    spi_transfer_t masterXfer;
    flexio_spi_transfer_t slaveXfer;

    /* Master config */
    /*
     * masterConfig.enableStopInWaitMode = false;
     * masterConfig.polarity = kSPI_ClockPolarityActiveHigh;
     * masterConfig.phase = kSPI_ClockPhaseFirstEdge;
     * masterConfig.direction = kSPI_MsbFirst;
     * masterConfig.dataMode = kSPI_8BitMode;
     * masterConfig.txWatermark = kSPI_TxFifoOneHalfEmpty;
     * masterConfig.rxWatermark = kSPI_RxFifoOneHalfFull;
     * masterConfig.pinMode = kSPI_PinModeNormal;
     * masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput;
     * masterConfig.baudRate_Bps = 500000U;
     */
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = TRANSFER_BAUDRATE;

    srcClock_Hz = SPI_MASTER_CLK_FREQ;
    SPI_MasterInit(BOARD_SPI_MASTER_BASE, &masterConfig, srcClock_Hz);

    /* Slave config */
    /*
     * slaveConfig.enableSlave = true;
     * slaveConfig.enableInDoze = false;
     * slaveConfig.enableInDebug = true;
     * slaveConfig.enableFastAccess = false;
     * slaveConfig.phase = kFLEXIO_SPI_ClockPhaseFirstEdge;
     * slaveConfig.dataMode = kFLEXIO_SPI_8BitMode;
     */
    FLEXIO_SPI_SlaveGetDefaultConfig(&slaveConfig);

    spiDev.flexioBase = BOARD_FLEXIO_BASE;
    spiDev.SDOPinIndex = FLEXIO_SPI_MISO_PIN;
    spiDev.SDIPinIndex = FLEXIO_SPI_MOSI_PIN;
    spiDev.SCKPinIndex = FLEXIO_SPI_SCK_PIN;
    spiDev.CSnPinIndex = FLEXIO_SPI_PCS0_PIN;
    spiDev.shifterIndex[0] = 0U;
    spiDev.shifterIndex[1] = 1U;
    spiDev.timerIndex[0] = 0U;
    FLEXIO_SPI_SlaveInit(&spiDev, &slaveConfig);

    /* Set spi slave interrupt priority higher. */
    NVIC_SetPriority(MASTER_SPI_IRQ, 1U);
    NVIC_SetPriority(SLAVE_SPI_IRQ, 0U);

    /* Set up the transfer data */
    for (i = 0U; i < TRANSFER_SIZE; i++)
    {
        masterTxData[i] = i % 256U;
        masterRxData[i] = 0U;

        slaveTxData[i] = ~masterTxData[i];
        slaveRxData[i] = 0U;
    }

    isTransferCompleted = false;

    /* Set up slave first */
    FLEXIO_SPI_SlaveTransferCreateHandle(&spiDev, &g_s_handle, FLEXIO_SPI_SlaveUserCallback, NULL);

    /*Set slave transfer ready to receive/send data*/
    slaveXfer.txData = slaveTxData;
    slaveXfer.rxData = slaveRxData;
    slaveXfer.dataSize = TRANSFER_SIZE;
    slaveXfer.flags = kFLEXIO_SPI_8bitMsb;

    FLEXIO_SPI_SlaveTransferNonBlocking(&spiDev, &g_s_handle, &slaveXfer);

    /* Set up master transfer */
    SPI_MasterTransferCreateHandle(BOARD_SPI_MASTER_BASE, &g_m_handle, SPI_MasterUserCallback, NULL);

    /*Start master transfer*/
    masterXfer.txData = masterTxData;
    masterXfer.rxData = masterRxData;
    masterXfer.dataSize = TRANSFER_SIZE;

    SPI_MasterTransferNonBlocking(BOARD_SPI_MASTER_BASE, &g_m_handle, &masterXfer);

    /* Wait slave received all data. */
    while (!isTransferCompleted)
    {
    }

    errorCount = 0U;
    for (i = 0U; i < TRANSFER_SIZE; i++)
    {
        if (masterTxData[i] != slaveRxData[i])
        {
            errorCount++;
        }

        if (slaveTxData[i] != masterRxData[i])
        {
            errorCount++;
        }
    }
    if (errorCount == 0U)
    {
        PRINTF("SPI master <-> FLEXIO SPI slave transfer all data matched!\r\n");
    }
    else
    {
        PRINTF("Error occured in SPI master <-> FLEXIO SPI slave transfer!\r\n");
    }

    SPI_Deinit(BOARD_SPI_MASTER_BASE);
    FLEXIO_SPI_SlaveDeinit(&spiDev);

    while (1)
    {
    }
}
