/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"

#include <stdio.h>
#include <stdlib.h>

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "fsl_debug_console.h"

#include "usb_device_descriptor.h"
#include "virtual_com.h"

void BOARD_InitHardware(void);
#include "usb_cdc.h"
/*******************************************************************************
* Definitions
******************************************************************************/

/*******************************************************************************
* Variables
******************************************************************************/
/* Data structure of virtual com device */
static usb_cdc_vcom_struct_t s_cdcVcom;

/* Line codinig of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_lineCoding[LINE_CODING_SIZE] = {
	/* E.g. 0x00,0xC2,0x01,0x00 : 0x0001C200 is 115200 bits per second */
	(LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
	(LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
	(LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
	(LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
	LINE_CODING_CHARFORMAT,
	LINE_CODING_PARITYTYPE,
	LINE_CODING_DATABITS 
};

/* Abstract state of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {
	(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU,
	(STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU 
};

/* Country code of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {
	(COUNTRY_SETTING >> 0U) & 0x00FFU,
	(COUNTRY_SETTING >> 8U) & 0x00FFU 
};

/* CDC ACM information */
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static usb_cdc_acm_info_t s_usbCdcAcmInfo;
/* Data buffer for receiving and sending*/
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currSendBuf[DATA_BUFF_SIZE];
volatile static uint32_t s_recvSize = 0;
volatile static uint32_t s_sendSize = 0;
static uint32_t s_usbBulkMaxPacketSize = FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;


// Application specific variables
#define TRANSMIT_BUFFER_SIZE                             1024
#define RECEIVE_BUFFER_SIZE                              1024
#define TRANSMIT_BUFFER_MASK                             (TRANSMIT_BUFFER_SIZE - 1)
#if (TRANSMIT_BUFFER_SIZE & TRANSMIT_BUFFER_MASK)
	#error "Transmit buffer size must be a power of 2"
#endif
#define TRANSMIT_BUFFER_INCREMENT(x)  (x = (x + 1) & TRANSMIT_BUFFER_MASK)

static uint8_t m_transmit_buffer[TRANSMIT_BUFFER_SIZE] = { 0 };
static uint8_t m_receive_buffer[RECEIVE_BUFFER_SIZE] = { 0 };

static volatile uint32_t m_tx_head = 0;
static volatile uint32_t m_tx_tail = 0;
static volatile bool transmit_in_progress = false;

static usb_cdc_receive rx_callback = NULL;

/*******************************************************************************
* Prototypes
******************************************************************************/

/*******************************************************************************
* Code
******************************************************************************/
void BOARD_DbgConsole_Deinit(void)
{
	DbgConsole_Deinit();
}

void BOARD_DbgConsole_Init(void)
{
	DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR,
		BOARD_DEBUG_UART_BAUDRATE,
		BOARD_DEBUG_UART_TYPE,
		BOARD_DEBUG_UART_CLK_FREQ);
}

void USB0_IRQHandler(void)
{
	USB_DeviceKhciIsrFunction(s_cdcVcom.deviceHandle);
}

void USB_DeviceClockInit(void)
{
	SystemCoreClockUpdate();
	CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
}

void USB_DeviceIsrEnable(void)
{
	uint8_t irqNumber;
	uint8_t usbDeviceKhciIrq[] = USB_IRQS;
	irqNumber = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];

	/* Install isr, set priority, and enable IRQ. */
	NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
	EnableIRQ((IRQn_Type)irqNumber);
}

/*!
 * @brief Interrupt in pipe callback function.
 *
 * This function serves as the callback function for interrupt in pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcAcmInterruptIn(usb_device_handle handle,
	usb_device_endpoint_callback_message_struct_t *message,
	void *callbackParam)
{
	usb_status_t error = kStatus_USB_Error;
	s_cdcVcom.hasSentState = 0;
	return error;
}

static bool start_usb_tx(void)
{
	uint32_t send_length = 0;
	// There is something to send. Send it now (up to 64 bytes of it)
	while((m_tx_tail != m_tx_head) && (send_length < 64))
	{
		s_currSendBuf[send_length] = m_transmit_buffer[m_tx_tail];
		TRANSMIT_BUFFER_INCREMENT(m_tx_tail);
		send_length++;
	}
	usb_status_t rc = USB_DeviceSendRequest(s_cdcVcom.deviceHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, send_length);
	return kStatus_USB_Success == rc;
}

/*!
 * @brief Bulk in pipe callback function.
 *
 * This function serves as the callback function for bulk in pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcAcmBulkIn(usb_device_handle handle,
	usb_device_endpoint_callback_message_struct_t *message,
	void *callbackParam)
{
	usb_status_t error = kStatus_USB_Error;

	if ((message->length != 0) && (!(message->length % s_usbBulkMaxPacketSize)))
	{
		/* If the last packet is the size of endpoint, then send also zero-ended packet,
		 ** meaning that we want to inform the host that we do not have any additional
		 ** data, so it can flush the output.
		 */
		USB_DeviceSendRequest(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
	}
	else if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
	{
		if ((message->buffer != NULL) || ((message->buffer == NULL) && (message->length == 0)))
		{
			/* User: add your own code for send complete event */
			if (m_tx_tail != m_tx_head)
			{
				if (!start_usb_tx())
				{
					transmit_in_progress = false;
				}
			}
			else
			{
				transmit_in_progress = false;
			}
		}
	}
	else
	{
		transmit_in_progress = false;
	}

	return error;
}

/*!
 * @brief Bulk out pipe callback function.
 *
 * This function serves as the callback function for bulk out pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcAcmBulkOut(usb_device_handle handle,
	usb_device_endpoint_callback_message_struct_t *message,
	void *callbackParam)
{
	usb_status_t error = kStatus_USB_Error;

	if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
	{
		// Call the callback and immediately set up for the next receive.
		if(rx_callback)
		{
			rx_callback(s_currRecvBuf, message->length);
		}
		/* Schedule buffer for next receive event */
		USB_DeviceRecvRequest(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf, s_usbBulkMaxPacketSize);
	}
	return error;
}

/*!
 * @brief Get the setup packet buffer.
 *
 * This function provides the buffer for setup packet.
 *
 * @param handle The USB device handle.
 * @param setupBuffer The pointer to the address of setup packet buffer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer)
{
	static uint32_t cdcVcomSetup[2];
	if (NULL == setupBuffer)
	{
		return kStatus_USB_InvalidParameter;
	}
	*setupBuffer = (usb_setup_struct_t *)&cdcVcomSetup;
	return kStatus_USB_Success;
}

/*!
 * @brief Get the setup packet data buffer.
 *
 * This function gets the data buffer for setup packet.
 *
 * @param handle The USB device handle.
 * @param setup The pointer to the setup packet.
 * @param length The pointer to the length of the data buffer.
 * @param buffer The pointer to the address of setup packet data buffer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetClassReceiveBuffer(usb_device_handle handle,
	usb_setup_struct_t *setup,
	uint32_t *length,
	uint8_t **buffer)
{
	static uint8_t setupOut[8];
	if ((NULL == buffer) || ((*length) > sizeof(setupOut)))
	{
		return kStatus_USB_InvalidRequest;
	}
	*buffer = setupOut;
	return kStatus_USB_Success;
}

/*!
 * @brief Configure remote wakeup feature.
 *
 * This function configures the remote wakeup feature.
 *
 * @param handle The USB device handle.
 * @param enable 1: enable, 0: disable.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable)
{
	return kStatus_USB_InvalidRequest;
}

/*!
 * @brief CDC class specific callback function.
 *
 * This function handles the CDC class specific requests.
 *
 * @param handle The USB device handle.
 * @param setup The pointer to the setup packet.
 * @param length The pointer to the length of the data buffer.
 * @param buffer The pointer to the address of setup packet data buffer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
	usb_setup_struct_t *setup,
	uint32_t *length,
	uint8_t **buffer)
{
	usb_status_t error = kStatus_USB_InvalidRequest;

	usb_cdc_acm_info_t *acmInfo = &s_usbCdcAcmInfo;
	uint32_t len;
	uint8_t *uartBitmap;
	if (setup->wIndex != USB_CDC_VCOM_COMM_INTERFACE_INDEX)
	{
		return error;
	}

	switch (setup->bRequest)
	{
	case USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND:
		break;
	case USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE:
		break;
	case USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE:
		if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == setup->wValue)
		{
			*buffer = s_abstractState;
		}
		else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == setup->wValue)
		{
			*buffer = s_countryCode;
		}
		else
		{
		}
		error = kStatus_USB_Success;
		break;
	case USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE:
		if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == setup->wValue)
		{
			*buffer = s_abstractState;
			*length = COMM_FEATURE_DATA_SIZE;
		}
		else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == setup->wValue)
		{
			*buffer = s_countryCode;
			*length = COMM_FEATURE_DATA_SIZE;
		}
		else
		{
		}
		error = kStatus_USB_Success;
		break;
	case USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE:
		break;
	case USB_DEVICE_CDC_REQUEST_GET_LINE_CODING:
		*buffer = s_lineCoding;
		*length = LINE_CODING_SIZE;
		error = kStatus_USB_Success;
		break;
	case USB_DEVICE_CDC_REQUEST_SET_LINE_CODING:
		*buffer = s_lineCoding;
		error = kStatus_USB_Success;
		break;
	case USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE:
		{
			error = kStatus_USB_Success;
			acmInfo->dteStatus = setup->wValue;
			/* activate/deactivate Tx carrier */
			if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
			{
				acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
			}
			else
			{
				acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
			}

			/* activate carrier and DTE */
			if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
			{
				acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
			}
			else
			{
				acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
			}

			/* Indicates to DCE if DTE is present or not */
			acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? true : false;

			/* Initialize the serial state buffer */
			acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE; /* bmRequestType */
			acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_REQUEST_SERIAL_STATE_NOTIF; /* bNotification */
			acmInfo->serialStateBuf[2] = 0x00; /* wValue */
			acmInfo->serialStateBuf[3] = 0x00;
			acmInfo->serialStateBuf[4] = 0x00; /* wIndex */
			acmInfo->serialStateBuf[5] = 0x00;
			acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
			acmInfo->serialStateBuf[7] = 0x00;
			/* Notifiy to host the line state */
			acmInfo->serialStateBuf[4] = setup->wIndex;
			/* Lower byte of UART BITMAP */
			uartBitmap = (uint8_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
			uartBitmap[0] = acmInfo->uartState & 0xFFu;
			uartBitmap[1] = (acmInfo->uartState >> 8) & 0xFFu;
			len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
			if (0 == s_cdcVcom.hasSentState)
			{
				error = USB_DeviceSendRequest(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, acmInfo->serialStateBuf, len);
				if (kStatus_USB_Success != error)
				{
					usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
				}
				s_cdcVcom.hasSentState = 1;
			}
			/* Update status */
			if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
			{
				/*    To do: CARRIER_ACTIVATED */
			}
			else
			{
				/* To do: CARRIER_DEACTIVATED */
			}
			if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
			{
				/* DTE_ACTIVATED */
				if (1 == s_cdcVcom.attach)
				{
					s_cdcVcom.startTransactions = 1;
				}
			}
			else
			{
				/* DTE_DEACTIVATED */
				if (1 == s_cdcVcom.attach)
				{
					s_cdcVcom.startTransactions = 0;
				}
			}
		}
		break;
	case USB_DEVICE_CDC_REQUEST_SEND_BREAK:
		break;
	default:
		break;
	}

	return error;
}

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
	usb_status_t error = kStatus_USB_Error;
	uint8_t *temp8 = (uint8_t *)param;

	switch (event)
	{
	case kUSB_DeviceEventBusReset:
		{
			USB_DeviceControlPipeInit(s_cdcVcom.deviceHandle);
			s_cdcVcom.attach = 0;
		}
		break;
	case kUSB_DeviceEventSetConfiguration:
		if (param)
		{
			s_cdcVcom.attach = 1;
			s_cdcVcom.currentConfiguration = *temp8;
			if (USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
			{
				usb_device_endpoint_init_struct_t epInitStruct;
				usb_device_endpoint_callback_struct_t epCallback;

				/* Initiailize endpoint for interrupt pipe */
				epCallback.callbackFn = USB_DeviceCdcAcmInterruptIn;
				epCallback.callbackParam = handle;

				epInitStruct.zlt = 0;
				epInitStruct.transferType = USB_ENDPOINT_INTERRUPT;
				epInitStruct.endpointAddress = USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT |
				                               (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
				if (USB_SPEED_HIGH == s_cdcVcom.speed)
				{
					epInitStruct.maxPacketSize = HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
				}
				else
				{
					epInitStruct.maxPacketSize = FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
				}

				USB_DeviceInitEndpoint(s_cdcVcom.deviceHandle, &epInitStruct, &epCallback);

				/* Initiailize endpoints for bulk pipe */
				epCallback.callbackFn = USB_DeviceCdcAcmBulkIn;
				epCallback.callbackParam = handle;

				epInitStruct.zlt = 0;
				epInitStruct.transferType = USB_ENDPOINT_BULK;
				epInitStruct.endpointAddress =
				    USB_CDC_VCOM_BULK_IN_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
				if (USB_SPEED_HIGH == s_cdcVcom.speed)
				{
					epInitStruct.maxPacketSize = HS_CDC_VCOM_BULK_IN_PACKET_SIZE;
				}
				else
				{
					epInitStruct.maxPacketSize = FS_CDC_VCOM_BULK_IN_PACKET_SIZE;
				}

				USB_DeviceInitEndpoint(s_cdcVcom.deviceHandle, &epInitStruct, &epCallback);

				epCallback.callbackFn = USB_DeviceCdcAcmBulkOut;
				epCallback.callbackParam = handle;

				epInitStruct.zlt = 0;
				epInitStruct.transferType = USB_ENDPOINT_BULK;
				epInitStruct.endpointAddress =
				    USB_CDC_VCOM_BULK_OUT_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
				if (USB_SPEED_HIGH == s_cdcVcom.speed)
				{
					epInitStruct.maxPacketSize = HS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
				}
				else
				{
					epInitStruct.maxPacketSize = FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
				}

				USB_DeviceInitEndpoint(s_cdcVcom.deviceHandle, &epInitStruct, &epCallback);

				if (USB_SPEED_HIGH == s_cdcVcom.speed)
				{
					s_usbBulkMaxPacketSize = HS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
				}
				else
				{
					s_usbBulkMaxPacketSize = FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
				}
				/* Schedule buffer for receive */
				USB_DeviceRecvRequest(handle,
					USB_CDC_VCOM_BULK_OUT_ENDPOINT,
					s_currRecvBuf,
					s_usbBulkMaxPacketSize);
			}
		}
		break;
	default:
		break;
	}

	return error;
}

/*!
 * @brief USB configure endpoint function.
 *
 * This function configure endpoint status.
 *
 * @param handle The USB device handle.
 * @param ep Endpoint address.
 * @param status A flag to indicate whether to stall the endpoint. 1: stall, 0: unstall.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
	if (status)
	{
		return USB_DeviceStallEndpoint(handle, ep);
	}
	else
	{
		return USB_DeviceUnstallEndpoint(handle, ep);
	}
}

/*!
 * @brief Application initialization function.
 *
 * This function initializes the application.
 *
 * @return None.
 */


/*!
 * @brief Application task function.
 *
 * This function runs the task for application.
 *
 * @return None.
 */
void usb_cdc_task(void)
{
	usb_status_t error = kStatus_USB_Error;
	if ((1 == s_cdcVcom.attach) && (1 == s_cdcVcom.startTransactions))
	{
		/* User Code */
		if ((0 != s_recvSize) && (0xFFFFFFFFU != s_recvSize))
		{
			int32_t i;

			// Call receive callback
			// Re setup for receive	
		}

		if (s_sendSize)
		{
			uint32_t size = s_sendSize;
			s_sendSize = 0;

			error = USB_DeviceSendRequest(s_cdcVcom.deviceHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, size);

			if (error != kStatus_USB_Success)
			{
				/* Failure to send Data Handling code here */
			}
		}
	}
}

void usb_cdc_init(void)
{
	USB_DeviceClockInit();

	s_cdcVcom.speed = USB_SPEED_FULL;
	s_cdcVcom.attach = 0;
	s_cdcVcom.deviceHandle = NULL;

	if (kStatus_USB_Success != USB_DeviceInit(CONTROLLER_ID, USB_DeviceCallback, &s_cdcVcom.deviceHandle))
	{
		usb_echo("USB device vcom failed\r\n");
		return;
	}
	else
	{
		usb_echo("USB device CDC virtual com demo\r\n");
	}

	USB_DeviceIsrEnable();

	USB_DeviceRun(s_cdcVcom.deviceHandle);

}

bool usb_cdc_write(uint8_t * data, uint32_t length)
{
	int i;
	uint32_t available_buffer_size = (m_tx_tail - m_tx_head - 1) & TRANSMIT_BUFFER_MASK;
    
    if (((1 != s_cdcVcom.attach) || (1 != s_cdcVcom.startTransactions)))
    {
        // Not ready yet
        return false;
    }
	
	if (available_buffer_size < length)
	{
		if (!transmit_in_progress)
		{
			// Start a transmit now
			transmit_in_progress = true;
			if (!start_usb_tx())
			{
				transmit_in_progress = false;
			}
		}
		return false;
	}
	
	for (i = 0; i < length; i++)
	{
		m_transmit_buffer[m_tx_head] = data[i];
		TRANSMIT_BUFFER_INCREMENT(m_tx_head);
	}
	
	if (!transmit_in_progress)
	{
		// Start a transmit now
		transmit_in_progress = true;
		if (!start_usb_tx())
		{
			transmit_in_progress = false;
		}
	}
	
	return true;
}

void usb_cdc_set_receive_callback(usb_cdc_receive rx)
{
	rx_callback = rx;
}
