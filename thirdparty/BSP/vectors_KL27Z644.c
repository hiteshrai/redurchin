/*
	This file contains the definitions of the interrupt handlers for KL27Z644 MCU family.
	The file is provided by Sysprogs under the BSD license.
*/


extern void *_estack;
#define NULL ((void *)0)
#define TRIM_VALUE ((void *)0xFFFFFFFF)

void Reset_Handler();
void Default_Handler();

void NMI_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void HardFault_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void SVC_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void PendSV_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void SysTick_Handler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void DMA3_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void Reserved20_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void FTFA_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void PMC_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void LLWU_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void I2C0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void SPI0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void SPI1_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void LPUART0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void LPUART1_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void UART2_FLEXIO_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void ADC0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void CMP0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void TPM0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void TPM1_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void TPM2_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void RTC_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void RTC_Seconds_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void PIT_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void Reserved39_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void USB0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void Reserved41_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void Reserved42_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void Reserved43_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void LPTMR0_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void Reserved45_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void PORTA_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));
void PORTBCDE_IRQHandler() __attribute__ ((weak, alias ("Default_Handler")));

void * __vect_table[0x30] __attribute__ ((section (".vectortable"))) = 
{
	&_estack,
	&Reset_Handler,
	&NMI_Handler,
	&HardFault_Handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	&SVC_Handler,
	NULL,
	NULL,
	&PendSV_Handler,
	&SysTick_Handler,
	&DMA0_IRQHandler,
	&DMA1_IRQHandler,
	&DMA2_IRQHandler,
	&DMA3_IRQHandler,
	&Reserved20_IRQHandler,
	&FTFA_IRQHandler,
	&PMC_IRQHandler,
	&LLWU_IRQHandler,
	&I2C0_IRQHandler,
	&I2C1_IRQHandler,
	&SPI0_IRQHandler,
	&SPI1_IRQHandler,
	&LPUART0_IRQHandler,
	&LPUART1_IRQHandler,
	&UART2_FLEXIO_IRQHandler,
	&ADC0_IRQHandler,
	&CMP0_IRQHandler,
	&TPM0_IRQHandler,
	&TPM1_IRQHandler,
	&TPM2_IRQHandler,
	&RTC_IRQHandler,
	&RTC_Seconds_IRQHandler,
	&PIT_IRQHandler,
	&Reserved39_IRQHandler,
	&USB0_IRQHandler,
	&Reserved41_IRQHandler,
	&Reserved42_IRQHandler,
	&Reserved43_IRQHandler,
	&LPTMR0_IRQHandler,
	&Reserved45_IRQHandler,
	&PORTA_IRQHandler,
	&PORTBCDE_IRQHandler
};

void Default_Handler()
{
	asm("BKPT 255");
}
