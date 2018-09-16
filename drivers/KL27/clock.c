#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_lptmr.h"
#include "fsl_gpio.h"

#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LPTMR_HANDLER           LPTMR0_IRQHandler
#define LPTMR_SOURCE_CLOCK      CLOCK_GetFreq(kCLOCK_McgInternalRefClk)

#define MSEC_PER_TICK           1


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t lptmrCounter = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/
void LPTMR_HANDLER(void)
{
	LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
	lptmrCounter++;
}

void clock_init(void)
{
	enum 
	{
		TICK_COUNT_MS = 1000
	};
    lptmr_config_t lptmrConfig;

	LPTMR_GetDefaultConfig(&lptmrConfig);
	lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_0;

	/* Initialize the LPTMR */
	LPTMR_Init(LPTMR0, &lptmrConfig);

	/* Set timer period */
	LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(TICK_COUNT_MS, LPTMR_SOURCE_CLOCK));

	/* Enable timer interrupt */
	LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(LPTMR0_IRQn);
	
	LPTMR_StartTimer(LPTMR0);
}

uint32_t clock_get_tick(void)
{
	return lptmrCounter;
}

uint32_t clock_get_time_ms(void)
{
	return (lptmrCounter * MSEC_PER_TICK);
}

uint32_t clock_get_elapsed_time_ms(uint32_t later_tick, uint32_t previous_tick)
{
	return ((later_tick - previous_tick) * MSEC_PER_TICK);
}



