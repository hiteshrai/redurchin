#include "board.h"
#include "fsl_tpm.h"

#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The Flextimer instance/channel used for board */
#define PWM_BASEADDR                   TPM2
#define PWM_CHANNEL                    0U

/* Get source clock for FTM driver */
#define PWM_SOURCE_CLOCK               CLOCK_GetFreq(kCLOCK_McgIrc48MClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
void pwm_init(uint32_t freq)
{
	tpm_config_t tpm_config;
	uint8_t updatedDutycycle = 20U;
	tpm_chnl_pwm_signal_param_t tpm_param;

	/* Configure ftm params with frequency 24kHZ */
	tpm_param.chnlNumber = (tpm_chnl_t)PWM_CHANNEL;
	tpm_param.level = kTPM_HighTrue;
	tpm_param.dutyCyclePercent = 0U;

	/* Select the clock source for the TPM counter as MCGPLLCLK */
	CLOCK_SetTpmClock(1U);
	
	TPM_GetDefaultConfig(&tpm_config);
	/* Initialize FTM module */
	TPM_Init(PWM_BASEADDR, &tpm_config);

	TPM_SetupPwm(PWM_BASEADDR, &tpm_param, 1U, kTPM_EdgeAlignedPwm, freq, PWM_SOURCE_CLOCK);
	TPM_StartTimer(PWM_BASEADDR, kTPM_SystemClock);

	/* Disable channel output before updating the dutycycle */
	TPM_UpdateChnlEdgeLevelSelect(PWM_BASEADDR, (tpm_chnl_t)PWM_CHANNEL, kTPM_NoPwmSignal);

	/* Start PWM mode with updated duty cycle */
	TPM_UpdatePwmDutycycle(PWM_BASEADDR,
		(tpm_chnl_t)PWM_CHANNEL,
		kTPM_EdgeAlignedPwm,
		updatedDutycycle);

	/* Start channel output with updated dutycycle */
	TPM_UpdateChnlEdgeLevelSelect(PWM_BASEADDR, (tpm_chnl_t)PWM_CHANNEL, kTPM_HighTrue);	
}

void pwm_deinit(void)
{
	TPM_Deinit(PWM_BASEADDR);
}
