#include "board.h"
#include "fsl_tpm.h"
#include "pwm.h"

#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

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
void pwm_init(pwm_info_t *pwm_info, uint32_t freq)
{
    tpm_config_t tpm_config;
    tpm_chnl_pwm_signal_param_t tpm_param;

    /* Configure ftm params with frequency 24kHZ */
    tpm_param.chnlNumber = (tpm_chnl_t)pwm_info->channel;
    tpm_param.level = kTPM_HighTrue;
    tpm_param.dutyCyclePercent = 0U;

    /* Select the clock source for the TPM counter as MCGPLLCLK */
    CLOCK_SetTpmClock(1U);
	
    TPM_GetDefaultConfig(&tpm_config);
    /* Initialize FTM module */
    TPM_Init(pwm_info->baseaddr, &tpm_config);

    TPM_SetupPwm(pwm_info->baseaddr, &tpm_param, 1U, kTPM_EdgeAlignedPwm, freq, PWM_SOURCE_CLOCK);
    TPM_StartTimer(pwm_info->baseaddr, kTPM_SystemClock);
}

void pwm_set_duty_cycle(pwm_info_t *pwm_info, int percent)
{
    if (percent < 0)
    {
        percent = 0;
    }
    else if (percent > 100)
    {
        percent = 100;
    }

    /* Start PWM mode with updated duty cycle */
    TPM_UpdatePwmDutycycle(pwm_info->baseaddr,
        (tpm_chnl_t)pwm_info->channel,
        kTPM_EdgeAlignedPwm,
        percent);
}

void pwm_deinit(pwm_info_t *pwm_info)
{
    TPM_Deinit(pwm_info->baseaddr);
}
