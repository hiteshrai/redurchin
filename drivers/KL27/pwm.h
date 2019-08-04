#ifndef _PWM_H_
#define _PWM_H_

#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct
{
    TPM_Type *baseaddr;
    uint32_t channel;
} pwm_info_t;

/*******************************************************************************
* API
******************************************************************************/
void pwm_init(pwm_info_t *pwm_info, uint32_t freq);
void pwm_deinit(pwm_info_t *pwm_info);
void pwm_set_duty_cycle(pwm_info_t *pwm_info, int percent);

#endif /* _PWM_H_ */
