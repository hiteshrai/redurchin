/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
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
#include "fsl_gpio.h"

#include "custom_board.h"
#include "pwm.h"


#include "ui.h"
#include "drivers/clock.h"
#include "app/command.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// UI PWM
#define UI_PWM_BASEADDR                   TPM2
#define UI_PWM_CHANNEL                    0U

/*******************************************************************************
 * Variables
 ******************************************************************************/
pwm_info_t ui_pwm_info = 
{ 
    .baseaddr = UI_PWM_BASEADDR,
    .channel = UI_PWM_CHANNEL,
};

static int current_brightness = 0;
static uint32_t last_led_change_tick = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void ui_brighten(void)
{
	current_brightness++;
	if (current_brightness > 100)
	{
		current_brightness = 100;
	}
	pwm_set_duty_cycle(&ui_pwm_info, current_brightness);
}

void ui_darken(void)
{
	current_brightness--;
	if (current_brightness < 0)
	{
		current_brightness = 0;
	}
	pwm_set_duty_cycle(&ui_pwm_info, current_brightness);    
}

void ui_init(void)
{
	CLOCK_EnableClock(UI_LED_CLOCK);
    PORT_SetPinMux(UI_LED_PORT, UI_LED_PIN, kPORT_MuxAlt3);
	
    pwm_init(&ui_pwm_info, 1000);
    pwm_set_duty_cycle(&ui_pwm_info, current_brightness);
	
	last_led_change_tick = clock_get_tick();
}

void ui_update(void)
{
	if (clock_get_elapsed_time_ms(clock_get_tick(), last_led_change_tick) >= 10)
	{
		if (command_is_dark_mode())
		{
			ui_darken();
		}
		else
		{
			ui_brighten();                
		}
		
		last_led_change_tick = clock_get_tick();
	}
}
