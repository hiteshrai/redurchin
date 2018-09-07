/*
 * The Clear BSD License
 * Copyright 2018 NXP
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

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v1.0
processor: MKL27Z64xxx4
package_id: MKL27Z64VLH4
mcu_data: ksdk2_0
processor_version: 0.0.11
board: FRDM-KL27Z
functionalGroups:
- name: BOARD_InitPeripherals
  called_from_default_init: true
  id_prefix: BOARD_
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * TIMER initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'TIMER'
- type: 'tpm'
- mode: 'EdgeAligned'
- type_id: 'tpm_e7472ea12d53461b8d293488f3ed72ec'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'TPM2'
- config_sets:
  - tpm_main_config:
    - tpm_config:
      - clockSource: 'kTPM_SystemClock'
      - tpmSrcClkFreq: 'custom:48000000'
      - prescale: 'kTPM_Prescale_Divide_1'
      - timerFrequency: '24000'
      - useGlobalTimeBase: 'false'
      - triggerSelect: 'kTPM_Trigger_Select_0'
      - triggerSource: 'kTPM_TriggerSource_External'
      - enableDoze: 'false'
      - enableDebugMode: 'false'
      - enableReloadOnTrigger: 'false'
      - enableStopOnOverflow: 'false'
      - enableStartOnTrigger: 'false'
      - enablePauseOnTrigger: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - tpm_interrupt:
      - IRQn: 'TPM2_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'false'
  - tpm_edge_aligned_mode:
    - tpm_edge_aligned_channels_config:
      - 0:
        - edge_aligned_mode: 'kTPM_EdgeAlignedPwm'
        - edge_aligned_pwm:
          - chnlNumber: 'kTPM_Chnl_0'
          - level: 'kTPM_LowTrue'
          - dutyCyclePercent: '0'
          - enable_chan_irq: 'false'
      - 1:
        - edge_aligned_mode: 'kTPM_EdgeAlignedPwm'
        - edge_aligned_pwm:
          - chnlNumber: 'kTPM_Chnl_1'
          - level: 'kTPM_LowTrue'
          - dutyCyclePercent: '0'
          - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const tpm_config_t BOARD_TIMER_config = {.prescale = kTPM_Prescale_Divide_1,
                                         .useGlobalTimeBase = false,
                                         .triggerSelect = kTPM_Trigger_Select_0,
                                         .triggerSource = kTPM_TriggerSource_External,
                                         .enableDoze = false,
                                         .enableDebugMode = false,
                                         .enableReloadOnTrigger = false,
                                         .enableStopOnOverflow = false,
                                         .enableStartOnTrigger = false,
                                         .enablePauseOnTrigger = false};

const tpm_chnl_pwm_signal_param_t BOARD_TIMER_pwmSignalParams[] = {
    {.chnlNumber = kTPM_Chnl_0, .level = kTPM_LowTrue, .dutyCyclePercent = 0},
    {.chnlNumber = kTPM_Chnl_1, .level = kTPM_LowTrue, .dutyCyclePercent = 0}};

void BOARD_TIMER_init(void)
{
    TPM_Init(BOARD_TIMER_PERIPHERAL, &BOARD_TIMER_config);
    TPM_SetupPwm(BOARD_TIMER_PERIPHERAL, BOARD_TIMER_pwmSignalParams,
                 sizeof(BOARD_TIMER_pwmSignalParams) / sizeof(tpm_chnl_pwm_signal_param_t), kTPM_EdgeAlignedPwm, 24000U,
                 BOARD_TIMER_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * ACCEL_I2C initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ACCEL_I2C'
- type: 'i2c'
- mode: 'I2C_Polling'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'I2C1'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '100000'
      - glitchFilterWidth: '0'
    - quick_selection: 'QS_I2C_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t BOARD_ACCEL_I2C_config = {
    .enableMaster = true, .enableStopHold = false, .baudRate_Bps = 100000, .glitchFilterWidth = 0};

void BOARD_ACCEL_I2C_init(void)
{
    /* Initialization function */
    I2C_MasterInit(BOARD_ACCEL_I2C_PERIPHERAL, &BOARD_ACCEL_I2C_config, BOARD_ACCEL_I2C_CLK_FREQ);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
    /* Initialize components */
    BOARD_TIMER_init();
    BOARD_ACCEL_I2C_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
    BOARD_InitPeripherals();
}
