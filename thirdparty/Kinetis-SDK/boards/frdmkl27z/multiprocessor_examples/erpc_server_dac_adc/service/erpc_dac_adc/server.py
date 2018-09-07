# The Clear BSD License
# Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
# Copyright 2016 NXP
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted (subject to the limitations in the disclaimer below) provided
# that the following conditions are met:
#
# o Redistributions of source code must retain the above copyright notice, this list
#   of conditions and the following disclaimer.
#
# o Redistributions in binary form must reproduce the above copyright notice, this
#   list of conditions and the following disclaimer in the documentation and/or
#   other materials provided with the distribution.
#
# o Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Generated by erpcgen 1.7.0 on Thu Apr  5 09:26:04 2018.
#
# AUTOGENERATED - DO NOT EDIT
#

import erpc
from . import common, interface

# Client for dac_adc
class dac_adcService(erpc.server.Service):
    def __init__(self, handler):
        super(dac_adcService, self).__init__(interface.Idac_adc.SERVICE_ID)
        self._handler = handler
        self._methods = {
                interface.Idac_adc.ADC_GET_CONFIG_ID: self._handle_adc_get_config,
                interface.Idac_adc.CONVERT_DAC_ADC_ID: self._handle_convert_dac_adc,
                interface.Idac_adc.SET_LED_ID: self._handle_set_led,
                interface.Idac_adc.READ_SENZOR_MAG_ACCEL_ID: self._handle_read_senzor_mag_accel,
                interface.Idac_adc.BOARD_GET_NAME_ID: self._handle_board_get_name,
            }

    def _handle_adc_get_config(self, sequence, codec):
        # Create reference objects to pass into handler for out/inout parameters.
        config = erpc.Reference()

        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.adc_get_config(config)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.Idac_adc.SERVICE_ID,
            request=interface.Idac_adc.ADC_GET_CONFIG_ID,
            sequence=sequence))
        if config.value is None:
            raise ValueError("config is None")
        config.value._write(codec)

    def _handle_convert_dac_adc(self, sequence, codec):
        # Create reference objects to pass into handler for out/inout parameters.
        result = erpc.Reference()

        # Read incoming parameters.
        numberToConvert = codec.read_uint32()

        # Invoke user implementation of remote function.
        self._handler.convert_dac_adc(numberToConvert, result)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.Idac_adc.SERVICE_ID,
            request=interface.Idac_adc.CONVERT_DAC_ADC_ID,
            sequence=sequence))
        if result.value is None:
            raise ValueError("result is None")
        codec.write_uint32(result.value)

    def _handle_set_led(self, sequence, codec):
        # Read incoming parameters.
        whichLed = codec.read_uint8()

        # Invoke user implementation of remote function.
        self._handler.set_led(whichLed)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.Idac_adc.SERVICE_ID,
            request=interface.Idac_adc.SET_LED_ID,
            sequence=sequence))

    def _handle_read_senzor_mag_accel(self, sequence, codec):
        # Create reference objects to pass into handler for out/inout parameters.
        results = erpc.Reference()

        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.read_senzor_mag_accel(results)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.Idac_adc.SERVICE_ID,
            request=interface.Idac_adc.READ_SENZOR_MAG_ACCEL_ID,
            sequence=sequence))
        if results.value is None:
            raise ValueError("results is None")
        results.value._write(codec)

    def _handle_board_get_name(self, sequence, codec):
        # Create reference objects to pass into handler for out/inout parameters.
        name = erpc.Reference()

        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.board_get_name(name)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.Idac_adc.SERVICE_ID,
            request=interface.Idac_adc.BOARD_GET_NAME_ID,
            sequence=sequence))
        if name.value is None:
            raise ValueError("name is None")
        codec.write_string(name.value)


