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

# Constant variable declarations
StringMaxSize = 11

# Structures data types declarations
class AdcConfig(object):
    def __init__(self, vref=None, atomicSteps=None):
        self.vref = vref # float
        self.atomicSteps = atomicSteps # float

    def _read(self, codec):
        self.vref = codec.read_float()
        self.atomicSteps = codec.read_float()
        return self

    def _write(self, codec):
        if self.vref is None:
            raise ValueError("vref is None")
        codec.write_float(self.vref)
        if self.atomicSteps is None:
            raise ValueError("atomicSteps is None")
        codec.write_float(self.atomicSteps)

    def __str__(self):
        return "<%s@%x vref=%s atomicSteps=%s>" % (self.__class__.__name__, id(self), self.vref, self.atomicSteps)

    def __repr__(self):
        return self.__str__()
        
class Vector(object):
    def __init__(self, A_x=None, A_y=None, A_z=None, M_x=None, M_y=None, M_z=None):
        self.A_x = A_x # int16
        self.A_y = A_y # int16
        self.A_z = A_z # int16
        self.M_x = M_x # int16
        self.M_y = M_y # int16
        self.M_z = M_z # int16

    def _read(self, codec):
        self.A_x = codec.read_int16()
        self.A_y = codec.read_int16()
        self.A_z = codec.read_int16()
        self.M_x = codec.read_int16()
        self.M_y = codec.read_int16()
        self.M_z = codec.read_int16()
        return self

    def _write(self, codec):
        if self.A_x is None:
            raise ValueError("A_x is None")
        codec.write_int16(self.A_x)
        if self.A_y is None:
            raise ValueError("A_y is None")
        codec.write_int16(self.A_y)
        if self.A_z is None:
            raise ValueError("A_z is None")
        codec.write_int16(self.A_z)
        if self.M_x is None:
            raise ValueError("M_x is None")
        codec.write_int16(self.M_x)
        if self.M_y is None:
            raise ValueError("M_y is None")
        codec.write_int16(self.M_y)
        if self.M_z is None:
            raise ValueError("M_z is None")
        codec.write_int16(self.M_z)

    def __str__(self):
        return "<%s@%x A_x=%s A_y=%s A_z=%s M_x=%s M_y=%s M_z=%s>" % (self.__class__.__name__, id(self), self.A_x, self.A_y, self.A_z, self.M_x, self.M_y, self.M_z)

    def __repr__(self):
        return self.__str__()
        

