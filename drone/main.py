#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.81
# Date:     2022-12-20
#
'''
The MIT License (MIT)
Copyright (C) 2022 Stanley Huang, huangstan1215@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy 
of this software and associated documentation files (the "Software"), to deal 
in the Software without restriction, including without limitation the rights 
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
'''
from machine import Pin
import time

# MPU-6050 --------------------------------------------------------------------
from machine import I2C
from imu import MPU6050

# R8EF ------------------------------------------------------------------------
import state_machine
from state_machine import R8EF_channel

# ESC --------------------------------------------------------------
from esc import DSHOT

# Motor Controller -----------------------------------------------------------
from motor_ctr import motor_ctr_fr, motor_ctr_fl
from motor_ctr import motor_ctr_bl, motor_ctr_br

# Flight Controller -----------------------------------------------------------
from flight_controller import flight_controller

# debug module ----------------------------------------------------------------
from flight_data import flight_data
bb = flight_data()
bb.debug_level = 4 # 0: disable, 1: exception, 2: error, 3: warning, 4: information


### initializing MPU-6050
bb.write(4, 'initializing MPU-6050')
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)

### initializing R8EF_channel
bb.write(4, 'initializing R8EF channels')
pin16 = Pin(16, Pin.IN, Pin.PULL_UP)
st0 = R8EF_channel(0, state_machine.mark, in_base=pin16, jmp_pin=pin16)
st0.active(1)

pin17 = Pin(17, Pin.IN, Pin.PULL_UP)
st1 = R8EF_channel(1, state_machine.mark, in_base=pin17, jmp_pin=pin17)
st1.active(1)

pin18 = Pin(18, Pin.IN, Pin.PULL_UP)
st2 = R8EF_channel(2, state_machine.mark, in_base=pin18, jmp_pin=pin18)
st2.active(1)

### initializing SimonK PWM
bb.write(4, 'initializing ESC')
esc_0 = DSHOT(Pin(6))
esc_1 = DSHOT(Pin(7))
esc_2 = DSHOT(Pin(8))
esc_3 = DSHOT(Pin(9))
time.sleep(1.0)


### initializing Motor Controllers
bb.write(4, 'initializing Motor Controllers')
#                          i_values=[init,  min, balance,   max, limit]
motor_ctr_0 = motor_ctr_br(i_values=[ 300,  393,     580,  1747,  2047], i_adjst=33)
motor_ctr_1 = motor_ctr_fr(i_values=[ 300,  393,     580,  1747,  2047], i_adjst=-1)
motor_ctr_2 = motor_ctr_fl(i_values=[ 300,  393,     580,  1747,  2047], i_adjst=1)
motor_ctr_3 = motor_ctr_bl(i_values=[ 300,  393,     580,  1747,  2047], i_adjst=31)

esc_0.value = motor_ctr_0.init_value
esc_1.value = motor_ctr_1.init_value
esc_2.value = motor_ctr_2.init_value
esc_3.value = motor_ctr_3.init_value


### before taking off, initialize FlightController
bb.write(4, 'before taking off, initialize FlightController')

# the value range of the joysticks
st_matrics = [ # min,  mid,  max
                [0, 4888, 9928],
                [0, 5031, 9966],
                [0, 4925, 9920],
           ]

flight_ctr = flight_controller(imu, st0, st1, st2, st_matrics, 
                               esc_0, esc_1, esc_2, esc_3,
                               motor_ctr_0, motor_ctr_1, motor_ctr_2, motor_ctr_3)
flight_ctr.debug = bb
flight_ctr.b_pid = True

flight_ctr.init()
flight_ctr.takeoff()
flight_ctr.ufo_float()
flight_ctr.shutdown()
bb.write(4, 'demo complete, log flushing and closed')
bb.flush()
bb.close()
