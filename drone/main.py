#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.8
# Date:     2022-11-25
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

# ZMR SimonK ------------------------------------------------------------------
from simonk_pwm import ZMR

# Flight Controller -----------------------------------------------------------
from flight_controller import acc_sum_base, ufo_float, shutdown, main_loop
from flight_controller import flight_ctr_fr, flight_ctr_fl
from flight_controller import flight_ctr_bl, flight_ctr_br

# debug module ----------------------------------------------------------------
from flight_data import flight_data
bb = flight_data(b_debug=True)


### initializing MPU-6050
bb.write('initializing MPU-6050')
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)

### initializing R8EF_channel
bb.write('initializing R8EF channels')
pin16 = Pin(16, Pin.IN, Pin.PULL_UP)
st0 = R8EF_channel(0, state_machine.mark, in_base=pin16, jmp_pin=pin16)
st0.active(1)

pin17 = Pin(17, Pin.IN, Pin.PULL_UP)
st1 = R8EF_channel(1, state_machine.mark, in_base=pin17, jmp_pin=pin17)
st1.active(1)

pin18 = Pin(18, Pin.IN, Pin.PULL_UP)
st2 = R8EF_channel(2, state_machine.mark, in_base=pin18, jmp_pin=pin18)
st2.active(1)

# the value range of the joysticks
st_range = [ # min, mid, max
            [0, 4888, 9928],
            [0, 5031, 9966],
            [0, 4925, 9920],
           ]


### initializing SimonK PWM
bb.write('initializing SimonK ESC')
# dutys = [min, max, init, limit, seatbelt]
motor_0 = ZMR(Pin(6), dutys=[ 3000, 62400,   200, 68000, 20710,  2250,  7950, 15350, 23100])
motor_1 = ZMR(Pin(7), dutys=[ 2600, 45600,   400, 67200, 14710,  2825,  7100, 12500, 17750])
motor_2 = ZMR(Pin(8), dutys=[ 5400, 63200,  2000, 69600, 21445,  5825, 11500, 18500, 25750])
motor_3 = ZMR(Pin(9), dutys=[30500, 51200, 20000, 69600, 34548, 30980, 32250, 33850, 35500])
time.sleep(1.0)
motor_0.duty = motor_0.min_duty
motor_1.duty = motor_1.min_duty
motor_2.duty = motor_2.min_duty
motor_3.duty = motor_3.min_duty


### initializing Flight Controllers
bb.write('initializing Flight Controllers')
flight_ctr_0 = flight_ctr_fr('fc0', st_range, motor_0, pwr_cr=4, debug_obj=bb)
flight_ctr_1 = flight_ctr_fl('fc1', st_range, motor_1, pwr_cr=3, debug_obj=bb)
flight_ctr_2 = flight_ctr_bl('fc2', st_range, motor_2, pwr_cr=4, debug_obj=bb)
flight_ctr_3 = flight_ctr_br('fc3', st_range, motor_3, pwr_cr=1, debug_obj=bb)


### before taking off, initialize PicoDrone
bb.write('before taking off, initialize PicoDrone')
# figuring out the baseline of acc sum 
based_acc_sum = acc_sum_base(imu, bb)
flight_ctr_0.based_acc_sum = based_acc_sum
flight_ctr_1.based_acc_sum = based_acc_sum
flight_ctr_2.based_acc_sum = based_acc_sum
flight_ctr_3.based_acc_sum = based_acc_sum


# figuring out the acc sum at the boundary of escape gravity
es_acc_sum = ufo_float(imu, 
                       flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
                       motor_0, motor_1, motor_2, motor_3,
                       bb=bb)
if es_acc_sum==0:
    es_acc_sum = based_acc_sum
    
flight_ctr_0.es_acc_sum = es_acc_sum
flight_ctr_1.es_acc_sum = es_acc_sum
flight_ctr_2.es_acc_sum = es_acc_sum
flight_ctr_3.es_acc_sum = es_acc_sum

main_loop(imu, st0, st1, st2, 
          flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
          motor_0, motor_1, motor_2, motor_3,
          bb=bb, sec=3,
          st0_val=5000, st1_val=5000, st2_val=st_range[2][1])

shutdown(imu, 
         flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
         motor_0, motor_1, motor_2, motor_3, 
         bb=bb)


### entering the main loop
bb.write('entering the main loop')
main_loop(imu, st0, st1, st2, 
          flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
          motor_0, motor_1, motor_2, motor_3,
          bb)
