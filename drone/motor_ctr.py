#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.81
# Date:     2022-12-20
#
# The flight controller of PicoDrone
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
from moving_average import moving_average


class motor_ctr():
    def __init__(self, duties, cr):
        self._duties = duties # SimonK ESC pwm duty parameters
        self._F_UNIT = (self.i_max_duty - self.i_min_duty)/3000
        self._F_CR = cr # conversion rate
        self._I_AX = 0
        self._I_AY = 0
        self._I_BASE_ACC_SUM = 0
        self._rpm = 0


    @property
    def i_init_duty(self):
        return self._duties[0]

    @property
    def i_min_duty(self):
        return self._duties[1]

    @property
    def i_balance_duty(self):
        return self._duties[2]

    @property
    def i_max_duty(self):
        return self._duties[3]

    @property
    def i_limit_duty(self):
        return self._duties[4]
        
    @property
    def f_conversion_rate(self):
        return self._F_CR
        
    @property
    def i_ax(self):
        return self._I_AX
        
    @property
    def i_ay(self):
        return self._I_AY

    def i_duty2rpm(self, duty):
        return int(2000 + (duty-self.i_min_duty)/self._F_UNIT)

    def i_rpm2duty(self, rpm):
        return int(self.i_min_duty + (rpm-2000)*self._F_UNIT)

    def i_balancer(self, curr, ma, max_add_rpm, boundary, baseline):
        p = curr - baseline
        if p == 0:
            return 0.0

        if (curr > ma and ma > baseline) or (baseline > ma and ma > curr):
            if abs(curr - baseline) < boundary/10:
                return int(max_add_rpm * (p/boundary))
        else:
            if abs(curr - baseline) < boundary:
                return int(max_add_rpm * (p/boundary))
        if p < 0:
            return int(-1 * max_add_rpm)
        return max_add_rpm


    def i_pid_x(self, curr, ma, max_add_rpm=150, boundary=0.75, baseline=0.0):
        return self._I_AX * self.i_balancer(curr, ma, max_add_rpm, boundary, baseline)

    def i_pid_y(self, curr, ma, max_add_rpm=150, boundary=0.75, baseline=0.0):
        return self._I_AY * self.i_balancer(curr, ma, max_add_rpm, boundary, baseline)

    @property
    def i_based_acc_sum(self):
        return self._I_BASE_ACC_SUM

    @i_based_acc_sum.setter
    def i_based_acc_sum(self, based_acc_sum):
        self._I_BASE_ACC_SUM = based_acc_sum

    @property
    def rpm(self):
        return self._rpm

    @rpm.setter
    def rpm(self, rpm):
        self._rpm = rpm


class motor_ctr_fr(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_fr, self).__init__(duties, cr)
        self._I_AX = -1
        self._I_AY = 1


class motor_ctr_fl(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_fl, self).__init__(duties, cr)
        self._I_AX = -1
        self._I_AY = -1


class motor_ctr_bl(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_bl, self).__init__(duties, cr)
        self._I_AX = 1
        self._I_AY = -1


class motor_ctr_br(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_br, self).__init__(duties, cr)
        self._I_AX = 1
        self._I_AY = 1
