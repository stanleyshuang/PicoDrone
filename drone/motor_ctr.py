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
        self._M_UNIT = (self.max_duty - self.min_duty)/3000
        self._CR = cr # conversion rate
        self._AX = 0.0
        self._AY = 0.0


    @property
    def init_duty(self):
        return self._duties[0]

    @property
    def min_duty(self):
        return self._duties[1]

    @property
    def balance_duty(self):
        return self._duties[2]

    @property
    def max_duty(self):
        return self._duties[3]

    @property
    def limit_duty(self):
        return self._duties[4]
        
    @property
    def conversion_rate(self):
        return self._CR
        
    @property
    def ax(self):
        return self._AX
        
    @property
    def ay(self):
        return self._AY

    def duty2rpm(self, duty):
        return int(2000 + (duty-self.min_duty)/self._M_UNIT)

    def rpm2duty(self, rpm):
        return int(self.min_duty + (rpm-2000)*self._M_UNIT)

    def balance_pid(self, curr, max_add_rpm=500, boundary=100.0, baseline=0.0):
        p = curr - baseline
        if p < boundary:
            return max_add_rpm * (p/boundary)
        return max_add_rpm


class motor_ctr_fr(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_fr, self).__init__(duties, cr)
        self._AX = -1.0
        self._AY = 1.0


class motor_ctr_fl(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_fl, self).__init__(duties, cr)
        self._AX = -1.0
        self._AY = -1.0


class motor_ctr_bl(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_bl, self).__init__(duties, cr)
        self._AX = 1.0
        self._AY = -1.0


class motor_ctr_br(motor_ctr):
    def __init__(self, duties=[26214, 26214,   39321, 52428, 52428], cr=1.0):
        #              duties=[ init,   min, balance,   max, limit]
        super(motor_ctr_br, self).__init__(duties, cr)
        self._AX = 1.0
        self._AY = 1.0
