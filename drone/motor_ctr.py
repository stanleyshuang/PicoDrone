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
import math

from moving_average import moving_average


class motor_ctr():
    def __init__(self, i_values, i_adjst=0):
        self._i_values = i_values
        self._i_adjst = i_adjst
        self._I_X = 0
        self._I_Y = 0
        self._i_rpm = 0


    @property
    def init_value(self):
        return self._i_values[0]

    @property
    def min_value(self):
        return self._i_values[1]

    @property
    def balance_value(self):
        return self._i_values[2]

    @property
    def max_value(self):
        return self._i_values[3]

    @property
    def limit_value(self):
        return self._i_values[4]
    
    @property
    def i_adjst(self):
        return self._i_adjst
    
    '''
    @property
    def f_conversion_rate(self):
        return 1.0
        # return self._F_CR
        # return ((self._F_CR * self.max_value - self.init_value) - self._i_rpm * (self._F_CR - 1.0)) / (self.max_value - self.init_value)
    '''
        
    @property
    def i_x(self):
        return self._I_X
        
    @property
    def i_y(self):
        return self._I_Y

    def f_balancer(self, d, p, i, f_baseline):
        pid = (p - f_baseline)*0.0 + d*0.0 + i*0.0
        return pid

    def f_pid_x(self, d, p, i, f_baseline=0.0):
        return self._I_X * self.f_balancer(d, p, i, f_baseline)

    def f_pid_y(self, d, p, i, f_baseline=0.0):
        return self._I_Y * self.f_balancer(d, p, i, f_baseline)


    @staticmethod
    def pitch(ax, ay, az):
        # pitch：atan2(-Ax, sqrt(Ay^2 + Az^2))
        return motor_ctr.angle(math.atan2(ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi)

    @staticmethod
    def roll(ax, ay, az):
        # roll：atan2(Ay, Az)
        return motor_ctr.angle(math.atan2(ay, az) * 180.0 / math.pi)

    @staticmethod
    def yaw(gx, gy, gz):
        # yaw：atan2(Gx, sqrt(Gy^2 + Gz^2))
        return motor_ctr.angle(math.atan2(gx, math.sqrt(gy*gy + gz*gz)) * 180.0 / math.pi)

    @staticmethod
    def angle(a):
        while a > 135.0:
            a -= 180.0
        while a < -135.0:
            a += 180.0
        return a


    def i_rpm_bound_check(self, rpm):
        if rpm < self.min_value:
            rpm = self.min_value
        elif rpm > self.max_value:
            rpm = self.max_value
        return int(rpm)

    @property
    def i_rpm(self):
        return self._i_rpm

    @i_rpm.setter
    def i_rpm(self, rpm):
        self._i_rpm = int(rpm)


class motor_ctr_fr(motor_ctr):
    def __init__(self, i_values=[300, 393, 580, 1747, 2047], i_adjst=0):
        super(motor_ctr_fr, self).__init__(i_values, i_adjst)
        self._I_X =  1
        self._I_Y =  1


class motor_ctr_fl(motor_ctr):
    def __init__(self, i_values=[300, 393, 580, 1747, 2047], i_adjst=0):
        super(motor_ctr_fl, self).__init__(i_values, i_adjst)
        self._I_X = -1
        self._I_Y =  1


class motor_ctr_bl(motor_ctr):
    def __init__(self, i_values=[300, 393, 580, 1747, 2047], i_adjst=0):
        super(motor_ctr_bl, self).__init__(i_values, i_adjst)
        self._I_X = -1
        self._I_Y = -1


class motor_ctr_br(motor_ctr):
    def __init__(self, i_values=[300, 393, 580, 1747, 2047], i_adjst=0):
        super(motor_ctr_br, self).__init__(i_values, i_adjst)
        self._I_X =  1
        self._I_Y = -1

