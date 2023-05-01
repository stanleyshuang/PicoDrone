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
        i_rpm_q_size = 10
        self._i_rpm_q = moving_average(i_rpm_q_size+1)


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
    
    @property
    def i_rpm_q_avg(self):
        return self._i_rpm_q.average
    
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
    
    @staticmethod
    def compare_with_max(x, max_value):
        if abs(x) >= max_value:
            return 0
        else:
            return max_value - abs(x)

    def f_balancer(self, gyro, euler_ang, euler_sum, z_accsum, cd, cp, ci, f_tar_ang, f_tar_gyro):
        c_zacc = 92.0
        MAX = 500.0
        max = motor_ctr.compare_with_max(abs(z_accsum*10.0), c_zacc) * MAX / c_zacc
        pid = (euler_ang - f_tar_ang)*cp + (gyro - f_tar_gyro)*cd + euler_sum*ci
        '''
        if pid > max:
            pid = max
        elif pid < -1.0 * max:
            pid = -1.0 * max
        '''
        return pid

    '''
    v0.82b05: Ang 1.00, PID 1.0, 1.5, 0.0, MAX N/A PD 皆沒有發揮功能
    Leo-0408: Ang 1.00, PID 2.0, 5.0, 0.0, MAX N/A
    v0.82b08: Ang 1.00, PID 2.0, 4.0, 0.2, MAX 30  升空後向右後方飛，無法停下來。
    v0.82b13: Ang 1.00, PID 3.0, 7.0, 0.3, MAX N/A D 加大才能克服尾巴的重量。
    v0.82b16: Ang 0.75, PID 3.0, 7.0, 0.3, MAX N/A 縮小Ang以符合觀察現象。
    v0.82b18: Ang 0.75, PID 65.0, 5.0, 4.5, MAX N/A P要比D大才能讓震動逐漸變小？
    '''
    def f_pid_x(self, gyro, euler_ang, euler_sum, z_accsum, cd=2.5, cp=32.5, ci=2.25, f_tar_ang=0.0, f_tar_gyro=0.0):
        return self._I_X * self.f_balancer(gyro, euler_ang, euler_sum, z_accsum, cd, cp, ci, f_tar_ang, f_tar_gyro)

    def f_pid_y(self, gyro, euler_ang, euler_sum, z_accsum, cd=5.0, cp=65.0, ci=4.5, f_tar_ang=0.0, f_tar_gyro=0.0):
        return self._I_Y * self.f_balancer(gyro, euler_ang, euler_sum, z_accsum, cd, cp, ci, f_tar_ang, f_tar_gyro)

    '''
    @staticmethod
    def pitch(ax, ay, az):
        # pitch：atan2(-Ax, sqrt(Ay^2 + Az^2))
        return motor_ctr.angle(math.atan2(ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi)
    '''

    @staticmethod
    def pitch(ax, ay, az):
        # pitch：atan2(Ax, Az)
        return motor_ctr.angle(math.atan2(ax, az) * 180.0 / math.pi)

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
        while a > 180.0:
            a -= 180.0
        if a > 90.0:
            a = a - 180.0
        while a < -180.0:
            a += 180.0
        if a < -90.0:
            a = a + 180.0
 
        if a > 0:
            sign = 1.0
        else:
            sign = -1.0
        a = sign * (abs(a) ** 0.75)
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

    def i_rpm_adj(self, rpm):
        self._i_rpm = int(rpm)
        self._i_rpm_q.update_val(int(rpm))


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

