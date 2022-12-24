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
import utime
from moving_average import moving_average


class flight_controller():
    def __init__(self, imu, st0, st1, st2, st_matrics, 
                 esc0, esc1, esc2, esc3, 
                 motor_ctr_0, motor_ctr_1, motor_ctr_2, motor_ctr_3, 
                 debug_obj=None):
        self._IMU = imu
        self._acc_currs = [self._IMU.accel.x, self._IMU.accel.y, self._IMU.accel.z]
        i_acc_q_size = 3
        ax_q = moving_average(i_acc_q_size+1)
        ay_q = moving_average(i_acc_q_size+1)
        az_q = moving_average(i_acc_q_size+1)
        self._acc_mas = [ax_q, ay_q, az_q]
        self._gyro_currs = [0.0, 0.0, 0.0]

        self._ST0 = st0
        self._ST1 = st1
        self._ST2 = st2
        self._i_st_matrics = st_matrics # stick parameters
        i_st_q_size = 10
        st0_q = moving_average(i_st_q_size+1)
        st1_q = moving_average(i_st_q_size+1)
        st2_q = moving_average(i_st_q_size+1)
        self._st_q = [st0_q, st1_q, st2_q]
        for i in range(i_st_q_size):
            self._st_q[0].update_val(self._i_st_matrics[0][1])
            self._st_q[1].update_val(self._i_st_matrics[1][1])
            self._st_q[2].update_val(self._i_st_matrics[2][1])

        self._ESC0 = esc0
        self._ESC1 = esc1
        self._ESC2 = esc2
        self._ESC3 = esc3

        self._m0 = motor_ctr_0
        self._m1 = motor_ctr_1
        self._m2 = motor_ctr_2
        self._m3 = motor_ctr_3

        self._bb = debug_obj
        self._debug_show_detail = False


    def acc_sum_base(self):
        ### figuring out the baseline of acc sum
        import time
        if self._bb:
            self._bb.write('    figuring out the baseline of acc sum..')
        ACC_BASE_SAMPLING_COUNT = 15
        i_acc_sum = [0, 0, 0]
        for i in range(ACC_BASE_SAMPLING_COUNT):
            ax = self._IMU.accel.x
            ay = self._IMU.accel.y
            az = self._IMU.accel.z
            i_acc_sum[0] += int(ax * 100)
            i_acc_sum[1] += int(ay * 100)
            i_acc_sum[2] += int(az * 100)
            self._acc_mas[0].update_val(ax)
            self._acc_mas[1].update_val(ay)
            self._acc_mas[2].update_val(az)
            if i%10==0 and self._bb:
                self._bb.write('    countdown: '+str(int((ACC_BASE_SAMPLING_COUNT-i)/10))+' sec.', end='\r')
            time.sleep(0.1)
        if self._bb:
            self._bb.write('    countdown: 0 sec.')
        i_acc_base = [0, 0, 0]
        i_acc_base[0] = int(i_acc_sum[0]/ACC_BASE_SAMPLING_COUNT)
        i_acc_base[1] = int(i_acc_sum[1]/ACC_BASE_SAMPLING_COUNT)
        i_acc_base[2] = int(i_acc_sum[2]/ACC_BASE_SAMPLING_COUNT)
        i_acc_sum_base = i_acc_base[0]**2 + i_acc_base[1]**2 + i_acc_base[2]**2

        # figuring out the baseline of acc sum 
        self._m0.i_based_acc_sum = i_acc_sum_base
        self._m1.i_based_acc_sum = i_acc_sum_base
        self._m2.i_based_acc_sum = i_acc_sum_base
        self._m3.i_based_acc_sum = i_acc_sum_base
        if self._bb:
            self._bb.write('    Base G: '+str(i_acc_sum_base))
        return i_acc_sum_base


    def simple_mode(self, msg, start, stop, step):
        begin = utime.ticks_ms()
        import sys, time
        if self._bb:
            self._bb.write(msg)
        acc_mas = [0, 0, 0]
        prob = 1
        step = int(step/prob)
        i = 0
        for rpm in range(start, stop, step):
            self._acc_currs[0] = self._IMU.accel.x
            self._acc_currs[1] = self._IMU.accel.y
            self._acc_currs[2] = self._IMU.accel.z
            self._acc_mas[0].update_val(self._acc_currs[0])
            self._acc_mas[1].update_val(self._acc_currs[1])
            self._acc_mas[2].update_val(self._acc_currs[2])
            if i%prob==0:
                pid_x0 = self._m0.i_pid_x(self._acc_currs[0], self._acc_mas[0].average)
                pid_y0 = self._m0.i_pid_y(self._acc_currs[1], self._acc_mas[1].average)
                pid_x1 = self._m1.i_pid_x(self._acc_currs[0], self._acc_mas[0].average)
                pid_y1 = self._m1.i_pid_y(self._acc_currs[1], self._acc_mas[1].average)
                pid_x2 = self._m2.i_pid_x(self._acc_currs[0], self._acc_mas[0].average)
                pid_y2 = self._m2.i_pid_y(self._acc_currs[1], self._acc_mas[1].average)
                pid_x3 = self._m3.i_pid_x(self._acc_currs[0], self._acc_mas[0].average)
                pid_y3 = self._m3.i_pid_y(self._acc_currs[1], self._acc_mas[1].average)
                rpm0 = rpm + pid_x0 + pid_y0
                rpm1 = rpm + pid_x1 + pid_y1
                rpm2 = rpm + pid_x2 + pid_y2
                rpm3 = rpm + pid_x3 + pid_y3
                i_m0 = self._m0.i_rpm2duty(int(rpm0 * self._m0.f_conversion_rate))
                i_m1 = self._m1.i_rpm2duty(int(rpm1 * self._m1.f_conversion_rate))
                i_m2 = self._m2.i_rpm2duty(int(rpm2 * self._m2.f_conversion_rate))
                i_m3 = self._m3.i_rpm2duty(int(rpm3 * self._m3.f_conversion_rate))
                self._ESC0.duty = i_m0
                self._ESC1.duty = i_m1
                self._ESC2.duty = i_m2
                self._ESC3.duty = i_m3
                if i%(10*prob)==0 and self._bb:
                    self._bb.write('    countdown: '+str(int(i/(10*prob)))+' sec.', end='\r')
                if self._bb:
                    acc_mas[0] = self._acc_mas[0].average
                    acc_mas[1] = self._acc_mas[1].average
                    acc_mas[2] = self._acc_mas[2].average
                    imu_tem = self._IMU.temperature
                    self._bb.show_status(self._acc_currs, acc_mas, imu_tem, rpm, pid_x0, pid_y0, pid_x1, pid_y1, pid_x2, pid_y2, pid_x3, pid_y3)
            time.sleep(0.01/prob) # workload = (0.1 - 0.02)
            i += 1
        if self._bb:
            self._bb.write('    countdown: '+str(int(i/(10*prob)))+' sec.', end='\r')
        end = utime.ticks_ms()
        diff = utime.ticks_diff(end, begin)
        if self._bb:
            self._bb.write('    duration: '+str(round(diff/1000, 2))+' sec.')



    def takeoff(self):
        self.simple_mode('    Take off..', 2000, 4650, 50)


    def shutdown(self):
        self.simple_mode('    Shutdown..', 4640, 0, -60)
