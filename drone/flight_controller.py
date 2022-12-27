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
import sys, time, utime
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
        self._acc_qs = [ax_q, ay_q, az_q]
        self._gyro_currs = [self._IMU.gyro.x, self._IMU.gyro.y, self._IMU.gyro.z]

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
        begin = utime.ticks_ms()
        if self._bb:
            self._bb.write('    figuring out the baseline of acc sum..')
        ACC_BASE_SAMPLING_COUNT = 20
        acc_sum = [0.0, 0.0, 0.0]
        for i in range(ACC_BASE_SAMPLING_COUNT):
            ax = self._IMU.accel.x
            ay = self._IMU.accel.y
            az = self._IMU.accel.z
            acc_sum[0] += ax * 100.0
            acc_sum[1] += ay * 100.0
            acc_sum[2] += az * 100.0
            self._acc_qs[0].update_val(ax)
            self._acc_qs[1].update_val(ay)
            self._acc_qs[2].update_val(az)
            if i%10==0 and self._bb:
                self._bb.write('    countdown: '+str(int((ACC_BASE_SAMPLING_COUNT-i)/10))+' sec.', end='\r')
            time.sleep(0.09)
        if self._bb:
            self._bb.write('    countdown: 0 sec.')
        acc_base = [0.0, 0.0, 0.0]
        acc_base[0] = acc_sum[0]/ACC_BASE_SAMPLING_COUNT
        acc_base[1] = acc_sum[1]/ACC_BASE_SAMPLING_COUNT
        acc_base[2] = acc_sum[2]/ACC_BASE_SAMPLING_COUNT
        acc_sum_base = acc_base[0]**2 + acc_base[1]**2 + acc_base[2]**2

        # figuring out the baseline of acc sum 
        self._m0.f_based_acc_sum = acc_sum_base
        self._m1.f_based_acc_sum = acc_sum_base
        self._m2.f_based_acc_sum = acc_sum_base
        self._m3.f_based_acc_sum = acc_sum_base
        if self._bb:
            self._bb.write('    Base G: '+str(acc_sum_base))
        end = utime.ticks_ms()
        diff = utime.ticks_diff(end, begin)
        if self._bb:
            self._bb.write('    duration: '+str(round(diff/1000, 2))+' sec.')
        return acc_sum_base


    def simple_mode(self, msg, start, stop, step):
        begin = utime.ticks_ms()
        if self._bb:
            self._bb.write(msg)
        acc_sums = [0.0, 0.0, 0.0]
        gyro_currs = [0.0, 0.0, 0.0]
        step = int(step)
        i = 0
        for rpm in range(start, stop, step):
            self._acc_currs[0] = self._IMU.accel.x
            self._acc_currs[1] = self._IMU.accel.y
            self._acc_currs[2] = self._IMU.accel.z
            self._gyro_currs[0] = self._IMU.gyro.x
            self._gyro_currs[1] = self._IMU.gyro.y
            self._gyro_currs[2] = self._IMU.gyro.z
            self._acc_qs[0].update_val(self._acc_currs[0])
            self._acc_qs[1].update_val(self._acc_currs[1])
            self._acc_qs[2].update_val(self._acc_currs[2])

            acc_sums[0] = self._acc_qs[0].sum
            acc_sums[1] = self._acc_qs[1].sum
            acc_sums[2] = self._acc_qs[2].sum
            gyro_currs[0] = self._gyro_currs[0]
            gyro_currs[1] = -1 * self._gyro_currs[1]
            gyro_currs[2] = self._gyro_currs[2]

            pid_x0 = self._m0.i_pid_x(self._acc_currs[0], acc_sums[0], gyro_currs[1])
            pid_y0 = self._m0.i_pid_y(self._acc_currs[1], acc_sums[1], gyro_currs[0])
            pid_x1 = self._m1.i_pid_x(self._acc_currs[0], acc_sums[0], gyro_currs[1])
            pid_y1 = self._m1.i_pid_y(self._acc_currs[1], acc_sums[1], gyro_currs[0])
            pid_x2 = self._m2.i_pid_x(self._acc_currs[0], acc_sums[0], gyro_currs[1])
            pid_y2 = self._m2.i_pid_y(self._acc_currs[1], acc_sums[1], gyro_currs[0])
            pid_x3 = self._m3.i_pid_x(self._acc_currs[0], acc_sums[0], gyro_currs[1])
            pid_y3 = self._m3.i_pid_y(self._acc_currs[1], acc_sums[1], gyro_currs[0])
            rpm0 = rpm + pid_x0 + pid_y0
            rpm1 = rpm + pid_x1 + pid_y1
            rpm2 = rpm + pid_x2 + pid_y2
            rpm3 = rpm + pid_x3 + pid_y3
            i_m0 = self._m0.rpm2duty(int(rpm0 * self._m0.f_conversion_rate))
            i_m1 = self._m1.rpm2duty(int(rpm1 * self._m1.f_conversion_rate))
            i_m2 = self._m2.rpm2duty(int(rpm2 * self._m2.f_conversion_rate))
            i_m3 = self._m3.rpm2duty(int(rpm3 * self._m3.f_conversion_rate))
            diff_rmp0 = rpm0 - self._m0.rpm
            diff_rmp1 = rpm1 - self._m1.rpm
            diff_rmp2 = rpm2 - self._m2.rpm
            diff_rmp3 = rpm3 - self._m3.rpm
            self._m0.rpm = rpm0
            self._m1.rpm = rpm1
            self._m2.rpm = rpm2
            self._m3.rpm = rpm3
            self._ESC0.duty = i_m0
            self._ESC1.duty = i_m1
            self._ESC2.duty = i_m2
            self._ESC3.duty = i_m3
            if i%10==0 and self._bb:
                self._bb.write('    countdown: '+str(int(i/10))+' sec.', end='\r')
            if self._bb:
                imu_tem = self._IMU.temperature
                self._bb.show_status(self._acc_currs, gyro_currs, acc_sums, imu_tem, 
                                     rpm, rpm0, rpm1, rpm2, rpm3, 
                                     diff_rmp0, diff_rmp1, diff_rmp2, diff_rmp3,
                                     pid_x0, pid_y0, pid_x1, pid_y1, pid_x2, pid_y2, pid_x3, pid_y3)
            time.sleep(0.01) # workload = (0.1 - 0.02)
            i += 1
        if self._bb:
            self._bb.write('    countdown: '+str(int(i/(10)))+' sec.', end='\r')
        end = utime.ticks_ms()
        diff = utime.ticks_diff(end, begin)
        if self._bb:
            self._bb.write('    duration: '+str(round(diff/1000, 2))+' sec.')



    def takeoff(self):
        self.simple_mode('    Take off..', 2000, 4650, 50)

    def ufo_float(self):
        self.simple_mode('    UFO floating..', 4650, 4700, 1)

    def shutdown(self):
        self.simple_mode('    Shutdown..', 4700, 0, -100)
