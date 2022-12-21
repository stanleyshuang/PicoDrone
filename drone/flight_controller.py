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


class flight_controller():
    def __init__(self, imu, st0, st1, st2, st_matrics, 
                 esc0, esc1, esc2, esc3, 
                 motor_ctr_0, motor_ctr_1, motor_ctr_2, motor_ctr_3, 
                 debug_obj=None):
        self._IMU = imu
        self._acc_vals = [0, 0, 0]
        self._gyro_vals = [0, 0, 0]

        self._ST0 = st0
        self._ST1 = st1
        self._ST2 = st2
        self._st_matrics = st_matrics # stick parameters
        st_q_size = 10
        st0_q = moving_average(st_q_size+1)
        st1_q = moving_average(st_q_size+1)
        st2_q = moving_average(st_q_size+1)
        self._st_q = [st0_q, st1_q, st2_q]
        for i in range(st_q_size):
            self._st_q[0].update_val(self._st_matrics[0][1])
            self._st_q[1].update_val(self._st_matrics[1][1])
            self._st_q[2].update_val(self._st_matrics[2][1])

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
        ACC_BASE_SAMPLING_COUNT = 30
        acc_sum = [0, 0, 0]
        for i in range(ACC_BASE_SAMPLING_COUNT):
            acc_sum[0] += int(self._IMU.accel.x * 100)
            acc_sum[1] += int(self._IMU.accel.y * 100)
            acc_sum[2] += int(self._IMU.accel.z * 100)
            if i%10==0 and self._bb:
                self._bb.write('    countdown: '+str(int((ACC_BASE_SAMPLING_COUNT-i)/10))+' sec.', end='\r')
            time.sleep(0.1)
        if self._bb:
            self._bb.write('    countdown: 0 sec.')
        acc_base = [0, 0, 0]
        acc_base[0] = int(acc_sum[0]/ACC_BASE_SAMPLING_COUNT)
        acc_base[1] = int(acc_sum[1]/ACC_BASE_SAMPLING_COUNT)
        acc_base[2] = int(acc_sum[2]/ACC_BASE_SAMPLING_COUNT)
        acc_sum_base = acc_base[0]**2 + acc_base[1]**2 + acc_base[2]**2

        # figuring out the baseline of acc sum 
        self._m0.based_acc_sum = acc_sum_base
        self._m1.based_acc_sum = acc_sum_base
        self._m2.based_acc_sum = acc_sum_base
        self._m3.based_acc_sum = acc_sum_base
        if self._bb:
            self._bb.write('    Base G: '+str(acc_sum_base))
        return acc_sum_base


    def simple_mode(self, msg, start, stop, step):
        import sys, time
        if self._bb:
            self._bb.write(msg)
        i = 0
        for rpm in range(start, stop, step):
            try:
                m0 = self._m0.rpm2duty(rpm * 1.01)
                m1 = self._m1.rpm2duty(rpm)
                m2 = self._m2.rpm2duty(rpm * 0.99)
                m3 = self._m3.rpm2duty(rpm)
                self._ESC0.duty = m0
                self._ESC1.duty = m1
                self._ESC2.duty = m2
                self._ESC3.duty = m3
                if i%10==0 and self._bb:
                    self._bb.write('    countdown: '+str(int(i/10))+' sec.', end='\r')
                if self._bb:
                    self._acc_vals[0] = self._IMU.accel.x
                    self._acc_vals[1] = self._IMU.accel.y
                    self._acc_vals[2] = self._IMU.accel.z
                    imu_tem = self._IMU.temperature
                    self._bb.update(self._acc_vals, self._gyro_vals, imu_tem, m0, m1, m2, m3)
                    self._bb.show_status(self._acc_vals, self._gyro_vals, imu_tem, m0, m1, m2, m3)
                time.sleep(0.1)
                i += 1
            except Exception as e:
                print('!!! Exception: ' + str(e))
                sys.exit()
            else:
                pass
            finally:
                pass
                
        if self._bb:
            self._bb.write('    countdown: 0 sec.')


    def takeoff(self):
        self.simple_mode('    Take off..', 2000, 4500, 50)


    def shutdown(self):
        self.simple_mode('    Shutdown..', 4500, 2000, -50)
