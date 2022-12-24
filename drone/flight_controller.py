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
        i_acc_q_size = 3
        ax_q = moving_average(i_acc_q_size+1)
        ay_q = moving_average(i_acc_q_size+1)
        az_q = moving_average(i_acc_q_size+1)
        self._i_acc_vals = [ax_q, ay_q, az_q]
        self._i_gyro_vals = [0, 0, 0]

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
            i_acc_sum[0] += int(self._IMU.accel.x * 100)
            i_acc_sum[1] += int(self._IMU.accel.y * 100)
            i_acc_sum[2] += int(self._IMU.accel.z * 100)
            self._i_acc_vals[0].update_val(self._IMU.accel.x)
            self._i_acc_vals[1].update_val(self._IMU.accel.y)
            self._i_acc_vals[2].update_val(self._IMU.accel.z)
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
        i_acc_vals = [0, 0, 0]
        prob = 1
        step = int(step/prob)
        i = 0
        for rpm in range(start, stop, step):
            try:
                self._i_acc_vals[0].update_val(self._IMU.accel.x)
                self._i_acc_vals[1].update_val(self._IMU.accel.y)
                self._i_acc_vals[2].update_val(self._IMU.accel.z)
                if i%prob==0:
                    rpm0 = rpm + self._m0.i_pid_x(self._i_acc_vals[0].average) + self._m0.i_pid_y(self._i_acc_vals[1].average)
                    rpm1 = rpm + self._m1.i_pid_x(self._i_acc_vals[0].average) + self._m1.i_pid_y(self._i_acc_vals[1].average)
                    rpm2 = rpm + self._m2.i_pid_x(self._i_acc_vals[0].average) + self._m2.i_pid_y(self._i_acc_vals[1].average)
                    rpm3 = rpm + self._m3.i_pid_x(self._i_acc_vals[0].average) + self._m3.i_pid_y(self._i_acc_vals[1].average)
                    i_m0 = self._m0.i_rpm2duty(int(rpm * self._m0.f_conversion_rate))
                    i_m1 = self._m1.i_rpm2duty(int(rpm * self._m1.f_conversion_rate))
                    i_m2 = self._m2.i_rpm2duty(int(rpm * self._m2.f_conversion_rate))
                    i_m3 = self._m3.i_rpm2duty(int(rpm * self._m3.f_conversion_rate))
                    self._ESC0.duty = i_m0
                    self._ESC1.duty = i_m1
                    self._ESC2.duty = i_m2
                    self._ESC3.duty = i_m3
                    if i%(10*prob)==0 and self._bb:
                        self._bb.write('    countdown: '+str(int(i/(10*prob)))+' sec.', end='\r')
                    if self._bb:
                        i_acc_vals[0] = self._i_acc_vals[0].average
                        i_acc_vals[1] = self._i_acc_vals[1].average
                        i_acc_vals[2] = self._i_acc_vals[2].average
                        '''
                        i_acc_vals[0] = self._IMU.accel.x
                        i_acc_vals[1] = self._IMU.accel.y
                        i_acc_vals[2] = self._IMU.accel.z
                        '''
                        imu_tem = self._IMU.temperature
                        self._bb.update(i_acc_vals, self._i_gyro_vals, imu_tem, i_m0, i_m1, i_m2, i_m3)
                        self._bb.show_status(i_acc_vals, self._i_gyro_vals, imu_tem, i_m0, i_m1, i_m2, i_m3)
                time.sleep(0.02/prob) # workload = (0.1 - 0.02)
                i += 1
            except Exception as e:
                print('!!! Exception: ' + str(e))
                sys.exit()
            else:
                pass
            finally:
                pass
        if self._bb:
            self._bb.write('    countdown: '+str(int(i/(10*prob)))+' sec.', end='\r')
        end = utime.ticks_ms()
        diff = utime.ticks_diff(end, begin)
        if self._bb:
            self._bb.write('    duration: '+str(round(diff/1000, 2))+' sec.')



    def takeoff(self):
        self.simple_mode('    Take off..', 2000, 4640, 50)


    def shutdown(self):
        self.simple_mode('    Shutdown..', 4640, 2000, -50)
