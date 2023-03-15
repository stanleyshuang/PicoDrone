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
import sys, utime
from moving_average import moving_average


class flight_controller():
    INIT_SPEED =    393
    TAKEOFF_SPEED = 555
    FINAL_SPEED =   601
    THIRD_SPEED =   590
    FAST_STEP =       8
    SLOW_STEP =       1
    FIXED_STEP =      1
    def __init__(self, imu, st0, st1, st2, st_matrics, 
                 esc0, esc1, esc2, esc3, 
                 motor_ctr_0, motor_ctr_1, motor_ctr_2, motor_ctr_3):
        self._IMU = imu
        self._acc_currs = [0.0, 0.0, 0.0]
        i_acc_q_size = 10
        ax_q = moving_average(i_acc_q_size+1)
        ay_q = moving_average(i_acc_q_size+1)
        az_q = moving_average(i_acc_q_size+1)
        self._acc_qs = [ax_q, ay_q, az_q]
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

        self._bb = None
        self._b_pid = True

    @property
    def debug(self):
        return self._bb

    @debug.setter
    def debug(self, debug_obj):
        self._bb = debug_obj

    @property
    def b_pid(self):
        return self._b_pid

    @b_pid.setter
    def b_pid(self, b_pid):
        self._b_pid = b_pid
    
    def init(self):
        begin = utime.ticks_ms()
        if self._bb:
            self._bb.write(4, '    init..')

        self._ESC0.value = self._m0.init_value
        self._ESC1.value = self._m1.init_value
        self._ESC2.value = self._m2.init_value
        self._ESC3.value = self._m3.init_value

        increased_value = 0
        SAMPLING_COUNT = 20
        for i in range(SAMPLING_COUNT):
            try:
                ax = self._IMU.accel.x
                ay = self._IMU.accel.y
                az = self._IMU.accel.z
            except Exception as e:
                if self._bb:
                    self._bb.write(1, '!!! Exception: (init) ' + str(e))
                else:
                    print('!!! Exception: (init) ' + str(e))
                utime.sleep_us(3000)
                continue

            self._acc_qs[0].update_val(ax)
            self._acc_qs[1].update_val(ay)
            self._acc_qs[2].update_val(az)

            if i%7==6:
                increased_value += 30
                self._ESC0.value = self._m0.init_value + increased_value
                self._ESC1.value = self._m1.init_value + increased_value
                self._ESC2.value = self._m2.init_value + increased_value
                self._ESC3.value = self._m3.init_value + increased_value

            if i%10==0:
                if self._bb:
                    self._bb.write(4, '    countdown: '+str(int((SAMPLING_COUNT-i)/10))+' sec.', end='\r')
                else:
                    print('    countdown: '+str(int((SAMPLING_COUNT-i)/10))+' sec.', end='\r')

            utime.sleep_us(90000)

        if self._bb:
            self._bb.write(4, '    countdown: 0 sec.')

        self._ESC0.value = self._m0.min_value
        self._ESC1.value = self._m1.min_value
        self._ESC2.value = self._m2.min_value
        self._ESC3.value = self._m3.min_value
 
        end = utime.ticks_ms()
        diff = utime.ticks_diff(end, begin)
        msg_duration = '    duration: '+str(round(diff/1000, 2))+' sec.'
        if self._bb:
            self._bb.write(4, msg_duration)
        else:
            print(msg_duration)


    def set_rpm(self, rpm):
        if self._bb:
            self._bb.write(4, '    set RPM to '+str(rpm))

        self._m0.rpm = rpm
        self._m1.rpm = rpm
        self._m2.rpm = rpm
        self._m3.rpm = rpm

        self._ESC0.value = rpm
        self._ESC1.value = rpm
        self._ESC2.value = rpm
        self._ESC3.value = rpm


    def b_stop_condition(self, stop, step):
        if step>=0:
            return self._m0.rpm>=stop and self._m1.rpm>=stop and self._m2.rpm>=stop and self._m3.rpm>=stop
        else:
            return self._m0.rpm<=stop and self._m1.rpm<=stop and self._m2.rpm<=stop and self._m3.rpm<=stop


    def simple_mode(self, msg, stop, step):
        begin = utime.ticks_ms()
        if self._bb:
            self._bb.write(4, msg)

        acc_sums = [0.0, 0.0, 0.0]
        gyro_currs = [0.0, 0.0, 0.0]
        i = 0
        pid_x0, pid_y0 = 0, 0
        pid_x1, pid_y1 = 0, 0
        pid_x2, pid_y2 = 0, 0
        pid_x3, pid_y3 = 0, 0
        imu_tem = 0.0

        while not self.b_stop_condition(stop, step):
            try:
                self._acc_currs[0] = self._IMU.accel.x
                self._acc_currs[1] = self._IMU.accel.y * -1.0
                self._acc_currs[2] = self._IMU.accel.z
                self._gyro_currs[0] = self._IMU.gyro.x * -1.0
                self._gyro_currs[1] = self._IMU.gyro.y * -1.0
                self._gyro_currs[2] = self._IMU.gyro.z
                imu_tem = self._IMU.temperature
            except Exception as e:
                if self._bb:
                    self._bb.write(1, '!!! Exception: (simple_mode) ' + str(e))
                else:
                    print('!!! Exception: (simple_mode) ' + str(e))
                utime.sleep_us(30000)
                continue

            self._acc_qs[0].update_val(self._acc_currs[0])
            self._acc_qs[1].update_val(self._acc_currs[1])
            self._acc_qs[2].update_val(self._acc_currs[2])

            acc_sums[0] = self._acc_qs[0].sum
            acc_sums[1] = self._acc_qs[1].sum
            acc_sums[2] = self._acc_qs[2].sum

            gyro_currs[0] = self._gyro_currs[0]
            gyro_currs[1] = self._gyro_currs[1]
            gyro_currs[2] = self._gyro_currs[2]

            if self._b_pid:
                pid_x0 = self._m0.f_pid_x(gyro_currs[0], self._acc_currs[1], acc_sums[1])
                pid_y0 = self._m0.f_pid_y(gyro_currs[1], self._acc_currs[0], acc_sums[0])
                pid_x1 = self._m1.f_pid_x(gyro_currs[0], self._acc_currs[1], acc_sums[1])
                pid_y1 = self._m1.f_pid_y(gyro_currs[1], self._acc_currs[0], acc_sums[0])
                pid_x2 = self._m2.f_pid_x(gyro_currs[0], self._acc_currs[1], acc_sums[1])
                pid_y2 = self._m2.f_pid_y(gyro_currs[1], self._acc_currs[0], acc_sums[0])
                pid_x3 = self._m3.f_pid_x(gyro_currs[0], self._acc_currs[1], acc_sums[1])
                pid_y3 = self._m3.f_pid_y(gyro_currs[1], self._acc_currs[0], acc_sums[0])

            i_rpm0 = self._m0.i_rpm_bound_check(self._m0.rpm + step + pid_x0 + pid_y0)
            i_rpm1 = self._m1.i_rpm_bound_check(self._m1.rpm + step + pid_x1 + pid_y1)
            i_rpm2 = self._m2.i_rpm_bound_check(self._m2.rpm + step + pid_x2 + pid_y2)
            i_rpm3 = self._m3.i_rpm_bound_check(self._m3.rpm + step + pid_x3 + pid_y3)

            diff_rmp0 = i_rpm0 - self._m0.rpm
            diff_rmp1 = i_rpm1 - self._m1.rpm
            diff_rmp2 = i_rpm2 - self._m2.rpm
            diff_rmp3 = i_rpm3 - self._m3.rpm

            self._m0.rpm = i_rpm0
            self._m1.rpm = i_rpm1
            self._m2.rpm = i_rpm2
            self._m3.rpm = i_rpm3

            self._ESC0.value = i_rpm0
            self._ESC1.value = i_rpm1
            self._ESC2.value = i_rpm2
            self._ESC3.value = i_rpm3

            if i%10==0:
                if self._bb:
                    self._bb.write(4, '    countdown: '+str(int(i/10))+' sec.', end='\r')
                else:
                    print('    countdown: '+str(int(i/10))+' sec.', end='\r')

            if self._bb:
                self._bb.show_status(4, self._acc_currs, gyro_currs, acc_sums, imu_tem, 
                                     i_rpm0, i_rpm1, i_rpm2, i_rpm3, 
                                     diff_rmp0, diff_rmp1, diff_rmp2, diff_rmp3,
                                     pid_x0, pid_y0, pid_x1, pid_y1, pid_x2, pid_y2, pid_x3, pid_y3)
            # utime.sleep_us(1000) # workload = (0.1 - 0.02)
            i += 1
        if self._bb:
            self._bb.write(4, '    countdown: '+str(int(i/(10)))+' sec.', end='\r')
        else:
            print('    countdown: '+str(int(i/(10)))+' sec.', end='\r')

        end = utime.ticks_ms()
        diff = utime.ticks_diff(end, begin)
        msg_duration = '    duration: '+str(round(diff/1000, 2))+' sec.'
        if self._bb:
            self._bb.write(4, msg_duration)
        else:
            print(msg_duration)


    def takeoff(self):
        self.set_rpm(flight_controller.INIT_SPEED)
        # self.simple_mode('    Take off..', flight_controller.TAKEOFF_SPEED, flight_controller.FAST_STEP)
        # self.simple_mode('    Take off..', flight_controller.FINAL_SPEED, flight_controller.SLOW_STEP)
        self.simple_mode('    Take off..', flight_controller.FINAL_SPEED, flight_controller.SLOW_STEP)

    def ufo_float(self):
        self.simple_mode('    UFO floating..', flight_controller.THIRD_SPEED, -1*flight_controller.FIXED_STEP)

    def shutdown(self):
        self.simple_mode('    Shutdown..', flight_controller.INIT_SPEED, -1*flight_controller.SLOW_STEP)
        self.set_rpm(0)
