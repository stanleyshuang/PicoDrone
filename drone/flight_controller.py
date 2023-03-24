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
from motor_ctr import motor_ctr
from moving_average import moving_average


class flight_controller():
    INIT_SPEED =    393
    TAKEOFF_SPEED = 555
    FINAL_SPEED =   661
    STABLE_SPEED =  650
    TERM_SPEED =    300
    FAST_STEP =       1
    SLOW_STEP =       1
    def __init__(self, imu, st0, st1, st2, st_matrics, 
                 esc0, esc1, esc2, esc3, 
                 motor_ctr_0, motor_ctr_1, motor_ctr_2, motor_ctr_3):
        self._IMU = imu
        i_acc_q_size = 10
        ax_q = moving_average(i_acc_q_size+1)
        ay_q = moving_average(i_acc_q_size+1)
        az_q = moving_average(i_acc_q_size+1)
        self._acc_qs = [ax_q, ay_q, az_q]

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
        self._b_arm = True

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

        if self._b_arm:
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
                    print('!!  Exception: (init) ' + str(e))
                utime.sleep_us(3000)
                # continue
            else:
                self._acc_qs[0].update_val(ax)
                self._acc_qs[1].update_val(ay)
                self._acc_qs[2].update_val(az)

            if i%7==6:
                increased_value += 30
                if self._b_arm:
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

        if self._b_arm:
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

        self._m0.i_rpm = rpm
        self._m1.i_rpm = rpm
        self._m2.i_rpm = rpm
        self._m3.i_rpm = rpm

        if self._b_arm:
            self._ESC0.value = rpm
            self._ESC1.value = rpm
            self._ESC2.value = rpm
            self._ESC3.value = rpm


    def b_stop_condition(self, stop, step):
        if step==0:
            return False
        elif step>0:
            return self._m0.i_rpm>=stop and self._m1.i_rpm>=stop and self._m2.i_rpm>=stop and self._m3.i_rpm>=stop
        else:
            return self._m0.i_rpm<=stop and self._m1.i_rpm<=stop and self._m2.i_rpm<=stop and self._m3.i_rpm<=stop


    def simple_mode(self, msg, stop, step, duration=10000000):
        begin = end = utime.ticks_ms()
        if self._bb:
            self._bb.write(4, msg)

        i = 0
        the_period = 10
        while not self.b_stop_condition(stop, step) and utime.ticks_diff(end, begin)<duration:
            end = utime.ticks_ms()
            acc_currs = [0.0, 0.0, 0.0]
            gyro_currs = [0.0, 0.0, 0.0]
            acc_sums = [0.0, 0.0, 0.0]
            imu_tem = 0.0
            pitch = roll = yaw = 0.0

            pid_x0, pid_y0 = 0.0, 0.0
            pid_x1, pid_y1 = 0.0, 0.0
            pid_x2, pid_y2 = 0.0, 0.0
            pid_x3, pid_y3 = 0.0, 0.0
            if i%the_period==0 and self._b_pid:
                ax = ay = az = gx = gy = gz = 0.0
                try:
                    ax = self._IMU.accel.x
                    ay = self._IMU.accel.y * -1.0
                    az = self._IMU.accel.z
                    gx = self._IMU.gyro.x * -1.0
                    gy = self._IMU.gyro.y * -1.0
                    gz = self._IMU.gyro.z
                    imu_tem = self._IMU.temperature
                except Exception as e:
                    if self._bb:
                        self._bb.write(1, '!!! Exception: (simple_mode) ' + str(e) + ' (' + str(utime.ticks_ms())  + ' ms)')
                    else:
                        print('!!  Exception: (simple_mode) ' + str(e) + ' (' + str(utime.ticks_ms())  + ' ms)')
                else:
                    # 取到完整陀螺儀值後，才更新成員變數與 PID。
                    pitch = motor_ctr.pitch(ax, ay, az)
                    roll = motor_ctr.roll(ax, ay, az)
                    yaw = motor_ctr.yaw(gx, gy, gz)

                    self._acc_qs[0].update_val(roll)
                    self._acc_qs[1].update_val(pitch)
                    self._acc_qs[2].update_val(az)

                    acc_currs[0] = roll
                    acc_currs[1] = pitch
                    acc_currs[2] = az
                    gyro_currs[0] = gx
                    gyro_currs[1] = gy
                    gyro_currs[2] = gz
                    acc_sums[0] = self._acc_qs[0].sum
                    acc_sums[1] = self._acc_qs[1].sum
                    acc_sums[2] = self._acc_qs[2].sum

                    pid_x0 = self._m0.f_pid_x(gyro_currs[0], acc_currs[0], acc_sums[0])
                    pid_y0 = self._m0.f_pid_y(gyro_currs[1], acc_currs[1], acc_sums[1])
                    pid_x1 = self._m1.f_pid_x(gyro_currs[0], acc_currs[0], acc_sums[0])
                    pid_y1 = self._m1.f_pid_y(gyro_currs[1], acc_currs[1], acc_sums[1])
                    pid_x2 = self._m2.f_pid_x(gyro_currs[0], acc_currs[0], acc_sums[0])
                    pid_y2 = self._m2.f_pid_y(gyro_currs[1], acc_currs[1], acc_sums[1])
                    pid_x3 = self._m3.f_pid_x(gyro_currs[0], acc_currs[0], acc_sums[0])
                    pid_y3 = self._m3.f_pid_y(gyro_currs[1], acc_currs[1], acc_sums[1])

            t_rpm0 = self._m0.i_rpm + step
            t_rpm1 = self._m1.i_rpm + step 
            t_rpm2 = self._m2.i_rpm + step
            t_rpm3 = self._m3.i_rpm + step 

            i_rpm0 = self._m0.i_rpm_bound_check(t_rpm0 + int(pid_x0 + pid_y0))
            i_rpm1 = self._m1.i_rpm_bound_check(t_rpm1 + int(pid_x1 + pid_y1))
            i_rpm2 = self._m2.i_rpm_bound_check(t_rpm2 + int(pid_x2 + pid_y2))
            i_rpm3 = self._m3.i_rpm_bound_check(t_rpm3 + int(pid_x3 + pid_y3))

            diff_rmp0 = i_rpm0 - self._m0.i_rpm
            diff_rmp1 = i_rpm1 - self._m1.i_rpm
            diff_rmp2 = i_rpm2 - self._m2.i_rpm
            diff_rmp3 = i_rpm3 - self._m3.i_rpm

            self._m0.i_rpm = i_rpm0
            self._m1.i_rpm = i_rpm1
            self._m2.i_rpm = i_rpm2
            self._m3.i_rpm = i_rpm3

            if self._b_arm:
                self._ESC0.value = i_rpm0
                self._ESC1.value = i_rpm1
                self._ESC2.value = i_rpm2
                self._ESC3.value = i_rpm3

            if i%the_period==0 and self._b_pid:
                utime.sleep_us(10000)

                t_rpm0 = self._m0.i_rpm_bound_check(t_rpm0)
                t_rpm1 = self._m1.i_rpm_bound_check(t_rpm1)
                t_rpm2 = self._m2.i_rpm_bound_check(t_rpm2)
                t_rpm3 = self._m3.i_rpm_bound_check(t_rpm3)

                self._m0.i_rpm = t_rpm0
                self._m1.i_rpm = t_rpm1
                self._m2.i_rpm = t_rpm2
                self._m3.i_rpm = t_rpm3

                if self._b_arm:
                    self._ESC0.value = t_rpm0
                    self._ESC1.value = t_rpm1
                    self._ESC2.value = t_rpm2
                    self._ESC3.value = t_rpm3

                if self._bb:
                    self._bb.show_status(4, acc_currs, gyro_currs, acc_sums, imu_tem, 
                                         i_rpm0, i_rpm1, i_rpm2, i_rpm3, 
                                         diff_rmp0, diff_rmp1, diff_rmp2, diff_rmp3,
                                         pid_x0, pid_y0, pid_x1, pid_y1, pid_x2, pid_y2, pid_x3, pid_y3)
                else:
                    utime.sleep_us(30000)
            else:
                utime.sleep_us(30000)
            i += 1

        end = utime.ticks_ms()
        diff = utime.ticks_diff(end, begin)
        msg_duration = '    duration: '+str(round(diff/1000, 2))+' sec.'
        if self._bb:
            self._bb.write(4, msg_duration)
        else:
            print(msg_duration)


    def takeoff(self):
        self.set_rpm(flight_controller.INIT_SPEED)
        self.simple_mode('    Take off..', flight_controller.TAKEOFF_SPEED, flight_controller.FAST_STEP)
        self.simple_mode('    Take off..', flight_controller.FINAL_SPEED, flight_controller.SLOW_STEP)

    def ufo_float(self):
        self.simple_mode('    UFO floating..', flight_controller.STABLE_SPEED, 0, duration=15000)

    def shutdown(self):
        self.simple_mode('    Shutdown..', flight_controller.INIT_SPEED, -1*flight_controller.SLOW_STEP)
        self.set_rpm(flight_controller.TERM_SPEED)
