#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.8
# Date:     2022-11-25
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

class flight_ctr():
    def __init__(self, st_range, m_range, m_val_cr=1.0):
        self._BASED_ACC_SUM = 0
        self._ES_ACC_SUM = 0
        self._acc_vals = [0, 0, 0]
        self._gyro_vals = [0, 0, 0]

        self._ST_RANGE = st_range # stick parameters
        st_q_size = 10
        st0_q = moving_average(st_q_size+1)
        st1_q = moving_average(st_q_size+1)
        st2_q = moving_average(st_q_size+1)
        self._st_q = [st0_q, st1_q, st2_q]

        self._M_RANGE = m_range # simonk pwm parameters
        self._1_PERMILLE = int((self._M_RANGE[1]-self._M_RANGE[0])/1000*m_val_cr)
        self._M_UNIT = 5 * self._1_PERMILLE
        self._10_M_UNIT = self._M_UNIT * 10
        self._100_M_UNIT = self._M_UNIT * 100
        self._pwm_value = m_range[0]


    @property
    def acc_vals(self):
        return self._acc_vals

    @acc_vals.setter
    def acc_vals(self, acc_vals):
        for i in range(3):
            self._acc_vals[i] = int(acc_vals[i]*100)

    @property
    def gyro_vals(self):
        return self._gyro_vals

    @gyro_vals.setter
    def gyro_vals(self, gyro_vals):
        for i in range(3):
            self._gyro_vals[i] = int(gyro_vals[i])

    @property
    def based_acc_sum(self):
        return self._BASED_ACC_SUM

    @based_acc_sum.setter
    def based_acc_sum(self, based_acc_sum):
        self._BASED_ACC_SUM = based_acc_sum

    @property
    def es_acc_sum(self):
        return self._ES_ACC_SUM

    @es_acc_sum.setter
    def es_acc_sum(self, es_acc_sum):
        self._ES_ACC_SUM = es_acc_sum

    @property
    def st_vals(self):
        st_vals[0, 0, 0]
        for i in range(3):
            st_vals = self._st_q[i].average
        return st_vals

    @st_vals.setter
    def st_vals(self, st_vals):
        for i in range(3):
            self._st_q[i].update_val(st_vals[i])

    @property
    def one_permille(self):
        return self._1_PERMILLE
    

    def joystick_2(self):
        st_2_val = self._st_q[2].average
        if st_2_val>self._ST_RANGE[2][1]:
            self._pwm_value += int(self._100_M_UNIT*((st_2_val-self._ST_RANGE[2][1])/(self._ST_RANGE[2][2]-self._ST_RANGE[2][1])))
        else:
            self._pwm_value -= int(self._100_M_UNIT*((self._ST_RANGE[2][1]-st_2_val)/(self._ST_RANGE[2][1]-self._ST_RANGE[2][0])))
        return self._pwm_value

    def fall_protect(self):
        ax = self._acc_vals[0]
        ay = self._acc_vals[1]
        az = self._acc_vals[2]
        acc_sum = ax**2 + ay**2 + az**2
        if self._BASED_ACC_SUM!=0 and acc_sum < self._BASED_ACC_SUM:
            ### 下墜時加速
            self._pwm_value += int(self._10_M_UNIT * (1.0 - acc_sum/self._BASED_ACC_SUM))
        if self._ES_ACC_SUM!=0 and acc_sum > self._ES_ACC_SUM:
            ### 沒加油門時，爆升時減速
            st_2_val = self._st_q[2].average
            if st_2_val<=self._ST_RANGE[2][1]:
                self._pwm_value += int(self._10_M_UNIT * (1.0 - acc_sum/self._ES_ACC_SUM))
        return self._pwm_value

    def left(self):
        ay = self._acc_vals[1]
        self._pwm_value -= int(self._10_M_UNIT*ay/30)
        return self._pwm_value

    def right(self):
        ay = self._acc_vals[1]
        self._pwm_value += int(self._10_M_UNIT*ay/30)
        return self._pwm_value

    def front(self):
        ax = self._acc_vals[0]
        self._pwm_value -= int(self._10_M_UNIT*ax/30)
        return self._pwm_value

    def back(self):
        ax = self._acc_vals[0]
        self._pwm_value += int(self._10_M_UNIT*ax/30)
        return self._pwm_value

    def range_protect(self):
        pwm_value = self._pwm_value
        if pwm_value>self._M_RANGE[1]:
            pwm_value = self._M_RANGE[1]
        if pwm_value<self._M_RANGE[0]:
            pwm_value = self._M_RANGE[0]
        self._pwm_value = pwm_value
        return self._pwm_value


class flight_ctr_fr(flight_ctr):
    def motor_pwn_value(self):
        self.joystick_2()
        self.right()
        self.front()
        self.fall_protect()
        self.range_protect()
        return self._pwm_value


class flight_ctr_fl(flight_ctr):
    def motor_pwn_value(self):
        self.joystick_2()
        self.left()
        self.front()
        self.fall_protect()
        self.range_protect()
        return self._pwm_value


class flight_ctr_bl(flight_ctr):
    def motor_pwn_value(self):
        self.joystick_2()
        self.left()
        self.back()
        self.fall_protect()
        self.range_protect()
        return self._pwm_value


class flight_ctr_br(flight_ctr):
    def motor_pwn_value(self):
        self.joystick_2()
        self.right()
        self.back()
        self.fall_protect()
        self.range_protect()
        return self._pwm_value


### figuring out the baseline of acc sum
def acc_sum_base(imu, bb):
    import time
    bb.write('    figuring out the baseline of acc sum..')
    ACC_BASE_SAMPLING_COUNT = 30
    acc_sum = [0, 0, 0]
    for i in range(ACC_BASE_SAMPLING_COUNT):
        acc_sum[0] += int(imu.accel.x * 100)
        acc_sum[1] += int(imu.accel.y * 100)
        acc_sum[2] += int(imu.accel.z * 100)
        time.sleep(0.1)
    acc_base = [0, 0, 0]
    acc_base[0] = int(acc_sum[0]/ACC_BASE_SAMPLING_COUNT)
    acc_base[1] = int(acc_sum[1]/ACC_BASE_SAMPLING_COUNT)
    acc_base[2] = int(acc_sum[2]/ACC_BASE_SAMPLING_COUNT)
    acc_sum_base = acc_base[0]**2 + acc_base[1]**2 + acc_base[2]**2
    bb.write('    Base G: '+str(acc_sum_base))
    return acc_sum_base


### figuring out the acc sum at the boundary of escape gravity
def acc_sum_escape_g(imu, 
                     flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
                     motor_0, motor_1, motor_2, motor_3,
                     bb):
    import time
    acc_vals = [0.0, 0.0, 0.0]
    gyro_vals = [0.0, 0.0, 0.0]
    st_vals = [0, 0, 0]
    bb.write('    figuring out the acc sum at the boundary of escape gravity..')
    # i, az, delta-az, acc_sum, delta-acc_sum
    G_TEST_COUNT = 10 # 10 - 35
    data = []
    for i in range(G_TEST_COUNT):
        prev_ax = int(acc_vals[0]*100)
        prev_ay = int(acc_vals[1]*100)
        prev_az = int(acc_vals[2]*100)
        prev_acc_sum = prev_ax**2 + prev_ay**2 + prev_az**2

        acc_vals[0], acc_vals[1], acc_vals[2] = imu.accel.x, imu.accel.y, imu.accel.z
        ax = int(acc_vals[0]*100)
        ay = int(acc_vals[1]*100)
        az = int(acc_vals[2]*100)
        acc_sum = ax**2 + ay**2 + az**2

        flight_ctr_0.acc_vals = acc_vals
        flight_ctr_1.acc_vals = acc_vals
        flight_ctr_2.acc_vals = acc_vals
        flight_ctr_3.acc_vals = acc_vals

        flight_ctr_0.gyro_vals = gyro_vals
        flight_ctr_1.gyro_vals = gyro_vals
        flight_ctr_2.gyro_vals = gyro_vals
        flight_ctr_3.gyro_vals = gyro_vals

        st_vals[0] = 5000
        st_vals[1] = 5000
        st_vals[2] = 7500
        flight_ctr_0.st_vals = st_vals
        flight_ctr_1.st_vals = st_vals
        flight_ctr_2.st_vals = st_vals
        flight_ctr_3.st_vals = st_vals

        m0 = flight_ctr_0.motor_pwn_value()
        m1 = flight_ctr_1.motor_pwn_value()
        m2 = flight_ctr_2.motor_pwn_value()
        m3 = flight_ctr_3.motor_pwn_value()

        motor_0.duty(m0)
        motor_1.duty(m1)
        motor_2.duty(m2)
        motor_3.duty(m3)

        item = {'i': i,
                'az': az,
                'delta-az': az-prev_az,
                'acc_sum': acc_sum,
                'delta-acc_sum': acc_sum-prev_acc_sum,
                'm0': m0,
                'm1': m1,
                'm2': m2,
                'm3': m3,}
        if i>0: # skip the first run, becasue the value of delta-az and delta-acc_sum are meaningless.
            data.append(item)

        time.sleep(0.1)
    nlarge_delta_acc_sum = sorted(data, key = lambda x: x['delta-acc_sum'], reverse = True)[:5]
    for item in nlarge_delta_acc_sum:
        bb.write('    '+str(item))
    bb.write('')
    for item in data:
        bb.write('    '+str(item))
    bb.write('    Escape G: '+str(nlarge_delta_acc_sum[0]['acc_sum']))
    return nlarge_delta_acc_sum[0]['acc_sum']


def shutdown(imu, 
             flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
             motor_0, motor_1, motor_2, motor_3,
             m_range_0, m_range_1, m_range_2, m_range_3,
             bb):
    import sys, time
    acc_vals = [0.0, 0.0, 0.0]
    gyro_vals = [0.0, 0.0, 0.0]
    st_vals = [0, 0, 0]
    bb.write('shutdowning PicoDrone..')
    # i, az, delta-az, acc_sum, delta-acc_sum
    SHUTDOWN_COUNT = 500
    data = []
    for i in range(SHUTDOWN_COUNT):
        prev_ax = int(acc_vals[0]*100)
        prev_ay = int(acc_vals[1]*100)
        prev_az = int(acc_vals[2]*100)
        prev_acc_sum = prev_ax**2 + prev_ay**2 + prev_az**2

        acc_vals[0], acc_vals[1], acc_vals[2] = imu.accel.x, imu.accel.y, imu.accel.z
        ax = int(acc_vals[0]*100)
        ay = int(acc_vals[1]*100)
        az = int(acc_vals[2]*100)
        acc_sum = ax**2 + ay**2 + az**2

        flight_ctr_0.acc_vals = acc_vals
        flight_ctr_1.acc_vals = acc_vals
        flight_ctr_2.acc_vals = acc_vals
        flight_ctr_3.acc_vals = acc_vals

        flight_ctr_0.gyro_vals = gyro_vals
        flight_ctr_1.gyro_vals = gyro_vals
        flight_ctr_2.gyro_vals = gyro_vals
        flight_ctr_3.gyro_vals = gyro_vals

        st_vals[0] = 5000
        st_vals[1] = 5000
        st_vals[2] = 2500
        flight_ctr_0.st_vals = st_vals
        flight_ctr_1.st_vals = st_vals
        flight_ctr_2.st_vals = st_vals
        flight_ctr_3.st_vals = st_vals

        m0 = flight_ctr_0.motor_pwn_value()
        m1 = flight_ctr_1.motor_pwn_value()
        m2 = flight_ctr_2.motor_pwn_value()
        m3 = flight_ctr_3.motor_pwn_value()

        motor_0.duty(m0)
        motor_1.duty(m1)
        motor_2.duty(m2)
        motor_3.duty(m3)

        item = {'i': i,
                'az': az,
                'delta-az': az-prev_az,
                'acc_sum': acc_sum,
                'delta-acc_sum': acc_sum-prev_acc_sum,
                'm0': m0,
                'm1': m1,
                'm2': m2,
                'm3': m3,}
        if i>0: # skip the first run, becasue the value of delta-az and delta-acc_sum are meaningless.
            data.append(item)
        
        if m0<=m_range_0[0] and m1<=m_range_1[0] and m2<=m_range_2[0] and m3<=m_range_3[0]:
            bb.write('    shutdown completed, i='+str(i))
            break

        time.sleep(0.1)    
    motor_0.duty(m_range_0[2])
    motor_1.duty(m_range_1[2])
    motor_2.duty(m_range_2[2])
    motor_3.duty(m_range_3[2])

    nsmall_delta_acc_sum = sorted(data, key = lambda x: x['delta-acc_sum'])[:5]
    for item in nsmall_delta_acc_sum:
        bb.write('    '+str(item))
    bb.write('')
    for item in data:
        bb.write('    '+str(item))
    if len(nsmall_delta_acc_sum)>0:
        bb.write('    Shuting down G: '+str(nsmall_delta_acc_sum[0]['acc_sum']))
    sys.exit()


### entering the main loop
def main_loop(imu, st0, st1, st2, 
              flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
              motor_0, motor_1, motor_2, motor_3,
              bb):
    import time

    ST_PROB_FREQ = 10
    MOTOR_ADJ_FREQ = 10
    tickcount = 0

    acc_vals = [0.0, 0.0, 0.0]
    gyro_vals = [0.0, 0.0, 0.0]
    st_vals = [0, 0, 0]
    while True:
        if tickcount%(ST_PROB_FREQ/MOTOR_ADJ_FREQ)==0:
            acc_vals[0] = imu.accel.x
            acc_vals[1] = imu.accel.y
            acc_vals[2] = imu.accel.z
            gyro_vals[0] = imu.gyro.x
            gyro_vals[1] = imu.gyro.y
            gyro_vals[2] = imu.gyro.z
            imu_tem = imu.temperature

            flight_ctr_0.acc_vals = acc_vals
            flight_ctr_1.acc_vals = acc_vals
            flight_ctr_2.acc_vals = acc_vals
            flight_ctr_3.acc_vals = acc_vals

            flight_ctr_0.gyro_vals = gyro_vals
            flight_ctr_1.gyro_vals = gyro_vals
            flight_ctr_2.gyro_vals = gyro_vals
            flight_ctr_3.gyro_vals = gyro_vals

        st_vals[0] = st0.abs_scale()
        st_vals[1] = st1.abs_scale()
        st_vals[2] = st2.abs_scale()
        
        flight_ctr_0.st_vals = st_vals
        flight_ctr_1.st_vals = st_vals
        flight_ctr_2.st_vals = st_vals
        flight_ctr_3.st_vals = st_vals

        if tickcount%(ST_PROB_FREQ/MOTOR_ADJ_FREQ)==0:
            m0 = flight_ctr_0.motor_pwn_value()
            m1 = flight_ctr_1.motor_pwn_value()
            m2 = flight_ctr_2.motor_pwn_value()
            m3 = flight_ctr_3.motor_pwn_value()

        if tickcount%(ST_PROB_FREQ/MOTOR_ADJ_FREQ)==0:
            motor_0.duty(m0)
            motor_1.duty(m1)
            motor_2.duty(m2)
            motor_3.duty(m3)

        bb.update(acc_vals, gyro_vals, imu_tem, m0, m1, m2, m3)
        bb.show_status(acc_vals, gyro_vals, imu_tem, m0, m1, m2, m3)

        time.sleep(1.0/ST_PROB_FREQ)
        tickcount += 1
