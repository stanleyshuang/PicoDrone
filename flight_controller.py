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
    def __init__(self, name, st_range, motor, debug_obj=None, pwr_cr=1):
        self.name = name
        self.bb = debug_obj
        self.pwr_cr = pwr_cr
        self.debug_show_detail = False

        self._BASED_ACC_SUM = 0
        self._ES_ACC_SUM = 0
        self._acc_vals = [0, 0, 0]
        self._gyro_vals = [0, 0, 0]

        self.ST_RANGE = st_range # stick parameters
        st_q_size = 10
        st0_q = moving_average(st_q_size+1)
        st1_q = moving_average(st_q_size+1)
        st2_q = moving_average(st_q_size+1)
        self._st_q = [st0_q, st1_q, st2_q]

        self._MOTOR = motor # SimonK ESC pwm duty parameters
        
        self._M_UNIT_BAL = 0 # self.pwr_cr/2
        self._M_UNIT = self.pwr_cr
        if self.bb:
            self.bb.write('    '+self.name+'.'+'_M_UNIT:          '+str(self._M_UNIT))
        self._10_M_UNIT = self._M_UNIT * 10
        self._100_M_UNIT = self._M_UNIT * 100
        self._pwm_value = motor.min_duty


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
        st_vals = [0, 0, 0]
        for i in range(3):
            st_vals[i] = self._st_q[i].average
        return st_vals

    @st_vals.setter
    def st_vals(self, st_vals):
        for i in range(3):
            self._st_q[i].update_val(st_vals[i])

    @property
    def power_conversion_rate(self):
        return self.pwr_cr
    

    def joystick_2(self):
        st_2_val = self._st_q[2].average
        if st_2_val>self.ST_RANGE[2][1]:
            delta = int(self._100_M_UNIT*((st_2_val-self.ST_RANGE[2][1])/(self.ST_RANGE[2][2]-self.ST_RANGE[2][1])))
            self._pwm_value += delta
            if self.bb and self.debug_show_detail:
                if delta>0:
                    sign = '+'
                else:
                    sign = ''
                self.bb.write('    '+self.name+'.'+'joystick_2:    '+str(self._pwm_value)+', '+str(sign)+str(delta))
        else:
            delta = int(-1*self._100_M_UNIT*((self.ST_RANGE[2][1]-st_2_val)/(self.ST_RANGE[2][1]-self.ST_RANGE[2][0])))
            self._pwm_value += delta
            if self.bb and self.debug_show_detail:
                if delta>0:
                    sign = '+'
                else:
                    sign = ''
                self.bb.write('    '+self.name+'.'+'joystick_2:    '+str(self._pwm_value)+', '+str(sign)+str(delta))
        return self._pwm_value

    def fall_protect(self):
        ax = self._acc_vals[0]
        ay = self._acc_vals[1]
        az = self._acc_vals[2]
        acc_sum = ax**2 + ay**2 + az**2
        if self._BASED_ACC_SUM!=0 and acc_sum < self._BASED_ACC_SUM:
            ### 下墜時加速
            delta = int(self._10_M_UNIT * (1.0 - acc_sum/self._BASED_ACC_SUM))
            self._pwm_value += delta
            if self.bb and self.debug_show_detail:
                if delta>0:
                    sign = '+'
                else:
                    sign = ''
                self.bb.write('    '+self.name+'.'+'fall_protect:  '+str(self._pwm_value)+', '+str(sign)+str(delta))
        if self._ES_ACC_SUM!=0 and acc_sum > self._ES_ACC_SUM:
            ### 沒加油門時，爆升時減速
            st_2_val = self._st_q[2].average
            if st_2_val<=self.ST_RANGE[2][1]:
                delta = int(self._10_M_UNIT * (1.0 - acc_sum/self._ES_ACC_SUM))
                self._pwm_value += delta
                if self.bb and self.debug_show_detail:
                    if delta>0:
                        sign = '+'
                    else:
                        sign = ''
                    self.bb.write('    '+self.name+'.'+'fall_protect:  '+str(self._pwm_value)+', '+str(sign)+str(delta))
        return self._pwm_value

    def left(self):
        ay = self._acc_vals[1]
        delta = int(-1*self._M_UNIT_BAL*ay)
        self._pwm_value += delta
        if self.bb and self.debug_show_detail:
            if delta>0:
                sign = '+'
            else:
                sign = ''
            self.bb.write('    '+self.name+'.'+'left:          '+str(self._pwm_value)+', '+str(sign)+str(delta))
        return self._pwm_value

    def right(self):
        ay = self._acc_vals[1]
        delta = int(self._M_UNIT_BAL*ay)
        self._pwm_value += delta
        if self.bb and self.debug_show_detail:
            if delta>0:
                sign = '+'
            else:
                sign = ''
            self.bb.write('    '+self.name+'.'+'right:         '+str(self._pwm_value)+', '+str(sign)+str(delta))
        return self._pwm_value

    def front(self):
        ax = self._acc_vals[0]
        delta = int(-1*self._M_UNIT_BAL*ax)
        self._pwm_value += delta
        if self.bb and self.debug_show_detail:
            if delta>0:
                sign = '+'
            else:
                sign = ''
            self.bb.write('    '+self.name+'.'+'front:         '+str(self._pwm_value)+', '+str(sign)+str(delta))
        return self._pwm_value

    def back(self):
        ax = self._acc_vals[0]
        delta = int(self._M_UNIT_BAL*ax)
        self._pwm_value += delta
        if self.bb and self.debug_show_detail:
            if delta>0:
                sign = '+'
            else:
                sign = ''
            self.bb.write('    '+self.name+'.'+'back:          '+str(self._pwm_value)+', '+str(sign)+str(delta))
        return self._pwm_value

    def range_protect(self):
        pwm_value = self._pwm_value
        if pwm_value>self._MOTOR.balance_duty:
            pwm_value = self._MOTOR.balance_duty
            if self.bb and self.debug_show_detail:
                self.bb.write('    '+self.name+'.'+'range_protect: '+str(pwm_value))
        if pwm_value<self._MOTOR.min_duty:
            pwm_value = self._MOTOR.min_duty
            if self.bb and self.debug_show_detail:
                self.bb.write('    '+self.name+'.'+'range_protect: '+str(pwm_value))
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

# format debug message
def dump_flight_data(debug, flight_data):
    for item in flight_data:
        msg = '{'
        for key in sorted(item.keys()):
            msg += " '"+key+"':"+str(item[key])+","
        msg += '}'
        if debug:
            debug.write('    '+msg)


### figuring out the baseline of acc sum
def acc_sum_base(imu, bb=None):
    import time
    if bb:
        bb.write('    figuring out the baseline of acc sum..')
    ACC_BASE_SAMPLING_COUNT = 30
    acc_sum = [0, 0, 0]
    for i in range(ACC_BASE_SAMPLING_COUNT):
        acc_sum[0] += int(imu.accel.x * 100)
        acc_sum[1] += int(imu.accel.y * 100)
        acc_sum[2] += int(imu.accel.z * 100)
        if i%10==0 and bb:
            bb.write('    countdown: '+str(int((ACC_BASE_SAMPLING_COUNT-i)/10))+' sec.', end='\r')
        time.sleep(0.1)
    if bb:
        bb.write('    countdown: 0 sec.')
    acc_base = [0, 0, 0]
    acc_base[0] = int(acc_sum[0]/ACC_BASE_SAMPLING_COUNT)
    acc_base[1] = int(acc_sum[1]/ACC_BASE_SAMPLING_COUNT)
    acc_base[2] = int(acc_sum[2]/ACC_BASE_SAMPLING_COUNT)
    acc_sum_base = acc_base[0]**2 + acc_base[1]**2 + acc_base[2]**2
    if bb:
        bb.write('    Base G: '+str(acc_sum_base))
    return acc_sum_base


### The UFO Floating
def ufo_floating_set_flight_ctr_st2_val(st_vals, motor, flight_ctr):
    if motor.duty<motor.balance_duty and motor.balance_duty-motor.duty>=flight_ctr.power_conversion_rate*100:
        st_vals[2] = flight_ctr.ST_RANGE[2][2]
        flight_ctr.st_vals = st_vals
    elif motor.duty<motor.balance_duty and motor.balance_duty-motor.duty<flight_ctr.power_conversion_rate*100:
        st_vals[2] = flight_ctr.ST_RANGE[2][1]+(flight_ctr.ST_RANGE[2][2]-flight_ctr.ST_RANGE[2][1])*((motor.balance_duty-motor.duty)/flight_ctr.power_conversion_rate*100)
        flight_ctr.st_vals = st_vals
    else:
        st_vals[2] = flight_ctr.ST_RANGE[2][1]
        flight_ctr.st_vals = st_vals

def ufo_float(imu, 
              flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
              motor_0, motor_1, motor_2, motor_3,
              bb=None):
    import sys, time
    acc_vals = [0.0, 0.0, 0.0]
    gyro_vals = [0.0, 0.0, 0.0]
    st_vals = [5000, 5000, 0]
    if bb:
        bb.write('    make UFO floating..')
    # i, az, delta-az, acc_sum, delta-acc_sum
    data = []
    i = 0
    while (motor_0.duty<motor_0.balance_duty or 
           motor_1.duty<motor_1.balance_duty or 
           motor_2.duty<motor_2.balance_duty or 
           motor_3.duty<motor_3.balance_duty):
        try:
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

            ufo_floating_set_flight_ctr_st2_val(st_vals, motor_0, flight_ctr_0)
            ufo_floating_set_flight_ctr_st2_val(st_vals, motor_1, flight_ctr_1)
            ufo_floating_set_flight_ctr_st2_val(st_vals, motor_2, flight_ctr_2)
            ufo_floating_set_flight_ctr_st2_val(st_vals, motor_3, flight_ctr_3)

            m0 = flight_ctr_0.motor_pwn_value()
            m1 = flight_ctr_1.motor_pwn_value()
            m2 = flight_ctr_2.motor_pwn_value()
            m3 = flight_ctr_3.motor_pwn_value()

            motor_0.duty = m0
            motor_1.duty = m1
            motor_2.duty = m2
            motor_3.duty = m3

            item = {'i': i,
                    'ax': ax,
                    'ay': ay,
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
            if i%10==0 and bb:
                bb.write('    countdown: '+str(int(i/10))+' sec.', end='\r')
            time.sleep(0.05)
            i += 1
        except Exception as e:
            print('!!! Exception: ' + str(e))
            sys.exit()
        else:
            pass
        finally:
            pass
            
    if bb:
        bb.write('    countdown: 0 sec.')
    nlarge_delta_acc_sum = sorted(data, key = lambda x: x['delta-acc_sum'], reverse = True)[:5]
    if bb and len(nlarge_delta_acc_sum)>0:
        bb.write('    Escape G: '+str(nlarge_delta_acc_sum[0]['acc_sum']))

    dump_flight_data(bb, nlarge_delta_acc_sum)
    if bb:
        bb.write('')
    dump_flight_data(bb, data)
    if len(nlarge_delta_acc_sum)>0:
        return nlarge_delta_acc_sum[0]['acc_sum']
    return 0


def shutdown(imu, 
             flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
             motor_0, motor_1, motor_2, motor_3,
             bb=None):
    import sys, time
    acc_vals = [0.0, 0.0, 0.0]
    gyro_vals = [0.0, 0.0, 0.0]
    st_vals = [0, 0, 0]
    if bb:
        bb.write('shuttdowning PicoDrone..')
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

        motor_0.duty = m0
        motor_1.duty = m1
        motor_2.duty = m2
        motor_3.duty = m3

        item = {'i': i,
                'ax': ax,
                'ay': ay,
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
        
        if m0<=motor_0.min_duty and m1<=motor_1.min_duty and m2<=motor_2.min_duty and m3<=motor_3.min_duty:
            if bb:
                bb.write('    shutdown completed, i='+str(i))
            break

        if i%10==0 and bb:
            bb.write('    '+str(int(i/10))+' sec.') # , end='\r')
        time.sleep(0.1)    
    motor_0.duty = motor_0.init_duty
    motor_1.duty = motor_1.init_duty
    motor_2.duty = motor_2.init_duty
    motor_3.duty = motor_3.init_duty

    nsmall_delta_acc_sum = sorted(data, key = lambda x: x['delta-acc_sum'])[:5]

    dump_flight_data(bb, nsmall_delta_acc_sum)
    if bb:
        bb.write('')
    dump_flight_data(bb, data)

    if len(nsmall_delta_acc_sum)>0 and bb:
        bb.write('    Shutting down G: '+str(nsmall_delta_acc_sum[0]['acc_sum']))
    sys.exit()


### entering the main loop
def main_loop(imu, st0, st1, st2,
              flight_ctr_0, flight_ctr_1, flight_ctr_2, flight_ctr_3, 
              motor_0, motor_1, motor_2, motor_3,
              bb=None, sec=None,
              st0_val=0, st1_val=0, st2_val=0):
    import time
    if bb and sec!=None:
        bb.write('entering the main loop. period: '+str(sec)+' sec.')
    elif bb:
        bb.write('entering the main loop')

    ST_PROB_FREQ = 20
    MOTOR_ADJ_FREQ = 20
    tickcount = 0

    acc_vals = [0.0, 0.0, 0.0]
    gyro_vals = [0.0, 0.0, 0.0]
    st_vals = [0, 0, 0]
    st_vals[0], st_vals[1], st_vals[2] = st0_val, st1_val, st2_val
    if sec==None:
        tickcount_limit = 0
    else:
        tickcount_limit = int(sec*ST_PROB_FREQ)
    while sec==None or tickcount<tickcount_limit:
        try:
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
            '''
            st_vals[0] = st0.abs_scale()
            st_vals[1] = st1.abs_scale()
            st_vals[2] = st2.abs_scale()
            '''
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
                motor_0.duty = m0
                motor_1.duty = m1
                motor_2.duty = m2
                motor_3.duty = m3

            if bb:
                bb.update(acc_vals, gyro_vals, imu_tem, m0, m1, m2, m3)
                bb.show_status(acc_vals, gyro_vals, imu_tem, m0, m1, m2, m3)

            time.sleep(1.0/ST_PROB_FREQ)
            
            if sec and tickcount%ST_PROB_FREQ==0 and bb:
                bb.write('    '+str(int(tickcount/ST_PROB_FREQ))+' sec.') # , end='\r')
            tickcount += 1
        except Exception as e:
            print('!!! Exception: ' + str(e))
        else:
            pass
        finally:
            pass