#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.8
# Date:     2022-11-25
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
class flight_data():
    def __init__(self, b_debug=False):
        self._acc_sum_prop = [0, 15000, 0] # now, min, max
        self._b_debug = b_debug
        if b_debug:
            self._fd = open('data.txt', 'a+')

    def write(self, msg, end='\n'):
        if self._b_debug:
            print(msg, end=end)
            if self._fd:
                self._fd.write(msg+'\n')
                self._fd.flush()

    def update(self, acc_vals, gyro_vals, tem, m0, m1, m2, m3):
        acc_sum = int(acc_vals[0]*100)**2+int(acc_vals[1]*100)**2+int(acc_vals[2]*100)**2
        self._acc_sum_prop[0] = acc_sum
        if self._acc_sum_prop[1] > acc_sum:
            self._acc_sum_prop[1] = acc_sum
        if self._acc_sum_prop[2] < acc_sum:
            self._acc_sum_prop[2] = acc_sum


    def show_status(self, acc_vals, gyro_vals, tem, m0, m1, m2, m3, indent=4):
        if not self._b_debug:
            return
        msg = ''
        for i in range(indent):
            msg += ' '
        acc_keys = ['ax:',' ay:',' az:']
        for i in range(3):
            msg += acc_keys[i]
            val = int(acc_vals[i]*100)
            if val>=0:
                msg += ' '
            msg += str(val)
        gyro_keys = [' gx:',' gy:',' gz:']
        for i in range(3):
            msg += gyro_keys[i]
            val = int(gyro_vals[i])
            if val>=0:
                msg += ' '
            msg += str(val)
        msg += ' tem:'
        msg += str(int(tem))
        msg += ' m0:'
        msg += str(m0)
        msg += ' m1:'
        msg += str(m1)
        msg += ' m2:'
        msg += str(m2)
        msg += ' m3:'
        msg += str(m3)

        acc_sum_keys = [' acc sum:',' min:',' max:']
        for i in range(3):
            msg += acc_sum_keys[i]
            val = self._acc_sum_prop[i]
            if val>=0:
                msg += ' '
            msg += str(val)

        msg += '        '
        self.write(msg, end='\r')