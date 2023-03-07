#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.81
# Date:     2022-12-20
#
# SimonK ESC initialization and control
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

import time
from machine import PWM

class ESC():
    def __init__(self, pin, freq=50, duty=0):
        self._pwm = PWM(pin)
        self._pwm.freq(freq)
        self._duty = duty
        self._pwm.duty_u16(self._duty)

    @property
    def duty(self):
        return self._duty
    
    @duty.setter
    def duty(self, duty):
        self._duty = duty
        self._pwm.duty_u16(self._duty)


def test_escs():
    from machine import Pin
    esc0 = ESC(Pin(6))
    esc1 = ESC(Pin(7), freq=50)
    esc2 = ESC(Pin(8))
    esc3 = ESC(Pin(9), freq=50)

    duties = [3000, 3300, 3600, 3780]
    for duty in duties:
        m0 = m1 = m2 = m3 = duty
        print(m0, m1, m2, m3)
        esc0.duty = m0
        esc1.duty = m1
        esc2.duty = m2
        esc3.duty = m3
        time.sleep(0.2)

    while True:
        the_input = input("輸入馬達 PWM Duty Cycle: (例如: a3780, 值域：0 - 65535)")
        if len(the_input)>1:
            duty = int(the_input[1:])
            if the_input[0] == 'a':
                m0 = int(duty)
            elif the_input[0] == 'b':
                m1 = int(duty)
            elif the_input[0] == 'c':
                m2 = int(duty)
            elif the_input[0] == 'd':
                m3 = int(duty)
        print(m0, m1, m2, m3)
        esc0.duty = m0
        esc1.duty = m1
        esc2.duty = m2
        esc3.duty = m3
        time.sleep(0.2)


if __name__=='__main__':
    test_escs()
