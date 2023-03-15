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
import machine
import utime

class ESC():
    def __init__(self, pin, freq=50):
        self._pwm = machine.PWM(pin)
        self._pwm.freq(freq)
        self._value = 0
        self._pwm.duty_u16(0)

    @property
    def value(self):
        return self._value / 10
    
    @value.setter
    def value(self, value):
        self._value = value * 10
        self._pwm.duty_u16(self._value)


class DSHOT:
    def __init__(self, pin, freq=24000):
        self._pin = pin
        self._pin.value(0)
        self._value = 0
        self._freq = freq
        self._pwm = machine.PWM(self._pin)
        self._pwm.freq(freq)
        self._pwm.duty_u16(0)

    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, value):
        self._value = value
        self.write(value)
        
    def __del__(self):
        self._pwm.deinit()
        
    def write(self, value):
        """Write a Dshot value (0-2047) to the ESC."""
        # DShot 協議使用一個 11 位的 CRC 校驗和。首先計算出校驗和
        crc = 0
        for i in range(2, 11):
            if ((value >> i) & 1):
                crc ^= i

        # 將 PWM 占空比值轉換為 DShot 協議所需的 16 位數值
        self._pwm.duty_u16(((value & 0xFFF) << 5) | (crc & 0x7))
        
    def stop(self):
        self._pwm.duty_u16(0)
        self._pwm.deinit()
        self._pin.value(0)

    @staticmethod
    def zfill(s, width, fillchar='0'):
        """Return the string left-filled with zeros or the specified fill character."""
        if len(s) >= width:
            return s
        else:
            return (fillchar * (width - len(s))) + s


def test_escs():
    from machine import Pin
    esc0 = ESC(Pin(6))
    esc1 = ESC(Pin(7))
    esc2 = ESC(Pin(8))
    esc3 = ESC(Pin(9))

    values = [300, 330, 360, 378]
    for value in values:
        m0 = m1 = m2 = m3 = value
        print(m0, m1, m2, m3)
        esc0.value = m0
        esc1.value = m1
        esc2.value = m2
        esc3.value = m3
        utime.sleep_us(200000)

    while True:
        the_input = input("輸入馬達 PWM Duty Cycle: (例如: a378, 值域：0 - 6553)")
        if len(the_input)>1:
            value = int(the_input[1:])
            if the_input[0] == 'a':
                m0 = value
            elif the_input[0] == 'b':
                m1 = value
            elif the_input[0] == 'c':
                m2 = value
            elif the_input[0] == 'd':
                m3 = value
        print(m0, m1, m2, m3)
        esc0.value = m0
        esc1.value = m1
        esc2.value = m2
        esc3.value = m3
        utime.sleep_us(200000)


def test_dshot():
    from machine import Pin
    esc0 = DSHOT(Pin(6))
    esc1 = DSHOT(Pin(7))
    esc2 = DSHOT(Pin(8))
    esc3 = DSHOT(Pin(9))

    values = [300, 330, 360, 400]
    for value in values:
        m0 = m1 = m2 = m3 = value
        print(m0, m1, m2, m3)
        esc0.value = m0
        esc1.value = m1
        esc2.value = m2
        esc3.value = m3
        utime.sleep_us(200000)

    while True:
        the_input = input("輸入 Dshot value: (例如: a100, 值域：0 - 2047)")
        if len(the_input)>1:
            value = int(the_input[1:])
            if the_input[0] == 'a':
                m0 = value
            elif the_input[0] == 'b':
                m1 = value
            elif the_input[0] == 'c':
                m2 = value
            elif the_input[0] == 'd':
                m3 = value
        print(m0, m1, m2, m3)
        esc0.value = m0
        esc1.value = m1
        esc2.value = m2
        esc3.value = m3
        utime.sleep_us(20000)


if __name__=='__main__':
    test_dshot()
