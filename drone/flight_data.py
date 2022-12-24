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

    def format(self, s, l):
        msg = ''
        for i in range(l-len(str(s))):
            msg += ' '
        msg += str(s)
        return msg

    def show_status(self, acc_currs, acc_mas, imu_tem, rpm, pid_x0, pid_y0, pid_x1, pid_y1, pid_x2, pid_y2, pid_x3, pid_y3, indent=4):
        if not self._b_debug:
            return
        pid_x = ''
        pid_x += self.format(pid_x0, 4)
        pid_x += ' '
        pid_x += self.format(pid_x1, 4)
        pid_x += ' '
        pid_x += self.format(pid_x2, 4)
        pid_x += ' '
        pid_x += self.format(pid_x3, 4)

        pid_y = ''
        pid_y += self.format(pid_y0, 4)
        pid_y += ' '
        pid_y += self.format(pid_y1, 4)
        pid_y += ' '
        pid_y += self.format(pid_y2, 4)
        pid_y += ' '
        pid_y += self.format(pid_y3, 4)

        msg = ''
        keys = ['ax', 'axma', 'pid_x', 'ay', 'ayma', 'pid_y', 'az', 'azma', 'rpm', 't']
        lens = [3, 3, 19, 3, 3, 19, 3, 3, 4, 2]
        vals = [str(int(acc_currs[0]*100)), str(int(acc_mas[0]*100)), pid_x,
                str(int(acc_currs[1]*100)), str(int(acc_mas[1]*100)), pid_y,
                str(int(acc_currs[2]*100)), str(int(acc_mas[2]*100)), str(rpm), str(round(imu_tem))]
        for i in range(indent):
            msg += ' '
        for i in range(len(keys)):
            msg += keys[i]
            msg += '='
            msg += self.format(vals[i], lens[i])
            msg += ', '
        msg += '        '
        self.write(msg, end='\r')