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

    def show_status(self, acc_currs, gyro_currs, acc_sums, imu_tem, 
                    rpm, rpm0, rpm1, rpm2, rpm3, 
                    diff_rmp0, diff_rmp1, diff_rmp2, diff_rmp3,
                    pid_x0, pid_y0, pid_x1, pid_y1, pid_x2, pid_y2, pid_x3, pid_y3, indent=4):
        if not self._b_debug:
            return

        x_axis = ''
        x_axis += self.format(int(acc_currs[1]*100), 4)
        x_axis += ' '
        x_axis += self.format(int(gyro_currs[0]), 4)
        x_axis += ' '
        x_axis += self.format(int(acc_sums[1]*100), 4)

        y_axis = ''
        y_axis += self.format(int(acc_currs[0]*100), 4)
        y_axis += ' '
        y_axis += self.format(int(gyro_currs[1]), 4)
        y_axis += ' '
        y_axis += self.format(int(acc_sums[0]*100), 4)

        z_axis = ''
        z_axis += self.format(int(acc_currs[2]*100), 4)
        z_axis += ' '
        z_axis += self.format(int(gyro_currs[2]), 4)
        z_axis += ' '
        z_axis += self.format(int(acc_sums[2]*100), 4)

        pid_x = self.format(abs(int(pid_x0)), 3)
        pid_xs = [pid_x0, pid_x1, pid_x2, pid_x3]
        for i in range(len(pid_xs)):
            if pid_xs[i]>0:
                pid_x += '+'
            elif pid_xs[i]==0:
                pid_x += '0'
            else:
                pid_x += '-'

        pid_y = self.format(abs(int(pid_y0)), 3)
        pid_ys = [pid_y0, pid_y1, pid_y2, pid_y3]
        for i in range(len(pid_ys)):
            if pid_ys[i]>0:
                pid_y += '+'
            elif pid_ys[i]==0:
                pid_y += '0'
            else:
                pid_y += '-'

        rpms = ''
        rpms += self.format(int(rpm0), 4)
        rpms += ' '
        rpms += self.format(int(rpm1), 4)
        rpms += ' '
        rpms += self.format(int(rpm2), 4)
        rpms += ' '
        rpms += self.format(int(rpm3), 4)

        diff_rpm = ''
        diff_rpm += self.format(int(diff_rmp0), 4)
        diff_rpm += ' '
        diff_rpm += self.format(int(diff_rmp1), 4)
        diff_rpm += ' '
        diff_rpm += self.format(int(diff_rmp2), 4)
        diff_rpm += ' '
        diff_rpm += self.format(int(diff_rmp3), 4)

        msg = ''
        keys = ['x', 'pid', 'y', 'pid', 'z', 'rpm', 'rpms', 'diff']
        lens = [14, 8, 14, 8, 14, 4, 19, 19]
        vals = [x_axis, pid_x, y_axis, pid_y, z_axis, str(rpm), rpms, diff_rpm]
        for i in range(indent):
            msg += ' '
        for i in range(len(keys)):
            msg += keys[i]
            msg += '='
            msg += self.format(vals[i], lens[i])
            msg += ', '
        msg += '        '
        self.write(msg, end='\r')