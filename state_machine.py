#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.7
# Date:     2022-11-25
#
# Receive data from R8EF and transfer to value between 0 - 10,000
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
import rp2

@rp2.asm_pio(set_init=rp2.PIO.IN_LOW, autopush=True, push_thresh=32)
def period():
    wrap_target()
    set(x, 0)
    wait(0, pin, 0)  # Wait for pin to go low
    wait(1, pin, 0)  # Low to high transition
    label('low_high')
    jmp(x_dec, 'next') [1]  # unconditional
    label('next')
    jmp(pin, 'low_high')  # while pin is high
    label('low')  # pin is low
    jmp(x_dec, 'nxt')
    label('nxt')
    jmp(pin, 'done')  # pin has gone high: all done
    jmp('low')
    label('done')
    in_(x, 32)  # Auto push: SM stalls if FIFO full
    wrap()

@rp2.asm_pio(set_init=rp2.PIO.IN_LOW, autopush=True, push_thresh=32)
def mark():
    wrap_target()
    set(x, 0)
    wait(0, pin, 0)  # Wait for pin to go low
    wait(1, pin, 0)  # Low to high transition
    label('low_high')
    jmp(x_dec, 'next') [1]  # unconditional
    label('next')
    jmp(pin, 'low_high')  # while pin is high
    in_(x, 32)  # Auto push: SM stalls if FIFO full
    wrap()


class R8EF_channel(rp2.StateMachine):
    # Clock is 125MHz. 3 cycles per iteration, so unit is 24.0ns
    def scale(self):
        v = self.get()
        return (1 + (v ^ 0xffffffff)) * 24e-6  # Scale to ms

    def abs_scale(self):
        val = self.scale()*10000 - 10000
        if val<0:
            val = 0
        elif val>10000:
            val = 10000   
        return val

    def const_scale(self, v):
        return v
