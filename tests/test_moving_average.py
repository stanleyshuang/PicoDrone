#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.81
# Date:     2022-12-17
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
import unittest
from drone.moving_average import moving_average

class moving_average_test_case(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass
    
    def test_moving_average_10(self):
        ma = moving_average(3+1)
        sum = ma.sum
        average = ma.average
        self.assertTrue(sum == 0 and average == 0)
    
    def test_moving_average_sum_20(self):
        ma = moving_average(3+1)
        val = round(ma.update_val(10))
        sum = ma.sum
        average = round(ma.average)
        self.assertTrue(val == 3 and sum == 10 and average == 3)
    
    def test_moving_average_sum_30(self):
        ma = moving_average(3+1)
        val = round(ma.update_val(10))
        val = round(ma.update_val(10))
        sum = ma.sum
        average = round(ma.average)
        self.assertTrue(val == 7 and sum == 20 and average == 7)
    
    def test_moving_average_sum_40(self):
        ma = moving_average(3+1)
        val = round(ma.update_val(10))
        val = round(ma.update_val(10))
        val = round(ma.update_val(10))
        sum = ma.sum
        average = round(ma.average)
        self.assertTrue(val == 10 and sum == 30 and average == 10)
    
    def test_moving_average_sum_50(self):
        ma = moving_average(3+1)
        val = round(ma.update_val(10))
        val = round(ma.update_val(10))
        val = round(ma.update_val(10))
        val = round(ma.update_val(10))
        sum = ma.sum
        average = round(ma.average)
        self.assertTrue(val == 10 and sum == 30 and average == 10)
    