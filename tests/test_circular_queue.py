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
from drone.circular_queue import circular_queue

class circular_queue_test_case(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass
    
    def test_circular_queue_10(self):
        q = circular_queue(5)
        self.assertTrue(q.validate(0, 0, 0))
    
    def test_circular_queue_20(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        self.assertTrue(q.validate(0, 1, 1) and r)
    
    def test_circular_queue_30(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        self.assertTrue(q.validate(0, 2, 2) and r)
    
    def test_circular_queue_40(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        self.assertTrue(q.validate(0, 3, 3) and r)
    
    def test_circular_queue_50(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        self.assertTrue(q.validate(0, 4, 4) and r)
    
    def test_circular_queue_60(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        self.assertTrue(q.validate(0, 4, 4) and not r)
    
    def test_circular_queue_70(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        self.assertTrue(q.validate(0, 4, 4) and not r)
    
    def test_circular_queue_80(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        self.assertTrue(q.validate(0, 4, 4) and not r)
    
    def test_circular_queue_90(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        self.assertTrue(q.validate(1, 4, 3) and 0 == r)
    
    def test_circular_queue_100(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        self.assertTrue(q.validate(1, 0, 4) and r)
    
    def test_circular_queue_110(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        self.assertTrue(q.validate(1, 0, 4) and not r)
    
    def test_circular_queue_120(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        self.assertTrue(q.validate(2, 0, 3) and 1 == r)
    
    def test_circular_queue_130(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        self.assertTrue(q.validate(3, 0, 2) and 2 == r)
    
    def test_circular_queue_140(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        self.assertTrue(q.validate(4, 0, 1) and 3 == r)
    
    def test_circular_queue_150(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        self.assertTrue(q.validate(0, 0, 0) and 7 == r)
    
    def test_circular_queue_160(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        r = q.enqueue(9)               # 9, x, x, x, x
        self.assertTrue(q.validate(0, 1, 1) and r)
    
    def test_circular_queue_170(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        r = q.enqueue(9)               # 9, x, x, x, x
        r = q.enqueue(10)              # 9, 10, x, x, x
        self.assertTrue(q.validate(0, 2, 2) and r)
    
    def test_circular_queue_180(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        r = q.enqueue(9)               # 9, x, x, x, x
        r = q.enqueue(10)              # 9, 10, x, x, x
        r = q.dequeue()                # x, 10, x, x, x
        self.assertTrue(q.validate(1, 2, 1) and 9 == r)
    
    def test_circular_queue_190(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        r = q.enqueue(9)               # 9, x, x, x, x
        r = q.enqueue(10)              # 9, 10, x, x, x
        r = q.dequeue()                # x, 10, x, x, x
        r = q.enqueue(11)              # x, 10, 11, x, x
        self.assertTrue(q.validate(1, 3, 2) and r)
    
    def test_circular_queue_200(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        r = q.enqueue(9)               # 9, x, x, x, x
        r = q.enqueue(10)              # 9, 10, x, x, x
        r = q.dequeue()                # x, 10, x, x, x
        r = q.enqueue(11)              # x, 10, 11, x, x
        r = q.dequeue()                # x, x, 11, x, x
        self.assertTrue(q.validate(2, 3, 1) and 10 == r)
    
    def test_circular_queue_210(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        r = q.enqueue(9)               # 9, x, x, x, x
        r = q.enqueue(10)              # 9, 10, x, x, x
        r = q.dequeue()                # x, 10, x, x, x
        r = q.enqueue(11)              # x, 10, 11, x, x
        r = q.dequeue()                # x, x, 11, x, x
        r = q.dequeue()                # x, x, x, x, x
        self.assertTrue(q.validate(3, 3, 0) and 11 == r)
    
    def test_circular_queue_220(self):
        q = circular_queue(5)
        r = q.enqueue(0)               # 0, x, x, x, x
        r = q.enqueue(1)               # 0, 1, x, x, x
        r = q.enqueue(2)               # 0, 1, 2, x, x
        r = q.enqueue(3)               # 0, 1, 2, 3, x
        r = q.enqueue(4)               # 0, 1, 2, 3, x
        r = q.enqueue(5)               # 0, 1, 2, 3, x
        r = q.enqueue(6)               # 0, 1, 2, 3, x
        r = q.dequeue()                # x, 1, 2, 3, x
        r = q.enqueue(7)               # x, 1, 2, 3, 7
        r = q.enqueue(8)               # x, 1, 2, 3, 7
        r = q.dequeue()                # x, x, 2, 3, 7
        r = q.dequeue()                # x, x, x, 3, 7
        r = q.dequeue()                # x, x, x, x, 7
        r = q.dequeue()                # x, x, x, x, x
        r = q.enqueue(9)               # 9, x, x, x, x
        r = q.enqueue(10)              # 9, 10, x, x, x
        r = q.dequeue()                # x, 10, x, x, x
        r = q.enqueue(11)              # x, 10, 11, x, x
        r = q.dequeue()                # x, x, 11, x, x
        r = q.dequeue()                # x, x, x, x, x
        r = q.dequeue()                # x, x, x, x, x
        self.assertTrue(q.validate(3, 3, 0) and None is r)
