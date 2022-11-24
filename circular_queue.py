#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Auther:   Stanley Huang
# Project:  PicoDrone 0.6
# Date:     2022-11-22
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
class circular_queue:
    def __init__(self, size=8):
        self.queue = list()
        self.head = 0
        self.tail = 0
        self.maxSize = size

    def enqueue(self,data):
        if self.size() == self.maxSize-1:
            return False
        if len(self.queue)<self.maxSize:
            self.queue.append(data)
        else:
            self.queue[self.tail] = data
        self.tail = (self.tail + 1) % self.maxSize
        return True

    def dequeue(self):
        if self.size()==0:
            return None 
        data = self.queue[self.head]
        self.head = (self.head + 1) % self.maxSize
        return data

    def size(self):
        if self.tail>=self.head:
            return (self.tail-self.head)
        return (self.maxSize - (self.head-self.tail))

    def validate(self, head, tail, size):
        if head==self.head and tail==self.tail and size==self.size():
            return True
        else:
            print('!!! Expected result is '+str(head)+', '+str(tail)+', '+str(size))
            return False

def validate_result(expected, result):
    if expected!=result:
        print('!!! Expected result is '+str(expected))


def test_circular_queue():
    q = circular_queue(5)
    q.validate(0, 0, 0)

    r = q.enqueue(0)               # 0, x, x, x, x
    q.validate(0, 1, 1)
    print(r)
    validate_result(True, r)

    r = q.enqueue(1)               # 0, 1, x, x, x
    q.validate(0, 2, 2)
    print(r)
    validate_result(True, r)

    r = q.enqueue(2)               # 0, 1, 2, x, x
    q.validate(0, 3, 3)
    print(r)
    validate_result(True, r)

    r = q.enqueue(3)               # 0, 1, 2, 3, x
    q.validate(0, 4, 4)
    print(r)
    validate_result(True, r)

    r = q.enqueue(4)               # 0, 1, 2, 3, x
    q.validate(0, 4, 4)
    print(r)
    validate_result(False, r)

    r = q.enqueue(5)               # 0, 1, 2, 3, x
    q.validate(0, 4, 4)
    print(r)
    validate_result(False, r)

    r = q.enqueue(6)               # 0, 1, 2, 3, x
    q.validate(0, 4, 4)
    print(r)
    validate_result(False, r)

    r = q.dequeue()                # x, 1, 2, 3, x
    q.validate(1, 4, 3)
    print(r)
    validate_result(0, r)

    r = q.enqueue(7)                # x, 1, 2, 3, 7
    q.validate(1, 0, 4)
    print(r)
    validate_result(True, r)

    r = q.enqueue(8)                # x, 1, 2, 3, 7
    q.validate(1, 0, 4)
    print(r)
    validate_result(False, r)

    r = q.dequeue()                 # x, x, 2, 3, 7
    q.validate(2, 0, 3)
    print(r)
    validate_result(1, r)

    r = q.dequeue()                 # x, x, x, 3, 7
    q.validate(3, 0, 2)
    print(r)
    validate_result(2, r)

    r = q.dequeue()                 # x, x, x, x, 7
    q.validate(4, 0, 1)
    print(r)
    validate_result(3, r)

    r = q.dequeue()                 # x, x, x, x, x
    q.validate(0, 0, 0)
    print(r)
    validate_result(7, r)

    r = q.enqueue(9)                # 9, x, x, x, x
    q.validate(0, 1, 1)
    print(r)
    validate_result(True, r)

    r = q.enqueue(10)               # 9, 10, x, x, x
    q.validate(0, 2, 2)
    print(r)
    validate_result(True, r)

    r = q.dequeue()                 # x, 10, x, x, x
    q.validate(1, 2, 1)
    print(r)
    validate_result(9, r)

    r = q.enqueue(11)                # x, 10, 11, x, x
    q.validate(1, 3, 2)
    print(r)
    validate_result(True, r)

    r = q.dequeue()                 # x, x, 11, x, x
    q.validate(2, 3, 1)
    print(r)
    validate_result(10, r)

    r = q.dequeue()                 # x, x, x, x, x
    q.validate(3, 3, 0)
    print(r)
    validate_result(11, r)

    r = q.dequeue()                 # x, x, x, x, x
    q.validate(3, 3, 0)
    print(r)
    validate_result(None, r)



if __name__=='__main__':
    test_circular_queue()
