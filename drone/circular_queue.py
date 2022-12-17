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
