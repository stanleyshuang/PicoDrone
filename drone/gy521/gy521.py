# -*- coding: utf-8 -*-
from struct import pack

"""Main module."""
# SDA=Pin(5)
# SCL=Pin(16)
# Int = None
DEFAULT_ADDR = 0x68
REG_WHO_AM_I = 0x75

class gy521:
  def __init__(self):
    from machine import Pin, I2C
    self.bus = I2C(scl=Pin(16), sda=Pin(5), freq=400000)
    self.addr = DEFAULT_ADDR

#  def init(self, SCL=None, SDA=None, INT=None, addr=0x68):
#    from machine import Pin,I2C
#    (self.SCL, self.SDA, self.INT, self.addr) = (SCL, SDA, INT, addr)
#    self.bus = I2C(scl=Pin(SCL), sda=Pin(SDA), freq=400000)

  def ping(self):
    iam = self.bus.readfrom_mem( self.addr, REG_WHO_AM_I, 1 )
    return (iam == pack('B', 0x68))
#
#  def deinit(self):
#    self.bus.deinit()
