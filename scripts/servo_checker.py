#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from optparse import OptionParser
import time
#import roslib
#roslib.load_manifest('dynamixel_driver')

from dynamixel.dynamixel_driver.src.dynamixel_driver import dynamixel_const,dynamixel_io
# dynamixel_ioに全ての入出力関係が入っている




#port = "/dev/dynamixel_arm"
port = "/dev/ttyUSB0"
baudrate = 1000000
id = 1
#init_posは連続回転モードでない場合0~1023でおよそ300度を分割している
try:
  dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
except dynamixel_io.SerialOpenError as soe:
  print('ERROR:', soe)
else:
  if dxl_io.ping(id):
    print("ping responded")
    dxl_io.set_position(id,100)
  else:
    print("no response")
                  
              
      
