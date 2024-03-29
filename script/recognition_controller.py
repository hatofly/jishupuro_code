#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray,Int32MultiArray
from sensor_msgs.msg import Image
from converter import numpy2i32multi,i32multi2numpy
from manual_controller import motion_parser,walk_controller


class walker():
  def __init__(self):
    self.temp_container = None
    self.motion_list=["straight","left","right"]
    self.flag = True
    rospy.init_node('recognition_controller',anonymous=True)
    rospy.Timer(rospy.Duration(2),self.timer_callback)
    rospy.Subscriber('robot_operation',Int32MultiArray,self.callback)
    rospy.spin()

  def callback(self,msg):
    rospy.loginfo("callback")
    self.temp_container = i32multi2numpy(msg)

  def timer_callback(self,msg):
    rospy.loginfo("timer callback")
    if self.flag:
      operation = self.temp_container
      if str(type(operation))!="<class 'NoneType'>":
        for op in operation:
          rospy.loginfo("heading {}".format(self.motion_list[op[1]]))
          self.flag=False
          walk_controller(op[0],op[1])
          self.flag = True

if __name__ == '__main__':
  walk = walker()
