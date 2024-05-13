#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import message_filters
import math
import copy
import numpy as np

motion_pattern = [
]
class pos_controller():
    def __init__(self):
        ##↓member_params↓##
        self.n = 8
        self.ids = [0,1,2,3,4,5,6,7]
        # joint id and servo id
        self.joints = {0:[0,1],1:[2,3],2:[4,5],3:[6,7]}
        #standard_vel
        self.std_vel= 1
        #physical params
        self.rad = 0.1

        #rate
        self.rate_left = 1
        self.rate_right = 1
        ##↑member_params↑##
        rospy.Subscriber("/cmd_vel_robot",Twist,self.callback)
        # これだけでsubscribeは開始される
    
    def callback(self,cv_rb=Twist()):
        # 左右の時間rateを算出・保存するやつ
        # UI側でcmd_velは一旦ang.zとlin.xに限定している.
        vel = cv_rb.linear.x
        omg = cv_rb.angular.z

        self.rate_left = (vel-self.rad*omg)/self.std_vel
        self.rate_right = (vel+self.rad*omg)/self.std_vel

    def loop(self):
        self.lr = "l"
        self.static = False
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

