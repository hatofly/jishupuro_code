#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommandRequest,DynamixelCommand
import message_filters
import math
import copy
import numpy as np

def jnt_ctl():
    ##servo angle を0にresetするよ〜
    ##サービス"dynamixel_command"を関数として読み込む
    rospy.wait_for_service("/dynamixel_workbench/dynamixel_command")
    dmx_cmd = rospy.ServiceProxy("/dynamixel_workbench/dynamixel_command",DynamixelCommand)
    cmd = DynamixelCommandRequest()
    cmd.command=""
    cmd.addr_name="Goal_Position"
    cmd.value=0
    for i in range(8):
        cmd.id = i
    dmx_cmd(cmd)
