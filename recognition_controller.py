import math
import numpy as np
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray,Int32MultiArray
from sensor_msgs.msg import Image
from converter import numpy2i32multi,i32multi2numpy
from manual_controller import motion_parser,walk_controller

rospy.init_node('recognition_controller',anonymous=True)
r = rospy.rate(0.03)
# 一動作に30secかかる前提で動かす。

def callback(msg):
  operation = i32multi2numpy(msg)
  for op in operation:
    walk_controller(op[0],op[1])


def listener():
  while not rospy.is_shutdown():
    rospy.Subscriber('robot_operation',Int32MultiArray,callback)
    # spinだとrateが速すぎるのでr.sleep()を使う。
    r.sleep()

if __name__ == '__main__':
  listener()
