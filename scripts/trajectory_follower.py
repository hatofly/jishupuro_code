#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray,Bool
import math
import copy
import numpy as np
from f32npconverter import numpy2f32multi,f32multi2numpy
from kinematics import fw_kinematics,inv_kinematics

class Trajectory_follower():
    def __init__(self):
        self.sub_traj = rospy.Subscriber("/foot_trajectory",Float32MultiArray,self.callback_trajectory,queue_size=1)
        self.pub_joint_angles = rospy.Publisher("/joint_angles",Float32MultiArray,queue_size=1)
        self.pub_static = rospy.Publisher("/robot_static",Bool,queue_size=1)
        leg_sizes = rospy.get_param("/leg_sizes")
        self.r1 = leg_sizes[0]
        self.r2 = leg_sizes[1]
    
        rospy.init_node("trajectory_follower")
        rospy.sleep(6)
        self.pub_static.publish(Bool(data=True))
        rospy.spin()

    def callback_trajectory(self,msg=Float32MultiArray()):
        trajectory = f32multi2numpy(msg)
        for cmd in trajectory:
            [lx,lz,rx,rz,t] = cmd

            #逆運動学で関節角に変換
            l_thetas = inv_kinematics(self.r1,self.r2,[lx,lz])[0]
            r_thetas = inv_kinematics(self.r1,self.r2,[rx,rz])[0]
            l_thetas = np.array(l_thetas)
            r_thetas = np.array(r_thetas)
            thetas_and_t = np.append(np.concatenate(l_thetas,r_thetas),t)
            #publish
            self.pub_joint_angles(numpy2f32multi(thetas_and_t))
            #実現時間だけ待つ
            rospy.sleep(t)
        #出し終わったら/robot_staticをtrueにする
        robot_static = Bool(data=True)
        self.pub_static(robot_static)

if __name__ == "__main__":
    trajectory_generator=Trajectory_follower()
