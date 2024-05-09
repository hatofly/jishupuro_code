#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Twist
import message_filters
import math
import copy
import numpy as np
from f32npconverter import numpy2f32multi

class Trajectory_generator():
    def __init__(self):
        self.sub_vel = message_filters.Subscriber('/cmd_vel_robot', Twist,queue_size=3)
        self.sub_static = message_filters.Subscriber("/robot_static",Bool,queue_size=2)
        # /robot_static = True のときだけtrajectoryを出す。Trueのときしかpublishされないので大域変数保存は使わない（常にTrueになってしまう）
        mf = message_filters.ApproximateTimeSynchronizer([self.sub_vel, self.sub_static],queue_size=1,slop=1.5,allow_headerless=True)
        mf.registerCallback(self.callback)

        self.pub_trajectory = rospy.Publisher("/foot_trajectory",Float32MultiArray,queue_size=1)
        rospy.loginfo("foot_traj published")
        self.left_front = True
        self.walk_pattern = rospy.get_param("/walk_pattern")

        rospy.init_node("trajectory_generator")
        rospy.spin()

    def pattern_modifier(self,points,time_rate,lr_swch,time_swch):
        # pointsは[left_x,left_z,right_x,rignt_z,time] という組み合わせになっている
        #configと形式を合わせること
        new_points = copy.deepcopy(points)
        for i in range(len(new_points)):
            [lx,lz,rx,rz,t] = new_points[i]
            t *= time_rate
            if lr_swch:
                [lx,lz,rx,rz] = [rx,rz,lx,lz]
            new_points[i] = [lx,lz,rx,rz,t]
        
        if time_swch:
            new_points.reverse()
        return new_points

 

    def callback(self,msg=Twist(),msg_static=Bool()):
        if msg_static.data==True:
            rospy.loginfo("callback called")
            # 前進か回転のいずれかに丸める
            # 回転角度xロボット半径による比較
            if abs(msg.linear.x) > abs(msg.linear.z)*0.036:
                vel = msg.linear.x
                std_vel = self.walk_pattern["forward"]["std_vel"]
                if abs(vel)<std_vel*0.1:
                    # 遅すぎたら動かない
                    return
                if abs(vel)>std_vel*1.5:
                    # 速すぎたら抑える
                    vel *= std_vel*1.5/abs(vel)

                if vel>0:
                    #前進
                    points = self.pattern_modifier(self.walk_pattern["forward"]["points"],std_vel/abs(vel),self.left_front,False)
                else:
                    #後退
                    points = self.pattern_modifier(self.walk_pattern["forward"]["points"],std_vel/abs(vel),(not self.left_front),True)
                    #前に出している足を先に動かすのでright_fisrtを反転している
            else:
                # 左右回転
                omega = msg.angular.z
                std_omega = self.walk_pattern["left_turn"]["std_omega"]
                if abs(omega)<std_omega*0.1:
                    #遅すぎたら動かない
                    return
                if abs(omega)>std_omega*1.5:
                    omega *= std_omega*1.5/abs(omega)
                    #速すぎたら抑える
                points = self.pattern_modifier(self.walk_pattern["left_turn"]["points"],std_omega/abs(omega),self.left_front,(bool(omega<0) != bool(not self.left_front)))
                #この式の説明はmiroにある
            #これでpointsを設定できたのでpublishする
            self.pub_trajectory.publish(numpy2f32multi(np.array(points)))
            rospy.loginfo("published traj")
            # left_frontを更新する
            [lx,lz,rx,rz,t]=points[-1]
            self.left_front = (rx>lx)



if __name__ == "__main__":
    trajectory_generator=Trajectory_generator()