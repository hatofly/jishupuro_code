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


# generate joint angle trajectory
PI = math.pi
traj = [
    [94.67,124.18],
    [104.05,116.8],
    [111.96,112.1],
    [120.1,108.71],
    [127.32,108.46],
    [131.45,103.6],
    [135.33,94.62],
    [138.93,87.26],
    [140.74,83.2],
    [138.79,82.42],
    [134.69,85.74],
    [131.99,88.08],
    [127.09,91.92],
    [124.12,94.16],
    [119.2,97.92],
    [118.29,98.51],
    [115.44,99.83],
    [111.65,101.14],
    [109.18,101.74],
    [106.12,102.08],
    [102.93,102.35],
    [95.58,106.04],
    [91.46,125.68],
]
traj = np.array(traj)
traj = np.vectorize(np.deg2rad)(traj)
#2軸の角度変化のうち大きい方を基準角速度で割って、基準補完時間とする
#基準角速度はT-ω図の真ん中あたり
std_omega = 40*(2*PI/60)*(26/70)

#traj2[i]に、traj[i-1]からtraj[i]へのstd_omgによる到達時間を入れたい. traj2[0]については、trajの末尾からtrajの先頭に戻るときの所要時間を入れたい
itpl_times = []
for i in range(traj.shape[0]):
    t_cur=traj[i]
    t_prev=traj[(i-1)%traj.shape[0]]
    itpl_time = np.max(np.abs(t_prev-t_cur))/std_omega
    if len(itpl_times)==0:
        itpl_times.append(itpl_time)
    else:
        itpl_times.append(itpl_time+itpl_times[-1])

traj2 = np.hstack((traj,np.array(itpl_times).reshape((traj.shape[0],1))))

## traj2を時間データに対してスプライン補完 床接触時の多少の高さ誤差は無視
from scipy import interpolate
f0 = np.vectorize(interpolate.Akima1DInterpolator(traj2[:,2],traj2[:,0]))
f1 = np.vectorize(interpolate.Akima1DInterpolator(traj2[:,2],traj2[:,1]))
# f0,f1に標準時間を与えれば関節角度を返してくれる.
std_nt_tm = traj2[-1,2] #標準総時間




class pos_controller():
    def __init__(self):
        ##↓member_params↓##
        self.n = 8
        self.ids = [0,1,2,3,4,5,6,7]
        # joint id and servo id relation
        self.joints = {0:[0,1],1:[2,3],2:[4,5],3:[6,7]}
        #joint id and its commanded pos
        self.joint_cmd = {0:0,1:0,2:0,3:0}
        #standard_vel
        self.std_vel= 0.1
        #physical params
        self.rad = self.std_vel/self.rad

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

    def jnt_ctl(self):
        # self.jointsの値をサーボに反映する。
        ## joint_cmdに指示する角度値をサーボの角度に変換する必要がある。線形関係が成り立っているため、coef*jonit+ofst = mt というa,bのリストを持てば良い
        ##各ジョイントに使われる各々サーボについて、その値は共通とする。
        coefs = {0:26/70,1:26/70,2:26/70,3:26/70}
        ofsts = {0:0,1:0,2:0,3:0}
        ##サービス"dynamixel_command"を関数として読み込む
        rospy.wait_for_service("/dynamixel_command")
        dmx_cmd = rospy.ServiceProxy("/dynamixel_command",DynamixelCommand)
        for i in range(4):
            ### 各サーボについて指令を送る。
            jnt_ang=self.joint_cmd[i]
            mot_ang=coefs[i]*jnt_ang+ofsts
            cmd = DynamixelCommandRequest()
            #指令用メッセージオブジェクト
            for svid in self.joints[i]:
                #代入
                cmd.command=""
                cmd.id=svid
                cmd.addr_name="Goal_Position"
                cmd.value = mot_ang
                #指令
                dmx_cmd(cmd)

            

    def loop(self):
        static = True
        tm_rate = 10
        clk_ct_l = 0
        clk_ct_r = std_nt_tm/2
        # clk_ctはtraj_smthの基準時間（つまり三列目）を指し示す
        # 位相を保存したいが、一旦その機能はなしでいいかな。面倒くさいから...
        Rate = rospy.Rate(tm_rate)
        while not rospy.is_shutdown():
            clk_ct_l = (clk_ct_l+(1/tm_rate)*(self.rate_left))%std_nt_tm
            clk_ct_r = (clk_ct_r+(1/tm_rate)*(self.rate_left))%std_nt_tm
            self.joint_cmd[0],self.joint_cmd[1] = f0(clk_ct_l),f1(clk_ct_l)
            self.joint_cmd[2],self.joint_cmd[3] = f0(clk_ct_r),f1(clk_ct_r)
            self.jnt_ctl()
            Rate.sleep()
            
