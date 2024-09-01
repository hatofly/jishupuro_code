#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommandRequest,DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList,DynamixelState
import message_filters
import math
import copy
import numpy as np



# 以下は関節角が0,0の場合を0とした筋長のリスト(mmオーダー)
PI = math.pi
f = open("traj.txt","r")
traj = eval(f.read())

traj = np.array(traj)
#2軸の角度変化のうち大きい方を基準角速度で割って、基準補完時間とする
#基準関節角速度はT-ω図の真ん中あたりのやつに半径をかけたやつ
std_wire_vel = 40*(2*PI/60) * 14

#traj2[i]に、traj[i-1]からtraj[i]へのstd_omgによる到達時間を入れたい. traj2[0]については、trajの末尾からtrajの先頭に戻るときの所要時間を入れたい
itpl_times = []
for i in range(traj.shape[0]):
    t_cur=traj[i]
    t_prev=traj[(i-1)%traj.shape[0]]
    itpl_time = np.max(np.abs(t_prev-t_cur))/std_wire_vel
    if len(itpl_times)==0:
        itpl_times.append(itpl_time)
    else:
        itpl_times.append(itpl_time+itpl_times[-1])

traj2 = np.hstack((traj,np.array(itpl_times).reshape((traj.shape[0],1))))
traj2 = np.vstack((np.array([traj[-1][0],traj[-1][1],0]).reshape(1,3),traj2))

## traj2を時間データに対してスプライン補完 床接触時の多少の高さ誤差は無視
from scipy import interpolate
f0 = np.vectorize(interpolate.interp1d(traj2[:,2],traj2[:,0]))
f1 = np.vectorize(interpolate.interp1d(traj2[:,2],traj2[:,1]))
# f0,f1に標準時間を与えれば関節角度を返してくれる.
std_nt_tm = traj2[-1,2] #標準総時間




class pos_controller():
    def __init__(self):
        ##↓member_params↓##
        self.n = 8
        self.ids = [0,1,2,3,4,5,6,7]
        # joint id and servo id relation
        # 0:r_knee,1:r_shoulder,2:l_shoulder,3:l_knee
        self.joints = {0:[0,4],1:[1,5],2:[3,7],3:[2,6]}
        #joint id と　それへのワイヤ長さ指令を入れる辞書変数
        self.wire_cmd = {0:0,1:0,2:0,3:0}
        #standard_vel
        self.std_vel= 0.01
        #physical params
        self.rad = 0.12 #旋回半径

        #rate
        self.rate_left = 0
        self.rate_right = 0
        self.dynamixel_states = []
        ##↑member_params↑##
        rospy.init_node("pos_controller")
        rospy.Subscriber("/cmd_vel",Twist,self.callback)
        # これだけでsubscribeは開始される
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state",DynamixelStateList,self.callback2)
    
    def callback(self,cv_rb=Twist()):
        # 左右の時間rateを算出・保存するやつ
        # UI側でcmd_velは一旦ang.zとlin.xに限定している.
        vel = cv_rb.linear.x
        omg = cv_rb.angular.z

        self.rate_left = (vel-self.rad*omg)/self.std_vel
        self.rate_right = (vel+self.rad*omg)/self.std_vel
    def callback2(self,msg=DynamixelStateList()):
        msg=sorted(list(msg.dynamixel_state),key=lambda x:x.id)
        self.dynamixel_states = msg



    def jnt_ctl(self):
        # self.jointsの値をサーボに反映する。
        ## wire_cmdに指示する角度値をサーボの角度に変換する必要がある。線形関係が成り立っているため、coef*jonit+ofst = mt というa,bのリストを持てば良い
        ##各ジョイントに使われる各々サーボについて、その値は共通とする。
        retract_coefs = [-1,+1,-1,+1] #servo id and joint retract direction
        coefs = list(-np.array(retract_coefs)) #servo id and extend direction
        ofsts = [3990.7142857142853, 7372.666666666667, 3674.714285714285, 5100.666666666667]
 #servo id and angle value for wire length 0
        ##サービス"dynamixel_command"を関数として読み込む
        rospy.wait_for_service("/dynamixel_workbench/dynamixel_command")
        dmx_cmd = rospy.ServiceProxy("/dynamixel_workbench/dynamixel_command",DynamixelCommand)
        for i in range(4):

            ### 各サーボについて指令を送る。
            mot_val=self.wire_cmd[i]*coefs[self.joints[i][0]]/14*(4096/(2*np.pi)) + ofsts[self.joints[i][0]]
            rospy.loginfo(mot_val)
            cmd = DynamixelCommandRequest()
            cmd.command = ""
            #指令用メッセージオブジェクト

            ## まずはメインサーボに位置指令
            cmd.id = self.joints[i][0]
            cmd.addr_name="Goal_Position"
            cmd.value = int(mot_val) # 指令値は0~4096のint
            #指令
            dmx_cmd(cmd)
            continue
            ## さらにメインサーボから電流を取得
            if self.dynamixel_states!=[]:
                current = self.dynamixel_states[self.joints[i][0]].present_current
                cmd.id = self.joints[i][1]
                cmd.addr_name="Goal_Current"
                cmd.value = -current
                ### 取得した値は実電流/scale_factorなので、そのまま指令に使える
                ### ギア同期なので正負反転
                dmx_cmd(cmd)
            else:
                pass


      

    def loop(self):
        tm_rate = 100
        clk_ct_l = 0
        clk_ct_r = std_nt_tm/2
        # clk_ctはtraj_smthの基準時間（つまり三列目）を指し示す
        # 位相を保存したいが、一旦その機能はなしでいいかな。面倒くさいから...
        Rate = rospy.Rate(tm_rate)
        rospy.loginfo("loop start")
        while not rospy.is_shutdown():
            rospy.loginfo("on the loop")
            clk_ct_l = (clk_ct_l+(1/tm_rate)*(self.rate_left))%std_nt_tm
            clk_ct_r = (clk_ct_r+(1/tm_rate)*(self.rate_right))%std_nt_tm
            self.wire_cmd[1],self.wire_cmd[0] = f0(clk_ct_l),f1(clk_ct_l)
            self.wire_cmd[2],self.wire_cmd[3] = f0(clk_ct_r),f1(clk_ct_r)
            self.jnt_ctl()
            Rate.sleep()

if __name__ == "__main__":
    obj = pos_controller()
    obj.loop()
            
