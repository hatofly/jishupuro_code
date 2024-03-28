#!usr/bin/env python3
import sys
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray,Float32
from f32npconverter import f32multi2numpy
from dynamixel.dynamixel_driver.src.dynamixel_driver import dynamixel_const,dynamixel_io

class Motor_interface():
    def __init__(self):
        self.sub_leg_angle = rospy.Subscriber("/joint_angles",Float32MultiArray,self.callback_leg,queue_size=1)
        #脚の制御
        self.sub_arm_angle = rospy.Subscriber("/arm_angle",Float32,self.callback_arm,queue_size=2)
        #脚とは独立にarmの制御
        self.dxl_io = dynamixel_io.DynamixelIO("/dev/dynamixel_arm",1000000)
        self.motor_ids = rospy.get_param("/motor_ids")
        # motor_idsはプログラム上のモーター番号とサーボIDの対応
        self.motor_params = rospy.get_param("/motor_params")
        # motor_paramsは各idをキーとした関節角度[rad]からサーボ角度[rad]への変換線形関数の係数

        #関節角度[rad]からサーボ指令値[0~1023]への倍率
        self.angle_rate = 1023/(300/360 * 2 * np.pi)
        #関節角速度[rad/s]からサーボ指令値[0~1023]への倍率
        self.vel_rate = 1023/0.111*2*np.pi/60 #0.111は公式資料より引用 https://www.besttechnology.co.jp/modules/knowledge/?BTX030B%20Dynamixel%20AX-12A
        rospy.init_node("motor_interface")
        rospy.spin()
    
    def callback_leg(self,msg=Float32MultiArray()):
        thetas_and_t = f32multi2numpy(msg)
        [l_th1,l_th2,r_th1,r_th2,t] = thetas_and_t
        t = thetas_and_t[4]
        for i in range(4):
            id = self.motor_ids[i]
            params = self.motor_params[id]
            [a,b] = params
            next_servo_rad = a*thetas_and_t[i]+b
            curr_servo_rad = self.dxl_io.get_position(id)/self.angle_rate
            vel = (next_servo_rad-curr_servo_rad)/t #[rad/s]
            self.dxl_io.set_position_and_speed(id,self.angle_rate*next_servo_rad,self.vel_rate*vel)
    
    def callback_arm(self,msg=Float32()):
        # プログラム上では5番目のモーターがアーム
        id = self.motor_ids[4]
        params = self.motor_params[id]
        [a,b] = params
        next_servo_rad = a*msg.data+b
        self.dxl_io.set_position(id,self.angle_rate*next_servo_rad)




