#! /usr/bin/env python3
# 2本脚の順・逆運動学
import numpy as np
from scipy import optimize
from functools import partial
import copy

def fw_kinematics(r1,r2,thetas):
    thetas = np.array(thetas)
    th1,th2 = thetas[0],thetas[1]
    x = r1*np.sin(th1)+r2*np.sin(th1+th2)
    z = -(r1*np.cos(th1)+r2*np.cos(th1+th2))
    pos = np.array([x,z])
    return np.array(pos)

def inv_kinematics(r1,r2,pos):

    thetas_guess = [0,0]

    def func_to_solve(thetas):
        return fw_kinematics(r1,r2,thetas)-pos
    thetas = optimize.newton_krylov(func_to_solve,thetas_guess)
    # ただのnewtonより効率がいいらしい あと初期値の影響を拾ってくれてそう

    # -pi~piの範囲にする
    thetas[0] = thetas[0]%(2*np.pi)
    if thetas[0]>np.pi:
        thetas[0]-=2*np.pi
    thetas[1] = thetas[1]%(2*np.pi)
    if thetas[1]>np.pi:
        thetas[1]-=2*np.pi
    
    # 実際には解は2つある　2つ目の解thetas_oppを求める
    thetas_opp = copy.deepcopy(thetas)
    cent_angle = np.arctan2(pos[1],pos[0])
    thetas_opp[0] = 2*cent_angle - thetas_opp[0]
    thetas_opp[1] = -thetas_opp[1]
    roots = [thetas,thetas_opp]
    # 脚の付け根の角度が大きい順に並べ替える
    roots.sort(key=lambda x:x[0],reverse=True)
    return roots


