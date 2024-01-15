import cv2 
from cv2 import aruco
import os
import math

alpha = 87*math.pi/180
beta = 58*math.pi/180
#depthの取得はここでできそう
#https://qiita.com/keoitate/items/efe4212b0074e10378ec

p_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
img = cv2.imread("marker1.png")
# 実際にはRealsenseで読み込んだ画像topic
corners, ids, rejectedImgPoints = aruco.detectMarkers(img, p_dict) # 検出
#img_marked = aruco.drawDetectedMarkers(img.copy(), corners, ids)   # 検出結果をオーバーレイ

# cornersは 検出マーカー数×4×2という形
#　idsは 検出マーカー数*1 二重になってるんで注意

def cos_theorem(a,b,theta):
  #a,b,theta(ab間の角)からthetaの対辺を出す
  return math.sqrt(a**2+b**2-2*a*b*math.cos(theta))

def corner_to_pos(corner,d0,d1,x_range,y_range):
  #画像内での点から実際の座標へと変換。d0を(x,y)=(0,0)でのdepthとして、そこを実座標でも(0,0)とする
  #corner = (x,y)とする。
  (x_pix,y_pix)=corner
  y_b = y_pix*beta*d1/y_range
  x_b = x_pix*alpha*d1/x_range
  thy = math.pi - math.acos(y_b/(2*d0))
  thx = math.pi - math.acos(x_b/(2*d0))
  y_real = cos_theorem(d1-d0)


