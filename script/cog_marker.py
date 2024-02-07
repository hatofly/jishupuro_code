#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
from cv2 import aruco
import math
import numpy as np
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray,Int32MultiArray
from sensor_msgs.msg import Image
from converter import numpy2i32multi

# サブ関数定義
def signed_dist_line_pt(lp0,lp1,p):
  #lp0とlp1で結ばれた直線と点pの距離を求める
  (x1,y1)=lp0
  (x2,y2)=lp1
  # front,rearの順番で与えてるならば、(lp0-lp1)と(p-lp1)の外積で左右を判別
  if np.cross((lp1-lp0),(p-lp0))>0:
    sgn=-1
  else:
    sgn=1
  # 直線の方程式は(y1-y2)*x+(x2-x1)*y+(x1*y2-x2*y1)=0
  return sgn*abs((y1-y2)*p[0]+(x2-x1)*p[1]+(x1*y2-x2*y1))/math.sqrt((y1-y2)**2+(x2-x1)**2)

class localizer:
  def __init__(self) -> None:
    # 変数(変更しないつもりのもの)定義
    self.alpha = 87*math.pi/180
    self.beta = 58*math.pi/180
    self.offset_x = 5000
    self.offset_y = 5000
    self.rotation_magnifier = 3 #1radの回転を何歩に変換するのか
    self.delta_theta = math.pi/8 #進行方向とゴール方向のズレの許容値。
    self.p_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    ## pts2：marker_0の投影先
    self.pts2 = np.array([[0.0,0.0],[200.0,0.0],[200.0,200.0],[0.0,200.0]]).astype(np.float32)
    self.pts2[:,0]+=self.offset_x
    self.pts2[:,1]+=self.offset_y

    # 変数(変更を加えるつもりのもの)定義
    self.temp_container = None
    # pubを定義
    self.pub1 = rospy.Publisher("projected_map",Image,queue_size=2)
    self.pub2 = rospy.Publisher("robot_operation",Int32MultiArray,queue_size=2)
    # robot_operationは移動方向と歩数をまとめたシンプルな配列。座標に基づくoperationは本ノード内で行う。
    #depthの取得はここでできそう
    #https://qiita.com/keoitate/items/efe4212b0074e10378ec

    # nodeを開始
    rospy.init_node('localizer',anonymous=True)
    rospy.Timer(rospy.Duration(0.3),self.timer_callback)

    rospy.Subscriber('/camera/color/image_raw',Image,self.callback)
    rospy.spin() 


  # 0番マーカーをロボットと、1番マーカーをゴールとする。

  def callback(self,msg):
    rospy.loginfo("callback")
    try:
      bridge = CvBridge()
      img = bridge.imgmsg_to_cv2(msg)
      self.temp_container = img
    except Exception:
      rospy.logerr("error in image conversion")
  
  def timer_callback(self,msg):
    img = self.temp_container
    if str(type(img))=="<class 'NoneType'>":
      rospy.loginfo("img is None")
      return
    rospy.loginfo("compressing img")
    #画像の圧縮
    img=cv2.resize(img,(int(img.shape[1]/2.0),int(img.shape[0]/2.0)))
    # マーカーを検知
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, self.p_dict) 
    ## cornersは 検出マーカー数×4×2という形
    ## 左上から時計回りに格納
    ##　idsは 検出マーカー数*1 二重になってるんで注意
    ## np.where(ids==np.array([0]))
    if str(type(ids))!="<class 'NoneType'>" and len(ids)==2:
      rospy.loginfo("markers found in raw img")
      # marker_1をpts2に投影するような投影行列を作り、投影を行う。
      M = cv2.getPerspectiveTransform(corners[np.where(ids==1)[0][0]],self.pts2)
      img_rec = cv2.warpPerspective(img,M,(10000,10000))
      ## transposeは左右反転で読めなくなるので注意

      ## 回転機能は一旦要らないかな
      ### rotate_matrix = cv2.getRotationMatrix2D(center=(img_rec.shape[0]/2,img_rec.shape[1]/2), angle=40, scale=1)
      ### img_rec = cv2.warpAffine(src=img_rec, M=rotate_matrix, dsize=(img_rec.shape[0],img_rec.shape[1]))

      # 投影変換したらもう一回detect
      corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(img_rec, self.p_dict) # 検出
      if str(type(ids2))!="<class 'NoneType'>" and len(ids2)==2:
        rospy.loginfo("markers found in warped img")
        img_marked = aruco.drawDetectedMarkers(img_rec.copy(), corners2, ids2)   # 検出結果をオーバーレイ

        # 0番マーカーの4点を取得して、重心を通る直線をひく
        #rospy.loginfo("np.where(ids2==0)={}\n".format(np.where(ids2==0)))
        # ↑これがあると何故かフリーズする
        marker_0 = corners2[np.where(ids2==0)[0][0]][0]
        marker_0 = corners2[0][0]
        front = (marker_0[0]+marker_0[1])/2.0
        rear = (marker_0[2]+marker_0[3])/2.0
        img_marked = cv2.line(img_marked,((front-rear)*5+rear).astype(int),((rear-front)*5+front).astype(int),(255,0,255),thickness=3)
        #0番マーカーの4点から重心を取得
        marker_0_g = np.average(marker_0,axis=0)
        img_marked = cv2.resize(img_marked,(img_marked.shape[0]//10,img_marked.shape[1]//10))
        # この時点でprojected_mapはpublishできる。
        rospy.loginfo("publishing image")
        self.pub1.publish(data=list(np.reshape(img_marked,-1)),height = img_marked.shape[0],width=img_marked.shape[1],encoding = "rgb8",is_bigendian=0,step=int(img_marked.shape[1])*3)
        rospy.loginfo("published imaage")

        # 1番マーカーの4点から重心を取得
        marker_1 = corners2[np.where(ids2==1)[0][0]][0]
        marker_1_g=np.average(marker_1,axis=0)

        # 1番マーカー(ゴール)中心と0番マーカー進行方向直線との距離
        ## 左右を判定するため、あえて符号付き距離を使っている。線に対して点が右ならマイナス、左ならプラス
        dist_betw_line_and_destination = signed_dist_line_pt(front,rear,marker_1_g)
        # 1番マーカー中心と0番マーカー中心の距離
        dist_to_dest = math.sqrt((marker_0_g[0]-marker_1_g[0])**2 + (marker_0_g[1]-marker_1_g[1]))

        # 進行方向とゴール方向のズレ角度
        ## これを用いることで方向合わせをする。dist_betw_line_and_destinationを使わずに角度を使う理由は、ゴールまでの距離によって角度調整難易度が変動しないようにするため。
        theta = math.atan2(dist_betw_line_and_destination,dist_to_dest)
        
        # 角度合わせ指令
        if theta < -self.delta_theta:
          # 右によりすぎ。左に曲がってから前に1歩進む。
          pub_array = ((2,1),(2,1))
        elif theta > self.delta_theta:
          # 左によりすぎ。右に曲がってから前に1歩進む。
          pub_array = ((2,2),(2,2))
        else:
          #許容誤差範囲内。前に2歩進む。
          pub_array = ((2,0),(2,0))
        self.pub2.publish(numpy2i32multi(np.array(pub_array)))
      else:
        # 2度めのdetectで検知されなかった場合
        pass
    else:
      # 1度目のdetectでmarker0か1が検知されなかった場合。
      pass


if __name__ == "__main__":
  node = localizer()
# メモ欄b
  # pts2の長さが一定なので、realsenseの距離・角度を動かしても投影したときの距離は一定

  # subscribeのrateが歩行のスピードに対して速すぎるが、指令を受け取る側のノードが十分遅いrateで進めた上で動作終了を待つようにすれば大丈夫だと思われる

  # marker1が見えなかったら何もしない。何もしないことで終了判定としても開始前の状態としても使える。
  # marker0が見えない場合も何もしないので、結局0も1も見える場合のみ処理をする。