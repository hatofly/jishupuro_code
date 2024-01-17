自主プロのプログラム
=

## 使い方
ターミナルを4つ開き、
1つでは
- `source ~/catkin_ws/devel/setup.bash; roslaunch realsense2_camera rs_rgbd.launch`

残り3つで
- `source ~/enshu_ws/devel/setup.bash`

を行った後で下をそれぞれ立ち上げる。
- `python3 cog_marker.py`
- `python3 recognition_controller.py`
- `rqt_image_view`