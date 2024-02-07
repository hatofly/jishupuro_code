自主プロのプログラム
=

# 環境構築
##必要なパッケージ
- dynamixel_driver
- realsense2_camera
- roswww
  - ./jishupuro/www/robot_webcontrollerをroswww/wwwにコピーする

# 使い方

- 認識を行わなずターミナルから動かす場合: `rosrun jishupuro manual_controller.py`
- iPadからsshでtopicを送って動かす場合: `rosrun jishupuro recognition_controller.py`
- roswwwを利用して動かす場合：`roslaunch jishupuro webcontroller.launch `
- Arucoマーカーの認識を行う場合：`roslaunch jishupuro cog_walk.launch`