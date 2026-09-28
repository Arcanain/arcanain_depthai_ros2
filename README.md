**1. OAK-D + YOLOでナンバープレート検出**
'''
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arcanain_depthai_ros2 inference.launch.py
'''

**2. OCRでナンバーを読み取り**
'''
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run arcanain_depthai_ros2 ocr_node.py
'''

**3. 検出画像を表示（確認用）**
'''
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /detections/image
'''

**1〜3はそれぞれ別ターミナルで実行**
