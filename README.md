## 実行方法

### 1. OAK-D + YOLOでナンバープレート検出

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arcanain_depthai_ros2 inference.launch.py
```

### 2. OCRでナンバーを読み取り

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run arcanain_depthai_ros2 ocr_node.py
```

### 3. 検出画像を表示（確認用）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /detections/image
```

> **Note:** 1〜3はそれぞれ別ターミナルで実行する
