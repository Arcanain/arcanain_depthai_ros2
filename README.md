# ROS2_OAKDlite_test

**現状できること**
- ①カメラを認識して起動し，画像を画面に出力する
  - https://qiita.com/Nkn4ryu/items/3637e8454ddac6c7ed29
- ②小さめのAIを自動でダウンロードしてカメラにデプロイして，推論する
  - https://qiita.com/Nkn4ryu/items/e48db18556bca3354f90
  - 推論結果はターミナルでしか確認できない

注意：おそらく必要なライブラリ等のインストールが事前に必要
→記事の[ここ](https://qiita.com/Nkn4ryu/items/3637e8454ddac6c7ed29#1%E5%B0%8E%E5%85%A5)と[公式Reference](https://docs.luxonis.com/software/depthai/manual-install/)を参照してください．

## TASK
**カメラの認識・起動を行う**
- [ ] [このリポジトリ](https://github.com/Arcanain/arcanain_depthai_ros2/tree/main)をクローン
- [ ] [①の記事](https://qiita.com/Nkn4ryu/items/3637e8454ddac6c7ed29)を参考にしてある程度理解


**AIによる推論**
- [ ] [②の記事](https://qiita.com/Nkn4ryu/items/e48db18556bca3354f90)を参考にして一旦実行だけしてみてください

ここまでで，**動作確認**を行ってください．

- [ ] それが全て問題なくできた（or問題が発生したが修正して，その原因がまとめられた）ら，　C++に書き換えてください．

### 1. 依存関係

必要パッケージのインストール

```bash
sudo apt update
sudo apt install ros-humble-depthai
```

### 2. ビルドと起動

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arcanain_depthai_ros2
source install/setup.bash
ros2 launch arcanain_depthai_ros2 inference.launch.py
```

### 3. 出力確認

別ターミナルで推論結果を確認します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /detections
```

検出枠付きカメラ画像を表示する場合：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /detections/image
```
