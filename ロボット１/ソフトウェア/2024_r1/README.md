# 2024_r1
JetsonOrinNanoで動作確認済み

# Installation for JetsonOrinNano
## Clone this repos
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src

git clone https://github.com/hakoroboken/2024_r1.git

```

## Run shell scripts
以下のスクリプトを実行する前に以下のコマンドで**ROSワークスペース**内に移動してください。
```bash
cd ~/ros2_ws
```
### install realsense SDK
**RealSenseSDK**、**librealsense**をダウンロードするスクリプトです。
```bash
sudo ./src/2024_r1/scripts/install_realsense_sdk.sh
```

### import third party repos
**micro-ROS**や**realsense-ros**など外部リポを取得するスクリプトです。
```bash
sudo ./src/2024_r1/scripts/import_third_party.sh
```

### install ros2 dependencies
rosdepを更新するスクリプトです。
```bash
sudo ./src/2024_r1/scripts/install_ros_depend.sh
```

# Build
```bash
cd ~/ros2_ws

colcon build --symlink-install

. install/setup.bash
```

# Launch
## Player
```bash
ros2 launch r1_launch player.launch.xml
```

## Robot(JetsonOrinNano)
```bash
ros2 launch r1_launch auto.launch.xml
```
