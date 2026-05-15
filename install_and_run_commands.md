# ur_ultrasonic_scan 依赖安装与运行命令清单

把下面的 `$ROS_DISTRO` 替换成你的 ROS2 版本，例如：

```bash
export ROS_DISTRO=humble
```

## 1. Source ROS2

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

## 2. 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-ament-cmake \
  ros-$ROS_DISTRO-moveit \
  ros-$ROS_DISTRO-moveit-ros-planning-interface \
  ros-$ROS_DISTRO-moveit-visual-tools \
  ros-$ROS_DISTRO-moveit-configs-utils \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-controller-manager \
  ros-$ROS_DISTRO-joint-trajectory-controller \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-eigen \
  ros-$ROS_DISTRO-tf2-geometry-msgs \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-realsense2-camera \
  ros-$ROS_DISTRO-realsense2-camera-msgs \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-gazebo-ros-pkgs \
  ros-$ROS_DISTRO-gazebo-ros2-control \
  libopencv-dev \
  libpcl-all-dev \
  libeigen3-dev \
  libvtk9-dev \
  libqhull-dev
```cd

## 3. 用 rosdep 补齐依赖

```bash
cd /home/user/documents/learning/RL_pre/ur_ultrasonic_scan
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 4. 编译

```bash
cd /home/user/documents/learning/RL_pre/ur_ultrasonic_scan
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

cd /home/user/documents/learning/RL_pre/ur_ultrasonic_scan-master
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
export PCL_ROOT=/home/user/documents/learning/RL_pre/ur_ultrasonic_scan_deps/pcl-1.13.0-install
export CMAKE_PREFIX_PATH=$PCL_ROOT:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$PCL_ROOT/lib:$LD_LIBRARY_PATH
colcon build --symlink-install


```

## 5. 仿真运行

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f gazebo

cd ~/documents/learning/RL_pre/ur_ultrasonic_scan-master
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/user/documents/learning/RL_pre/ur_ultrasonic_scan_deps/pcl-1.13.0-install/lib:$LD_LIBRARY_PATH
ros2 launch robot_camera_control_sim sim_bringup.launch.py

ros2 launch robot_camera_control_sim sim_bringup.launch.py
```

## 6. 真实 D435i 点云采集

```bash

ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true
```

另开终端：

```bash
cd /home/user/documents/learning/RL_pre/ur_ultrasonic_scan-master
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run camera_use my_camera_use_new
```

## 7. 检查关键话题

```bash
ros2 topic list | grep -E "camera|ft|wrench|planned|scan"
ros2 topic echo /robotiq_ft300/ft_sensor_wrench
```

## 8. RL Python 依赖

```bash
cd /home/user/documents/learning/RL_pre
python3 -m venv .venv-rl
source .venv-rl/bin/activate
pip install --upgrade pip
pip install torch torchvision numpy scipy opencv-python gymnasium stable-baselines3 pybullet tensorboard scikit-image
```

## 9. 当前构建失败记录

我在未 source/未安装 ROS2 基础包的当前 shell 中测试过：

```bash
cd /home/user/documents/learning/RL_pre/ur_ultrasonic_scan
colcon build --symlink-install
```

失败原因：

```text
Could not find a package configuration file provided by "ament_cmake"
```

先执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 pkg prefix ament_cmake
```

如果仍然失败，就安装：

```bash
sudo apt install ros-$ROS_DISTRO-ament-cmake
```
