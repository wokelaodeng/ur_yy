# 论文复现与 ur_ultrasonic_scan 项目对齐记录

日期：2026-05-14  
本地材料：
- 论文：`Autonomic_Robotic_Ultrasound_Imaging_System_Based_on_Reinforcement_Learning.pdf`
- 已下载仓库：`ur_ultrasonic_scan/`
- 仓库地址：`https://github.com/LK-gift123/ur_ultrasonic_scan`
- 当前仓库提交：`0c91124 添加了ft300六维力传感器,增加了接收力矩的功能...`

## 1. 结论先说

这个 GitHub 项目可以作为复现论文的硬件/ROS2 基础工程直接用一大半，但它不是论文的完整 RL 复现代码。

可直接复用：
- UR 系列机械臂模型、MoveIt 配置、Gazebo/RViz 启动框架。
- D435i/RealSense 点云采集、点云滤波、桌面分割、目标点云提取、MLS 法向估计。
- 点云从相机坐标系转到 `base_link` 的 TF 流程。
- 基于点云法向生成蛇形扫描轨迹，并通过 MoveIt 执行扫描。
- Robotiq FT300/力传感器几何模型和话题接入线索，仓库提交说明里提到 `/robotiq_ft300/ft_sensor_wrench`。

需要新增/重做：
- PPO 强化学习训练环境。
- 论文中的 state representation 模型：RGB 图像 CAE 编码 + reward prediction 编码。
- 论文中的 F2D force-to-displacement 闭环控制器。
- 超声图像采集、模板匹配、互信息 reward。
- 真实机器人 120 Hz 级控制闭环；当前仓库主要走 MoveIt 轨迹规划，适合扫描路径执行，但不等价于论文的实时 RL/F2D 控制。

## 2. 论文关键复现点

论文系统由三层组成：
- 环境：RGB/深度相机、超声探头、力传感器、UR3 机械臂。
- 决策：PPO agent，输入最终是状态表示模型编码后的低维状态。
- 控制：RL 输出期望接触力 `Fc in R3`，F2D 导纳控制把期望力映射成末端位移，再通过 IK/ID/PD 转成机器人命令。

论文硬件参数：
- 机器人：UR3，动作输出频率约 120 Hz。
- 超声：Wireless UProbe，20 FPS，图像 `256x256x3`。
- 相机：RGB camera，论文不要求相机与机器人精确标定，因为策略从 RGB 图像学习。
- 计算平台：i7-4790K + GTX 1080 Ti。
- 接触力：动作空间每个方向约 `[-15, 15] N`；实验中 z 向平均接触力约 `11.9 N`，标准差约 `1.7 N`。

论文训练/实验：
- 仿真：PyBullet，用仿真先训练/验证。
- 真实系统：仿真预训练约 200 万步，真实系统再训练约 100 episodes。
- SR 对比实验：每个模型 100 trials，约 100 万步训练。
- 成功标准：稳定获取目标超声图像；若超出 workspace 或安全力阈值则失败。

## 3. GitHub 仓库实际能力

主要 ROS2 包：
- `mybot_description`：UR/夹具/相机/力传感器/桌面等 URDF、mesh、xacro。
- `ur7e_moveit_config`：MoveIt 配置，规划组为 `ur7e_manipulator`，tip 为 `ft_frame`，含 `home_pose_1`。
- `realsense2_description`：RealSense D400 系列描述文件。
- `camera_use`：真实 D435i 点云处理，核心节点 `my_camera_use_new`。
- `robot_camera_control_sim`：仿真相机处理、点云 TF、轨迹规划、机器人控制。
- `hello_moveit`：真实/手写 MoveIt 扫描样例。

核心流程：
1. 相机订阅 `/camera/camera/color/image_raw` 和 `/camera/camera/depth/color/points`。
2. 点云处理：NaN 清理、VoxelGrid、PassThrough、RANSAC 去桌面、欧式聚类、SOR、MLS 法向估计。
3. 保存点云：
   - 仿真：`target_cylinder_smoothed.pcd`
   - 真实：`src/real_point_cloud_data/target_cylinder_smoothed.pcd`
4. `point_tf_position` 将点云从 `camera_depth_optical_frame` 转成 `base_link`，输出 `target_cylinder_base_link.pcd`。
5. `Trajectory_Planner_Sim.cpp` 按 X 方向切片生成蛇形 `/planned_poses` 和 `/planned_path`。
6. `robot_control_sim.cpp` 订阅 `/planned_poses`，用 MoveIt 做接近、扫描、退刀。

## 4. 与论文的差距

| 论文模块 | 仓库状态 | 复现建议 |
|---|---|---|
| UR 机械臂/夹具/传感器建模 | 基本具备，但型号是 `ur7e` 风格配置，不是论文 UR3 | 如果你的机械臂与仓库一致，可以优先用仓库；若严格复论文 UR3，需要改 MoveIt config |
| 深度/RGB 相机 | D435i 采集和点云处理已具备 | 直接用 |
| 相机-机器人标定 | 仓库依赖 TF 转换；论文强调不要求精确标定 | 工程运行可继续用 TF；做论文式 RL 时可弱化标定依赖 |
| 超声图像 | 仓库未见完整采集/处理链路 | 需新增 ROS2 节点发布 US image |
| 力传感器 | 模型与 `/robotiq_ft300/ft_sensor_wrench` 线索存在 | 需新增订阅、零偏校准、坐标变换和安全阈值 |
| PPO | 未发现 | 需新增 Python 训练包 |
| SR 模型 | 未发现 | 需新增 PyTorch 模型 |
| F2D 导纳控制 | 未发现 | 需新增实时控制节点 |
| PyBullet 仿真 RL 环境 | 未发现 | 需新增或单独建 `rl_ultrasound_env` |

## 5. 推荐复现路线

### 阶段 A：先跑通仓库硬件扫描

目标：证明你的 ROS2、UR、D435i、MoveIt、FT300 基础链路都可用。

建议顺序：
1. 安装依赖，source ROS2。
2. 编译仓库。
3. 启动 RealSense。
4. 启动 MoveIt/RViz。
5. 运行 `camera_use/my_camera_use_new` 生成真实目标点云。
6. 确认 TF 中存在 `camera_depth_optical_frame -> base_link`。
7. 运行点云转换、轨迹规划、机器人扫描。

这一步不复现论文 RL，但能复用你的硬件并得到“自动超声扫描路径”结果。

### 阶段 B：加入力传感闭环

新增一个 ROS2 节点：
- 订阅 `/robotiq_ft300/ft_sensor_wrench`。
- 做零偏校准：空载时采 2-5 秒均值作为 bias。
- 把力从传感器坐标系转换到工具或 base 坐标系。
- 设置安全阈值，例如 `Fz > 18 N` 或合力超过阈值立刻退刀。
- 先做恒力 z 方向控制，不接 RL。

推荐先实现简化 F2D：

```text
error_f = f_desired - f_measured
delta_z = kp * error_f + kd * d(error_f)
target_pose.z += clamp(delta_z, -0.001, 0.001)
```

跑稳后再换成论文导纳形式：

```text
-Fc = M(xdd_des - xdd) + B(xd_des - xd) + K(x_des - x)
```

### 阶段 C：接入超声图像 reward

新增节点：
- 采集超声图像，统一 resize 到 `256x256x3`。
- 发布 `/ultrasound/image_raw`。
- 保存模板图像 `template.png`。
- 计算：
  - `R_US`：是否出现有效超声图像。
  - `R_lesion`：当前 US 图和模板图的 mutual information。
  - `R_maintain`：连续获得有效 US 图的累计奖励。

### 阶段 D：实现论文 RL

建议新增目录：

```text
ur_ultrasonic_scan/src/rl_ultrasound/
  envs/
    pybullet_ultrasound_env.py
    ros2_real_env.py
  models/
    state_representation.py
    ppo_policy.py
  controllers/
    f2d_admittance.py
  scripts/
    train_sim.py
    finetune_real.py
    run_policy_ros2.py
```

训练建议：
- 先 PyBullet：目标为软体简化模型，接触力用仿真 contact force 或 joint wrench 近似。
- PPO 可用 `stable-baselines3` 或自己用 PyTorch 实现。
- SR 模型先离线收集数据训练，latent image 128 + latent reward 128，拼成 256 维状态。
- 动作空间：`Box(low=-15, high=15, shape=(3,))`。
- 真实系统先只让策略输出小范围增量或期望力，不直接给大幅位移。

## 6. 依赖安装清单

以下命令不一定完全适配你的 ROS2 发行版。把 `$ROS_DISTRO` 换成你的版本，例如 `humble` 或 `foxy`。

基础 ROS2/MoveIt：

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
  ros-$ROS_DISTRO-rviz2
```

相机/视觉/PCL：

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-realsense2-camera \
  ros-$ROS_DISTRO-realsense2-camera-msgs \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-pcl-ros \
  libopencv-dev \
  libpcl-all-dev \
  libeigen3-dev \
  libvtk9-dev \
  libqhull-dev
```

Gazebo/仿真：

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-gazebo-ros-pkgs \
  ros-$ROS_DISTRO-gazebo-ros2-control
```

RL/Python：

```bash
python3 -m venv .venv-rl
source .venv-rl/bin/activate
pip install --upgrade pip
pip install torch torchvision numpy scipy opencv-python gymnasium stable-baselines3 pybullet tensorboard scikit-image
```

Robotiq FT300：
- 仓库没有包含 FT300 driver 源码。
- 你需要安装或放入能发布 `geometry_msgs/WrenchStamped` 的 FT300 ROS2 driver。
- 目标话题优先对齐仓库记录：`/robotiq_ft300/ft_sensor_wrench`。

## 7. 构建检查结果

我在当前环境执行过：

```bash
cd /home/user/documents/learning/RL_pre/ur_ultrasonic_scan
colcon build --symlink-install
```

结果失败在 ROS2 环境未 source 或未安装基础包：

```text
Could not find a package configuration file provided by "ament_cmake"
```

下一步先确认：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
echo $CMAKE_PREFIX_PATH
ros2 pkg prefix ament_cmake
```

如果 `ros2 pkg prefix ament_cmake` 仍失败，安装上面基础依赖里的 `ros-$ROS_DISTRO-ament-cmake`。

## 8. 建议的最小运行命令

编译：

```bash
cd /home/user/documents/learning/RL_pre/ur_ultrasonic_scan
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

仿真链路：

```bash
ros2 launch robot_camera_control_sim sim_bringup.launch.py
```

真实 D435i 单帧点云：

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true
ros2 run camera_use my_camera_use_new
```

检查关键话题：

```bash
ros2 topic list | grep -E "camera|ft|wrench|planned|scan"
ros2 topic echo /robotiq_ft300/ft_sensor_wrench
```

## 9. 我建议你下一步做什么

优先级最高：
1. 装齐 ROS2/MoveIt/RealSense/PCL 依赖。
2. 重新 `colcon build --symlink-install`。
3. 先跑通 `camera_use/my_camera_use_new`，拿到真实 `target_cylinder_smoothed.pcd`。
4. 再跑 MoveIt 扫描，确认机械臂路径安全。
5. 最后再加论文的 F2D 和 PPO。不要一开始就把 RL 接到真机上。

如果目标是“论文级复现”，最终要补一个独立 RL 包；如果目标是“用现有硬件做自动超声扫描”，当前仓库已经是很好的起点，可以先不用 RL。
