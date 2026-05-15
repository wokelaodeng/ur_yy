import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ========================================================================
    # 启动 D435i 真实深度相机节点，并配置最新版 V4.57+ 底层参数
    # ========================================================================
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')
        ),
        launch_arguments={
            # ========================================================================
            # 1. 【核心开启参数 (本次任务必开)】
            # ========================================================================
            # 作用：开启 RGB 彩色图像流。关闭则无色彩信息。
            'enable_color': 'false',         
            # 作用：开启红外深度流。点云生成的绝对核心，必须为 true。
            'enable_depth': 'true',         
            # 作用：深度与色彩对齐。由于 RGB 镜头和红外镜头有物理间距，开启此项能让生成的 3D 点云附带真实的物理颜色。
            'align_depth.enable': 'false',   
            # 作用：直接在相机驱动层生成并发布点云话题 (/camera/depth/color/points)。
            'pointcloud.enable': 'true',    
            # 作用：底层硬件级时间同步。强制让彩色帧和深度帧在同一微秒级时间戳触发，防止运动中点云颜色错位。
            'enable_sync': 'true',          
            # 作用：启动时硬件复位。
            # 影响：【强烈建议设为 false】因为你的 D435i 之前报过 ISP Hardware Error，强制断电重启极易导致彩色镜头假死。
            'initial_reset': 'false',        

            # ========================================================================
            # 2. 【画质与性能调优参数 (新版语法)】
            # ========================================================================
            # 作用：限定深度与色彩的输出分辨率和帧率。
            # 影响：30cm 的近距离扫描无需 1080P。使用 640x480@30fps 可以极大降低 USB 带宽占用，并把 CPU 算力节省下来留给你的 PCL 联合滤波(SOR+ROR)。
            'depth_module.depth_profile': '640x480x30', 
            'rgb_camera.color_profile': '640x480x30',   
            
            # 作用：深度截断距离（单位：米）。
            # 影响：极其好用的物理级滤波！你的工件在 30cm 处，设为 0.8 米意味着相机底层会像切豆腐一样，直接把 80cm 以外的背景（墙壁、走动的人）全部丢弃，大幅减少系统计算量。
            'clip_distance': '0.8', 

            # ========================================================================
            # 3. 【官方三大后处理滤波器 (新版独立开关语法)】
            # ========================================================================
            # 作用：空间滤波器 (Spatial Filter)。像“磨皮”一样平滑点云表面。
            # 影响：开启后能有效抹平深度图的锯齿，非常适合你的平坦工件扫查。
            'spatial_filter.enable': 'true',
            
            # 作用：时间滤波器 (Temporal Filter)。消除点云的“跳动”和闪烁噪点。
            # 影响：利用历史帧融合数据。静止扫描时效果奇佳，但如果机械臂运动过快，点云边缘会出现类似“彗星尾巴”的拖影。
            'temporal_filter.enable': 'true',
            
            # 作用：破洞填补滤波器 (Hole Filling Filter)。
            # 影响：强制修补因为反光或遮挡产生的黑洞。系统默认采用最近邻像素填补。
            'hole_filling_filter.enable': 'true',

            # ========================================================================
            # 4. 【备用/进阶参数 (已注释，需要时解开)】
            # ========================================================================
            # 'camera_name': 'camera',           # 多相机组网时必用：更改相机话题前缀以防冲突 (如 camera1, camera2)。
            # 'serial_no': '',                   # 多相机组网时必用：绑定相机的物理硬件序列号，精确指定启动哪台设备。
            # 'enable_gyro': 'true',             # (D435i专属) 开启内置陀螺仪。对做 VSLAM 导航有用，对静止扫查无用。
            # 'enable_accel': 'true',            # (D435i专属) 开启内置加速度计。
            # 'publish_tf': 'true',              # 开启后，ROS 中会多出一堆 camera_link 之间的静态 TF 坐标树。
        }.items()
    )

    return LaunchDescription([
        realsense_launch
    ])