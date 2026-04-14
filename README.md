# 磁定位算法 (BML) 仿真与验证平台

本项目是基于 ROS2 和 Gazebo 的边界磁定位 (Boundary Magnetic Localization, BML) 算法仿真验证平台，包含机器人/磁铁模型、传感器仿真、解算算法以及轨迹可视化，用于核心算法的代码实现与仿真验证。

## 📦 目录结构说明

.
├── src/
│   ├── magnetic\_interfaces/     # 自定义消息接口包
│   │   └── msg/
│   │       └── SensorArrayData.msg  # 传感器数据统一消息格式
│   └── magnetic\_locator/       # 核心算法 Python 包
│       ├── magnetic\_math.py     # 底层物理模型与 BML 数学引擎
│       ├── sensor\_simulator.py  # 虚拟霍尔传感器阵列节点
│       ├── localization\_solver.py# 核心解算器 (PSO/DE + LM + KF)
│       └── plot\_trajectory.py   # 自动化绘图与误差分析工具
├── launch/                      # 启动脚本 (包含 Gazebo 世界与机器人加载)
├── urdf/                        # 磁铁与移动平台模型定义
└── world/                       # Gazebo 仿真环境定义

## 🛠️ 环境准备与安装

系统: Ubuntu 22.04 + ROS2 Humble
Python 依赖:
pip3 install numpy==1.24.4 scipy pandas matplotlib

## 📡 自定义消息接口

消息文件: `src/magnetic\_interfaces/msg/SensorArrayData.msg`

消息字段说明:

|字段|类型|描述|
|-|-|-|
|header|std\_msgs/Header|时间戳和坐标系|
|true\_pose|geometry\_msgs/Pose|Ground Truth 位姿|
|imu\_rpy|geometry\_msgs/Vector3|Roll, Pitch, Yaw|
|magnetic\_fields|float64\[]|展平的磁场数据|



## 操作演示步骤（注意;这里的文件目录基于我的电脑文件命名，使用时请修改）

首先到工作空间根目录进行编译，这里使用了 --symlink-install 参数创建软链接，这样后续单纯修改 Python 脚本逻辑时，无需重新编译即可生效：
cd \~/ros2\_ws
colcon build --symlink-install
为了完整运行仿真测试，你需要同时打开 4 个独立的终端。请严格按照以下顺序，在各自的终端中执行命令（确保每次运行前都在工作空间根目录加载了环境变量）。

## 终端 1：运行 Gazebo 仿真环境

启动包含机器人和磁铁的 3D 仿真物理环境。
cd \~/ros2\_ws
source install/setup.bash
ros2 launch magnetic\_locator sim\_launch.py

## 终端 2：运行传感器仿真

启动传感器节点，开始模拟数据的读取 and 发布。
cd \~/ros2\_ws
source install/setup.bash
ros2 run magnetic\_locator sensor\_simulator

## 终端 3：运行解算和打印

(注意：此步骤会先删除旧的日志文件 final\_log.csv，以确保记录的是本次运行的最新数据)
cd \~/ros2\_ws
rm final\_log.csv
source install/setup.bash
ros2 run magnetic\_locator localization\_solver

## 终端 4：移动磁铁与生成轨迹图

在这个终端里，我们可以控制磁铁运动，并在最后绘制出轨迹对比图。

# 先进入具体的脚本目录

cd \~/ros2\_ws/src/magnetic\_locator/magnetic\_locator/

# (选择执行) 运行预设的轨迹发布者

python3 trajectory\_publisher.py

# (选择执行) 手动设置特定位姿测试

python3 set_pose.py 0.1 0.1 0.25

# 仿真结束后，绘制最终轨迹图查看结果

python3 plot_trajectory.py

更新日志 (2026-04-14)
1.  **复现验证：** 完成对 `magnetic_locator-main` 仓库代码的逐条检查，确认其在 $SE(3)$ EKF 状态估计、混合观测模型、动态磁源补偿等方面完全对齐论文 **Da Veiga et al. - 2025** 的数学描述。
2.  **重力辅助初始化：** 改进了 `InitializePose` 服务逻辑，利用加速度计自动对齐 Roll 和 Pitch，将搜索空间从 6-DOF 降至 4-DOF，显著提升初始收敛速度。
3.  **增强型重定位：** 在 Gauss-Newton 重定位循环中引入了重力矢量约束和磁场模长（Norm）观测，增强了算法在奇异区域（Singularity Regions）的鲁棒性。
4.  **磁场模长观测项：** 在 EKF 更新步中加入了磁场强度模长 $\|\mathbf{B}\|$ 观测，利用其姿态无关性辅助位置快速收敛。
5.  **动态磁源补偿：** 统一了磁源位姿处理逻辑，通过 TF 历史记录精确外推双磁体在测量时刻的瞬时位姿，适配动态操纵场景。
6.  **诊断接口升级：** 更新了 `AlgorithmStatus` 消息，新增条件数（Observability）和最小磁源距离监控，为算法健康度提供数据支持。

更新日志 (2026-04-06)
模型：实现“偶极子 + 3D LUT + 高阶球谐 (SH) + 圆柱体积积分近似”混合磁场模型，支持近场/远场自动切换。
性能：集成 OpenMP 并行计算与 $O(1)$ 均匀网格查询优化，大幅降低高频采样下的 CPU 负载。
稳定：引入 Joseph Form 协方差更新、四元数流形归一化及自适应观测权重（Adaptive R-Matrix），解决高梯度区发散问题。
补偿：新增基于 TF 的磁源位姿外推器与基于 IMU 方差监测的在线零速修正 (ZUPT)，抑制长时运行漂移。
集成：更新 DV25 内部参数服务器与地图配置，支持磁体几何参数（半径、长度、类型）加载，对齐论文深层复现要求。
文档：在 DV25 README 中详细列出当前系统集成的“并行架构冗余”与“评估工具链断层”等现有问题与改进建议。

更新日志 (2026-03-25)
接口：新增 magnetic\_interfaces 包，定义统一的 SensorArrayData.msg 消息格式。
消息：包含 header、true\_pose、imu\_rpy、magnetic\_fields 四个字段。
集成：sensor\_simulator.py 改用自定义消息发布，替代原有的 Float64MultiArray。
文档：更新 README.md，新增自定义消息接口说明。

更新日志 (2026-03-29)
集成：将 magnetic_localization_DV25 代入 magnetic_locator-main/magnetic_localization_DV25，作为总仓论文复现算法模块。
接口：DV25 新增订阅 /sensor_measurements（magnetic_interfaces/SensorArrayData），对齐 2026-03-25 上游统一接口，算法端可直接接收传感器输出。
参数：新增 sensor_measurements_topic 与 sensor_measurements_in_uT，支持话题名与磁场单位差异的兼容切换。
依赖：DV25 包内补齐 magnetic_interfaces 依赖声明与构建依赖，避免跨包编译缺失。
文档：完善 DV25 readme 今日更新日志与“工作反馈”，明确上游需补齐 /imu/data 等信息维度以提升论文复现完整性（详情查看DV25中的readme）
