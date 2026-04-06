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

启动传感器节点，开始模拟数据的读取和发布。
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

更新日志 (2026-03-14)
修正：Bt计算模型，引入磁导率与体积参数，对齐理论量级。
新增：支持 Gazebo 实时四元数姿态读取，模拟非垂直状态下的磁场畸变。
算法：重构 BML 数学映射引擎（罗德里格矩阵 + 空间射线叉乘目标函数）。
滤波：加入卡尔曼滤波动态预测，显著降低了高频运动下的定位波动。

更新日志 (2026-03-25)
接口：新增 magnetic\_interfaces 包，定义统一的 SensorArrayData.msg 消息格式。
消息：包含 header、true\_pose、imu\_rpy、magnetic\_fields 四个字段。
集成：sensor\_simulator.py 改用自定义消息发布，替代原有的 Float64MultiArray。
文档：更新 README.md，新增自定义消息接口说明。
