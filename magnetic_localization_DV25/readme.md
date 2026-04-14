# magnetic_localization_DV25

ROS2 Humble 下 Da Veiga 等 (2025) 磁定位复现的“定位算法实现”模块，已代入 `magnetic_locator-main/magnetic_localization_DV25` 以便总仓集成运行。

## 功能概览

- 输入
  - `/sensor_measurements`：`magnetic_interfaces/msg/SensorArrayData`（magnetic_locator-main 2026-03-25 统一接口）
  - `/hall_effect_array`：`std_msgs/Float64MultiArray`，按传感器顺序扁平化 `[Bx, By, Bz]`，单位默认 µT（节点内部转为 T）
  - `/mag_sensor`：`magnetic_localization_dv25_interfaces/msg/MagSensorMsg`（可直接喂给算法侧）
  - `/imu/data`：`sensor_msgs/Imu`
- 输出
  - `/magnetic_pose`：`magnetic_localization_dv25_interfaces/msg/MagneticPose`
  - `/magnetic_path`：`nav_msgs/Path`
  - `/algorithm_status`：`magnetic_localization_dv25_interfaces/msg/AlgorithmStatus`
  - `map -> base_link` TF：使用 `tf2_ros::TransformBroadcaster`，时间戳对齐到输入 `header.stamp`

## 依赖

- ROS 2 Humble
- Eigen3
- OpenMP（可选）
- yaml-cpp

脚本依赖（可选）：
- `numpy`、`pyyaml`（LUT 生成）
- `rosbag2_py`、`matplotlib`（离线评估）

## 编译

推荐在 `magnetic_locator-main` 作为 colcon 工作区根目录编译，确保本模块依赖的 `magnetic_interfaces` 可被正确发现：

```bash
cd magnetic_locator-main
colcon build --symlink-install
source install/setup.bash
```

在少数环境下也可将 `magnetic_localization_DV25/` 作为工作区根目录单独编译，但需要提前确保 `magnetic_interfaces` 已在当前环境中可用（如已被 overlay 到 `AMENT_PREFIX_PATH`）。

## 启动

```bash
ros2 launch magnetic_localization_dv25 magnetic_localization.launch.py sim:=true
```

常用 launch 参数：
- `sim`：是否使用仿真时间（`use_sim_time`）
- `params_file`：参数文件（默认加载包内 `config/default_params.yaml`）
- `magnetic_map_yaml`：磁源地图 YAML（默认包内 `config/magnetic_map.yaml`）
- `use_lut`/`lut_path`：启用并指定 `magnetic_field_lut.npz`（`np.savez` 生成的 ZIP_STORED）

## 初始化服务

服务名：`/initialize_pose`，类型：`magnetic_localization_dv25_interfaces/srv/InitializePose`

- 支持在初值附近做 RPY ± 搜索，并在迭代收敛（默认 2 cm / 2°）后进入跟踪

## 配置说明

- `config/magnetic_map.yaml`：磁源列表（id、位置、磁矩、球谐路径占位）
- `config/default_params.yaml`：默认参数模板

## LUT 生成与使用

生成磁场 LUT（不压缩）：

```bash
python3 scripts/generate_lut.py --map src/magnetic_localization_dv25/config/magnetic_map.yaml --out magnetic_field_lut.npz
```

启动时加载：

```bash
ros2 launch magnetic_localization_dv25 magnetic_localization.launch.py use_lut:=true lut_path:=/abs/path/to/magnetic_field_lut.npz
```

## 离线评估

对包含 `/magnetic_pose` 与 `/groundtruth_pose` 的 bag 做误差统计与轨迹出图：

```bash
python3 scripts/evaluate_bag.py --bag /path/to/bag --out_dir /path/to/out
```

输出：
- `error_stats.yaml`
- `trajectory_plot.pdf`

## 已知问题 / TODO

- 当前磁场观测模型提供偶极子解析模型 + 可选 LUT 插值；球谐混合项仅保留配置占位
- CPU/内存占用统计字段暂置 0，需要接入平台相关采样
- 动态调参使用 ROS2 参数回调方式；若团队统一要求 `rqt_reconfigure` 插件，需要补齐对应工具链说明
- 需在真实 Gazebo/rosbag 数据上做参数标定与性能基准（100 Hz、CPU<20%、内存<300 MB）

## 更新日志 (2026-04-14)

### 日期
2026-04-14

### 更新概况
根据论文 **Da Veiga et al. - 2025 - Magnetic localization during manipulation by two robotized permanent magnets** 进行算法深度复现与改进。重点解决了动态磁源补偿、初始化鲁棒性、以及 EKF 观测模型的数学完整性。

### 更新具体情况
1.  **重力辅助初始化 (Gravity-Aligned Initialization)：** 改进了 `InitializePose` 服务逻辑。在搜索初始位姿前，利用加速度计自动对齐 Roll 和 Pitch，将搜索空间从 6-DOF 降至 4-DOF（位置 + 绕重力轴偏航），显著提升了初始收敛速度和成功率。
2.  **增强型重定位 (Robust Relocalization)：** 在 Gauss-Newton 重定位循环中引入了重力矢量约束和磁场模长（Norm）观测。相比单纯依赖磁场矢量，该改进极大增强了算法在磁场奇异区域（Singularity Regions）的鲁棒性。
3.  **磁场模长观测项 (Magnetic Norm Observation)：** 遵循论文 Section 2.2，在 EKF 更新步中加入了磁场强度模长 $\|\mathbf{B}\|$。该项对位置高度敏感且不依赖传感器姿态，有效缓解了姿态估计偏差对位置收敛的影响。
4.  **动态磁源 TF 外推 (TF Extrapolation)：** 统一了所有输入路径的磁源位姿处理逻辑。通过提取 `updateMagneticSources` 助手函数，确保在任何时间戳下都能根据 TF 历史记录精确外推双磁体的瞬时位置与姿态，适配论文所述的动态操作场景。
5.  **诊断接口升级 (Diagnostic Upgrades)：** 更新了 `AlgorithmStatus` 消息定义，新增 `condition_number`（可观性条件数）和 `min_dist_m`（最小磁源距离）字段，为下游分析提供更丰富的算法健康度数据。

### 工作反馈（针对上游 Category 1）
1.  **IMU 数据维度：** 再次强调，算法目前虽然能处理 RPY，但为实现论文原汁原味的 EKF 动力学预测，**强烈建议上游提供原始加速度和角速度数据**。
2.  **TF 广播：** 确认磁体 TF 广播的频率应不低于 50Hz，以减少外推误差。

## 更新日志 (2026-04-06)

### 日期
2026-04-06

### 更新概况
模型精细化与 EKF 稳定性增强。引入了混合磁场观测模型（LUT + 偶极子 + 高阶修正），并针对长距离运行优化了数值稳定性。

### 更新具体情况
1.  **混合观测模型：** 实现“偶极子 + 3D LUT + 高阶球谐 (SH) + 圆柱体积积分近似”混合磁场模型。在 R < 3D 区域自动切换至高精度模型，并在工作空间核心区优先调用 LUT。
2.  **数值稳定性增强：** 将协方差更新逻辑替换为 Joseph Form，并引入四元数流形归一化，防止数值截断误差导致协方差矩阵失去正定性。
3.  **磁源位姿外推：** 基于 TF 历史实现了磁源运动补偿，确保观测矩阵 H 中的源坐标与传感器时间戳严格对齐。
4.  **中间件优化：** 引入 LoanedMessage 零拷贝发布机制，并配套 FastDDS 共享内存配置文件。

### 工作反馈（现有问题与改进建议）
1.  **架构冗余：** 现有 Python 版本的定位节点与当前 C++ 版本功能重叠，建议后期统一。
2.  **评估链条断裂：** 现有 `plot_trajectory.py` 仍依赖离线 CSV，无法实时可视化 C++ 节点的定位轨迹。
3.  **上游 IMU 数据：** 上游传感器模拟器目前只提供姿态 RPY，建议补齐角速度和线加速度，以支持更完整的 9 轴预测模型。
4.  **球谐模型参数：** 现有 SH 实现为简化占位符，需进一步获取论文中的具体 Legendre 递归系数。
5.  **系统时间同步：** 需确保所有 Python 仿真脚本严格遵循 `/clock` 仿真时钟，避免 C++ 节点因时间跳变触发 `DT_INVALID`。

## 更新日志 (2026-03-29)

### 日期
2026-03-29

### 更新概况
- 将 `magnetic_localization_DV25` 代入 `magnetic_locator-main`，用于 DV25 论文磁定位算法端实现与验证
- 新增接口包（msg/srv）与核心定位节点（组件 + 可执行），对齐算法侧统一输入输出
- 新增对 `magnetic_locator-main` 2026-03-25 统一接口 `/sensor_measurements(SensorArrayData)` 的输入适配，保证算法可直接接收上游输出
- 新增磁源地图配置、LUT 生成与加载通路、launch 与默认参数模板
- 新增 10+ gtest 单元测试与离线 bag 评估脚本
- 新增文件日志（INFO+）与 ROS2 参数动态更新回调

### 更新具体情况
- 隔离与结构
  - 新增目录 `magnetic_localization_DV25/`，包含 `src/`、`config/`、`launch/`、`test/`、`scripts/` 与文档资源
- 接口与话题
  - 新增 `magnetic_localization_dv25_interfaces`（msg/srv/status）
  - 核心节点订阅：`/sensor_measurements`、`/hall_effect_array`、`/mag_sensor`、`/imu/data`
  - 核心节点发布：`/magnetic_pose`、`/magnetic_path`、`/algorithm_status`、TF
- 同步与滤波
  - 采用 `message_filters::sync_policies::ApproximateTime`
  - IMU 预测与 RPY 姿态注入路径适配
  - 观测更新与 Joseph 形式协方差更新
- 观测模型与 LUT
  - 静态磁源配置与偶极子/LUT 切换
- 初始化与失效恢复
  - 提供 `/initialize_pose` 服务与局部重定位逻辑
