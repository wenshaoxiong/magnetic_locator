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

## 更新日志 (2026-04-06)

### 日期

2026-04-06

### 更新概况

- **模型精细化与混合建模**：实现了“偶极子 + 3D LUT + 高阶球谐 (SH) + 圆柱体积积分近似”的四位一体混合磁场模型。
- **高性能计算优化**：引入 OpenMP 并行加速计算与 $O(1)$ 均匀网格 LUT 查询引擎，显著降低高频采样下的 CPU 开销。
- **EKF 稳定性与鲁棒性增强**：集成 Joseph Form 协方差更新、四元数流形归一化及自适应观测权重（Adaptive R-Matrix）。
- **运动补偿与在线校准**：新增基于 TF 的磁源位姿外推器与基于 IMU 方差监测的零速修正（ZUPT）逻辑。
- **配置与接口扩展**：更新参数服务器与地图配置，支持磁体几何参数（半径、长度、类型）及球谐系数路径加载。

### 更新具体情况

- **磁场模型深度复现**
  - **距离触发切换**：当传感器与磁源距离 $R < 3D$（直径）时，自动从点偶极子切换至 5 点多偶极子积分近似模型，补偿近场非线性失真。
  - **高阶球谐集成**：支持从外部 YAML 加载球谐系数，在远场提供比单纯偶极子更精确的场值修正。
  - **混合查询逻辑**：优先使用核心操作区（Workspace）的 1mm LUT，超出范围自动回退至解析模型，兼顾全局覆盖与局部精度。
- **数值计算优化**
  - **并行预测**：利用 `#pragma omp parallel for` 对传感器阵列的理论场值计算进行并行化处理。
  - **缓存友好型 LUT**：优化 LUT 内存布局（[ix][iy][iz][c]），确保三线性插值时的内存访问局部性。
- **EKF 核心改进**
  - **Joseph Form**：采用 $P = (I-KH)P(I-KH)^T + KRK^T$ 更新公式，强制保证协方差矩阵的正定性。
  - **自适应权重**：根据传感器到磁源的距离动态缩放观测噪声 $R$，在高梯度区自动调大噪声容忍度，防止滤波器发散。
  - **流形约束**：在预测与更新步后显式执行四元数归一化与协方差对称化（$P = 0.5(P+P^T)$）。
- **同步与补偿**
  - **位姿外推器**：通过监听 TF 历史计算磁源瞬时线速度与角速度，外推至传感器时间戳，解决 100Hz/250Hz 频率不匹配带来的时间滞后。
  - **零速修正 (ZUPT)**：实时监测加速度计与陀螺仪方差，在静止状态下触发伪观测更新，抑制 IMU 长时间运行产生的漂移。

### 工作反馈（现有问题与改进建议）

针对目前仓库整体集成情况，仍存在以下待解决问题：

- **并行架构冗余**：原有的 `localization_node.py` 与新的 `DV25` 节点存在逻辑重复且数据链路不互通（Python 节点绕过了统一接口）。
- **评估工具链断层**：`plot_trajectory.py` 依赖离线 CSV 格式，而 `DV25` 输出为 ROS2 话题，亟需增加“实时话题记录至 CSV”的转换节点。
- **上游数据维度不足**：`/sensor_measurements` 统一接口目前缺失关键的 IMU 角速度与线加速度，导致 `DV25` 的 9-DOF 预测模型无法全功能运行。
- **模型实现简化**：当前球谐函数 (SH) 评估为基础累加逻辑，需进一步引入严格的 Legendre 递归公式以完全复现论文毫米级精度。
- **系统时间同步**：需确保所有 Python 仿真脚本严格遵循 `/clock` 仿真时钟，避免 C++ 节点因时间跳变触发 `DT_INVALID`。

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
  - `.gitignore` 仅约束工作区构建产物（build/install/log 等），保留架构图与脚本可版本化
- 接口与话题
  - 新增 `magnetic_localization_dv25_interfaces`：
    - `msg/MagSensorMsg`：传感器位姿 + 磁场观测（算法侧统一输入）
    - `msg/MagneticPose`：`header + pose + covariance[36] + status(OK/RELOCALIZING/LOST)`
    - `msg/AlgorithmStatus`：算法状态与诊断摘要
    - `srv/InitializePose`：初始化搜索与收敛判定
  - 核心节点订阅：
    - `/sensor_measurements`（SensorArrayData；与 magnetic_locator-main 2026-03-25 文档一致）
    - `/hall_effect_array`（Float64MultiArray，按传感器顺序扁平化 3N；默认按 µT 输入并在节点侧转为 T）
    - `/mag_sensor`（MagSensorMsg，可绕过 raw 扁平化输入）
    - `/imu/data`（sensor_msgs/Imu）
  - 核心节点发布：
    - `/magnetic_pose`、`/magnetic_path`、`/algorithm_status`
    - TF：`map -> base_link`，时间戳严格使用输入 `header.stamp`
  - 兼容性参数：
    - `sensor_measurements_topic`：默认 `/sensor_measurements`
    - `sensor_measurements_in_uT`：控制 SensorArrayData 的 `magnetic_fields` 是否按 µT 输入（默认 false）
- 同步与滤波
  - 采用 `message_filters::sync_policies::ApproximateTime` 对（霍尔/IMU）与（MagSensor/IMU）分别软同步，默认窗口 `sync_slop_s=0.03`
  - IMU 预测（仅在使用 `/imu/data` 输入路径时）：使用角速度积分更新四元数，线加速度经姿态旋转并加重力项更新速度/位置
  - RPY 姿态注入（使用 `/sensor_measurements` 输入路径时）：使用 `SensorArrayData.imu_rpy` 直接构造姿态四元数并做简化时间推进
  - 观测更新：磁场观测残差 `z - h(x)`，数值差分构造 Jacobian，执行 EKF 更新（Joseph 形式协方差更新）
  - 过程噪声：位置 `process_noise_pos_3x3` 与四元数 `process_noise_quat_4x4` 支持独立参数化
- 观测模型与 LUT
  - 静态磁源配置：读取 `config/magnetic_map.yaml`（磁源 id、位置、磁矩；球谐路径字段预留）
  - 磁场模型：默认使用偶极子解析模型；可选加载 `magnetic_field_lut.npz` 并进行三线性插值查询
  - LUT 生成：`scripts/generate_lut.py`（使用 `np.savez` 生成 ZIP_STORED，匹配节点端 NPZ 解析器）
- 初始化与失效恢复
  - 提供 `/initialize_pose`：在初值附近进行 RPY ± 搜索并迭代收敛后进入跟踪（默认阈值 2 cm / 2°）
  - 失效检测：磁场强度突变 `field_jump_threshold_uT` 或梯度异常 `gradient_threshold_uT_per_m` 触发 `RELOCALIZING`
  - 自动恢复：`RELOCALIZING` 状态下周期尝试局部重定位；超过 `relocalize_timeout_s` 进入 `LOST`
- 启动与参数
  - 新增 `launch/magnetic_localization.launch.py`：默认加载 `config/default_params.yaml`，支持 `sim:=true` 切换 `use_sim_time`
  - 参数支持运行时动态更新（ROS2 参数回调）：同步窗口、失效阈值、Markov 常数、测量噪声与过程噪声矩阵
- 测试与评估
  - gtest 覆盖：磁图加载、磁场预测、EKF 更新、失效检测、TF 发布、输出消息发布、初始化路径
  - 离线评估：`scripts/evaluate_bag.py` 生成 `error_stats.yaml` 与 `trajectory_plot.pdf`
- 日志
  - INFO+ 日志写入 `~/.ros/log/magnetic_localization/<date>.log`（与 `/algorithm_status` 发布同步写入）

### 工作反馈

为提升 DV25 论文复现的模型完整性与指标可复现性，上游建议补齐以下输出与约定：

- 发布标准 IMU 话题 `/imu/data`（`sensor_msgs/Imu`），至少包含 `angular_velocity`（rad/s）与 `linear_acceleration`（m/s²），并与 `/sensor_measurements.header.stamp` 同时间戳体系
- 明确并统一 `SensorArrayData.magnetic_fields` 的单位（Tesla 或 µT）与三分量坐标系（与 `header.frame_id` 保持一致）
- 提供传感器阵列几何与展平顺序的可追溯描述（顺序/安装位姿/坐标系），避免上下游对齐误差
- 做时间同步与延迟评估，确保 `/sensor_measurements` 与 `/imu/data` 可在 ≤30 ms 内软同步
- 保持或提供真值/标定辅助输出（如 `/groundtruth_pose` 或 `true_pose`）以支撑误差评估与参数标定
