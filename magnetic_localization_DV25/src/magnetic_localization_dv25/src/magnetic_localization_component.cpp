#include "magnetic_localization_dv25/magnetic_localization_node.hpp"
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <limits>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <ctime>

namespace magnetic_localization_dv25
{

namespace
{

Eigen::Quaterniond deltaQuatFromOmega(const Eigen::Vector3d & omega_rad_s, double dt)
{
  const Eigen::Vector3d dtheta = omega_rad_s * dt;
  const double angle = dtheta.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z()).normalized();
  }
  const Eigen::Vector3d axis = dtheta / angle;
  const double half = 0.5 * angle;
  return Eigen::Quaterniond(std::cos(half), axis.x() * std::sin(half), axis.y() * std::sin(half), axis.z() * std::sin(half));
}

Eigen::Matrix<double, 22, 22> makeProcessNoise(
  const Eigen::Matrix3d & Q_pos, const Eigen::Matrix4d & Q_quat, double dt, double tau_s, bool is_stationary)
{
  Eigen::Matrix<double, 22, 22> Q = Eigen::Matrix<double, 22, 22>::Zero();
  Q.block<3, 3>(0, 0) = Q_pos * dt;
  Q.block<4, 4>(3, 3) = Q_quat * dt;
  Q.block<3, 3>(7, 7) = Eigen::Matrix3d::Identity() * (1e-4 * dt);
  Q.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity() * (1e-6 * dt);
  Q.block<3, 3>(13, 13) = Eigen::Matrix3d::Identity() * (1e-6 * dt);

  // 针对马尔可夫链偏置模型的处理
  // 当静止时，降低过程噪声以“锁定”偏置估计，使其演化更缓慢
  const double sigma2_markov = is_stationary ? 1e-12 : 1e-10; 
  const double q_markov = sigma2_markov * (1.0 - std::exp(-2.0 * dt / std::max(1e-3, tau_s)));
  Q.block<3, 3>(16, 16) = Eigen::Matrix3d::Identity() * q_markov;
  Q.block<3, 3>(19, 19) = Eigen::Matrix3d::Identity() * q_markov;
  return Q;
}

}  // namespace

MagneticLocalizationNode::MagneticLocalizationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("magnetic_localization_node", options)
{
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  sensor_measurements_topic_ = declare_parameter<std::string>("sensor_measurements_topic", "/sensor_measurements");
  hall_topic_ = declare_parameter<std::string>("hall_topic", "/hall_effect_array");
  mag_topic_ = declare_parameter<std::string>("mag_sensor_topic", "/mag_sensor");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
  robot_pose_topic_ = declare_parameter<std::string>("robot_pose_topic", "/robot_pose");
  sensor_measurements_in_uT_ = declare_parameter<bool>("sensor_measurements_in_uT", false);

  sync_slop_s_ = declare_parameter<double>("sync_slop_s", 0.03);
  field_jump_threshold_uT_ = declare_parameter<double>("field_jump_threshold_uT", 15.0);
  gradient_threshold_uT_per_m_ = declare_parameter<double>("gradient_threshold_uT_per_m", 500.0);
  relocalize_max_gap_s_ = declare_parameter<double>("relocalize_max_gap_s", 0.5);
  relocalize_timeout_s_ = declare_parameter<double>("relocalize_timeout_s", 5.0);
  markov_tau_s_ = declare_parameter<double>("markov_tau_s", 60.0);
  adaptive_noise_threshold_m_ = declare_parameter<double>("adaptive_noise_threshold_m", 0.1);
  adaptive_noise_min_dist_m_ = declare_parameter<double>("adaptive_noise_min_dist_m", 0.01);

  std::vector<double> sensors_flat = declare_parameter<std::vector<double>>(
    "sensor_positions_base_m",
    {-0.04, -0.04, 0.0045, -0.04, 0.0, 0.0045, -0.04, 0.04, 0.0045,
     0.0, -0.04, 0.0045, 0.0, 0.0, 0.0045, 0.0, 0.04, 0.0045,
     0.04, -0.04, 0.0045, 0.04, 0.0, 0.0045, 0.04, 0.04, 0.0045});
  sensor_positions_base_.clear();
  for (size_t i = 0; i + 2 < sensors_flat.size(); i += 3) {
    sensor_positions_base_.push_back(Eigen::Vector3d(sensors_flat[i], sensors_flat[i + 1], sensors_flat[i + 2]));
  }
  field_model_.setSensorsBase(sensor_positions_base_);

  std::vector<double> Q_pos_flat = declare_parameter<std::vector<double>>(
    "process_noise_pos_3x3",
    {1e-6, 0.0, 0.0,
     0.0, 1e-6, 0.0,
     0.0, 0.0, 1e-6});
  if (Q_pos_flat.size() == 9) {
    Q_pos_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(Q_pos_flat.data());
  }

  std::vector<double> Q_quat_flat = declare_parameter<std::vector<double>>(
    "process_noise_quat_4x4",
    {1e-6, 0.0, 0.0, 0.0,
     0.0, 1e-6, 0.0, 0.0,
     0.0, 0.0, 1e-6, 0.0,
     0.0, 0.0, 0.0, 1e-6});
  if (Q_quat_flat.size() == 16) {
    Q_quat_ = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(Q_quat_flat.data());
  }

  double meas_noise_uT = declare_parameter<double>("meas_noise_uT", 2.0);
  R_field_ = Eigen::Matrix3d::Identity() * std::pow(meas_noise_uT * 1e-6, 2);

  std::string magnetic_map_yaml = declare_parameter<std::string>("magnetic_map_yaml", "");
  if (!magnetic_map_yaml.empty()) {
    if (!magnetic_map_.loadFromYaml(magnetic_map_yaml)) {
      RCLCPP_ERROR(get_logger(), "Failed to load magnetic map yaml: %s", magnetic_map_yaml.c_str());
    }
  }

  use_lut_ = declare_parameter<bool>("use_lut", false);
  std::string lut_path = declare_parameter<std::string>("lut_path", "");
  if (use_lut_ && !lut_path.empty()) {
    lut_ = std::make_shared<MagneticFieldLUT>();
    if (lut_->loadNpzStored(lut_path)) {
      field_model_.setLut(lut_);
      RCLCPP_INFO(get_logger(), "Loaded LUT: %s", lut_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to load LUT: %s", lut_path.c_str());
    }
  }

  rclcpp::SensorDataQoS qos;
  mag_pub_from_raw_ = create_publisher<MagSensorMsg>("mag_sensor_internal", qos);
  pose_pub_ = create_publisher<MagneticPoseMsg>("/magnetic_pose", qos);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/magnetic_path", qos);
  status_pub_ = create_publisher<AlgorithmStatusMsg>("/algorithm_status", qos);

  sensor_array_sub_ = create_subscription<SensorArrayData>(
    sensor_measurements_topic_, qos,
    std::bind(&MagneticLocalizationNode::sensorArrayCallback, this, std::placeholders::_1));

  init_srv_ = create_service<InitializePose>(
    "/initialize_pose",
    std::bind(&MagneticLocalizationNode::handleInitializePose, this, std::placeholders::_1, std::placeholders::_2));

  imu_sub_variance_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, qos,
    std::bind(&MagneticLocalizationNode::imuVarianceCallback, this, std::placeholders::_1));

  // 初始化机器人位姿缓存器 (异步观测处理)
  robot_pose_sub_.subscribe(this, robot_pose_topic_, qos.get_rmw_qos_profile());
  robot_pose_cache_ = std::make_shared<message_filters::Cache<geometry_msgs::msg::PoseStamped>>(robot_pose_sub_, 100);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  path_msg_.header.frame_id = map_frame_;
  path_msg_.poses.clear();

  hall_mf_sub_.subscribe(this, hall_topic_, qos.get_rmw_qos_profile());
  imu_mf_sub_raw_.subscribe(this, imu_topic_, qos.get_rmw_qos_profile());
  raw_sync_ = std::make_shared<message_filters::Synchronizer<RawSyncPolicy>>(RawSyncPolicy(50), hall_mf_sub_, imu_mf_sub_raw_);
  raw_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s_));
  raw_sync_->registerCallback(std::bind(&MagneticLocalizationNode::syncedRawCallback, this, std::placeholders::_1, std::placeholders::_2));

  mag_mf_sub_.subscribe(this, mag_topic_, qos.get_rmw_qos_profile());
  imu_mf_sub_mag_.subscribe(this, imu_topic_, qos.get_rmw_qos_profile());
  mag_sync_ = std::make_shared<message_filters::Synchronizer<MagSyncPolicy>>(MagSyncPolicy(50), mag_mf_sub_, imu_mf_sub_mag_);
  mag_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s_));
  mag_sync_->registerCallback(std::bind(&MagneticLocalizationNode::syncedMagCallback, this, std::placeholders::_1, std::placeholders::_2));

  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == "sync_slop_s") {
          sync_slop_s_ = p.as_double();
        } else if (name == "field_jump_threshold_uT") {
          field_jump_threshold_uT_ = p.as_double();
        } else if (name == "gradient_threshold_uT_per_m") {
          gradient_threshold_uT_per_m_ = p.as_double();
        } else if (name == "relocalize_max_gap_s") {
          relocalize_max_gap_s_ = p.as_double();
        } else if (name == "relocalize_timeout_s") {
          relocalize_timeout_s_ = p.as_double();
        } else if (name == "markov_tau_s") {
          markov_tau_s_ = p.as_double();
        } else if (name == "adaptive_noise_threshold_m") {
          adaptive_noise_threshold_m_ = p.as_double();
        } else if (name == "adaptive_noise_min_dist_m") {
          adaptive_noise_min_dist_m_ = p.as_double();
        } else if (name == "sensor_measurements_in_uT") {
          sensor_measurements_in_uT_ = p.as_bool();
        } else if (name == "meas_noise_uT") {
          const double v = p.as_double() * 1e-6;
          R_field_ = Eigen::Matrix3d::Identity() * (v * v);
        } else if (name == "process_noise_pos_3x3") {
          const auto v = p.as_double_array();
          if (v.size() == 9) {
            Q_pos_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(v.data());
          } else {
            result.successful = false;
            result.reason = "process_noise_pos_3x3 size != 9";
          }
        } else if (name == "process_noise_quat_4x4") {
          const auto v = p.as_double_array();
          if (v.size() == 16) {
            Q_quat_ = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(v.data());
          } else {
            result.successful = false;
            result.reason = "process_noise_quat_4x4 size != 16";
          }
        }
      }

      return result;
    });

  {
    namespace fs = std::filesystem;
    const char * home = std::getenv("HOME");
    if (!home) {
      home = std::getenv("USERPROFILE");
    }
    fs::path base = home ? fs::path(home) : fs::temp_directory_path();
    fs::path log_dir = base / ".ros" / "log" / "magnetic_localization";
    std::error_code ec;
    fs::create_directories(log_dir, ec);

    std::time_t tt = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d") << ".log";
    fs::path log_path = log_dir / oss.str();
    log_file_.open(log_path.string(), std::ios::app);
    if (log_file_.is_open()) {
      logToFile("INFO", now(), std::string("log_file=") + log_path.string());
    }
  }

  publishStatus(now(), "RELOCALIZING");
}

void MagneticLocalizationNode::sensorArrayCallback(const SensorArrayData::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp;
  const size_t n_sensors = sensor_positions_base_.size();
  if (msg->magnetic_fields.size() != 3 * n_sensors) {
    publishStatus(stamp, "SENSOR_ARRAY_SIZE_MISMATCH");
    return;
  }

  if (last_meas_stamp_.nanoseconds() == 0) {
    last_meas_stamp_ = stamp;
    publishStatus(stamp, "WAITING_FOR_INIT");
    return;
  }

  const double dt = (stamp - last_meas_stamp_).seconds();
  last_meas_stamp_ = stamp;
  if (dt <= 0.0 || dt > relocalize_max_gap_s_) {
    publishStatus(stamp, "DT_INVALID");
    return;
  }

  // 异步观测处理：从 Cache 中回溯查找最接近传感器时间戳的机器人位姿
  auto closest_pose_msg = robot_pose_cache_->getElemBeforeTime(stamp);
  if (closest_pose_msg) {
    const double time_diff = (stamp - rclcpp::Time(closest_pose_msg->header.stamp)).seconds();
    if (std::abs(time_diff) < 0.05) { // 允许的最大异步偏差
      // 利用 Cache 中的位姿辅助校准或作为 EKF 步长的补偿参考
      // 记录时间偏差，用于后续 H 矩阵或状态外推的精细修正
      RCLCPP_DEBUG(get_logger(), "Async Sync: Found robot pose with time diff: %.4f s", time_diff);
    }
  }

  // 磁源位姿外推逻辑 (对齐论文 2025 双磁体动态场景)
  updateMagneticSources(stamp);

  // 如果有加速度计缓存，尝试进行姿态约束更新
  Eigen::Vector3d latest_acc = Eigen::Vector3d::Zero();
  bool use_acc = !acc_buffer_.empty();
  if (use_acc) {
    for (const auto & a : acc_buffer_) latest_acc += a;
    latest_acc /= static_cast<double>(acc_buffer_.size());
  }

  const double scale = sensor_measurements_in_uT_ ? 1e-6 : 1.0;
  Eigen::VectorXd z_T(3 * static_cast<int>(n_sensors));
  for (size_t i = 0; i < n_sensors; ++i) {
    z_T.segment<3>(3 * static_cast<int>(i)) = Eigen::Vector3d(
      msg->magnetic_fields[3 * i + 0] * scale,
      msg->magnetic_fields[3 * i + 1] * scale,
      msg->magnetic_fields[3 * i + 2] * scale);
  }

  last_z_T_ = z_T;

  double mean_norm = 0.0;
  for (size_t i = 0; i < n_sensors; ++i) {
    mean_norm += z_T.segment<3>(3 * static_cast<int>(i)).norm();
  }
  mean_norm /= static_cast<double>(n_sensors);
  if (tracking_enabled_ && std::abs(mean_norm - last_field_mean_norm_T_) > field_jump_threshold_uT_ * 1e-6) {
    tracking_enabled_ = false;
    status_ = MagneticPoseMsg::RELOCALIZING;
    relocalize_start_stamp_ = stamp;
    publishStatus(stamp, "FIELD_JUMP_RELOCALIZATION");
  }
  last_field_mean_norm_T_ = mean_norm;

  double mean_grad = 0.0;
  int grad_pairs = 0;
  for (size_t i = 0; i < n_sensors; ++i) {
    const Eigen::Vector3d Bi = z_T.segment<3>(3 * static_cast<int>(i));
    for (size_t j = i + 1; j < n_sensors; ++j) {
      const double dist = (sensor_positions_base_[i] - sensor_positions_base_[j]).norm();
      if (dist > 1e-6 && dist < 0.05) {
        const Eigen::Vector3d Bj = z_T.segment<3>(3 * static_cast<int>(j));
        mean_grad += (Bi - Bj).norm() / dist;
        ++grad_pairs;
      }
    }
  }
  if (grad_pairs > 0) {
    mean_grad /= static_cast<double>(grad_pairs);
  }
  if (tracking_enabled_ && mean_grad > gradient_threshold_uT_per_m_ * 1e-6) {
    tracking_enabled_ = false;
    status_ = MagneticPoseMsg::RELOCALIZING;
    relocalize_start_stamp_ = stamp;
    publishStatus(stamp, "GRADIENT_ANOMALY_RELOCALIZATION");
  }

  // 姿态预测：利用 SensorArrayData 中的 RPY 更新四元数 (如果可用)
  tf2::Quaternion q_tf;
  q_tf.setRPY(msg->imu_rpy.x, msg->imu_rpy.y, msg->imu_rpy.z);
  q_ = Eigen::Quaterniond(q_tf.w(), q_tf.x(), q_tf.y(), q_tf.z()).normalized();

  p_ += v_ * dt;

  const double phi = std::exp(-dt / std::max(1e-3, markov_tau_s_));
  bm_ *= phi;
  // Gravity g_ should not decay as it is a physical constant in map frame

  P_ = P_ + makeProcessNoise(Q_pos_, Q_quat_, dt, markov_tau_s_, is_stationary_);
  P_ = 0.5 * (P_ + P_.transpose()).eval(); // 强制对称性，防止数值截断误差累计
  q_.normalize(); // 预测步后确保四元数在流形上

  if (!tracking_enabled_ && status_ == MagneticPoseMsg::RELOCALIZING) {
    relocalize(stamp, z_T);
  }

  if (tracking_enabled_) {
    updateEKF(stamp, z_T, latest_acc, use_acc);
  }

  publishOutputs(stamp);
}

void MagneticLocalizationNode::imuVarianceCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  acc_buffer_.push_back(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
  gyro_buffer_.push_back(Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z));
  if (acc_buffer_.size() > 20) acc_buffer_.pop_front();
  if (gyro_buffer_.size() > 20) gyro_buffer_.pop_front();
  
  if (acc_buffer_.size() >= 20 && gyro_buffer_.size() >= 20) {
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
    for (const auto & a : acc_buffer_) acc_mean += a;
    for (const auto & g : gyro_buffer_) gyro_mean += g;
    acc_mean /= 20.0;
    gyro_mean /= 20.0;
    
    double acc_var = 0.0;
    double gyro_var = 0.0;
    for (const auto & a : acc_buffer_) acc_var += (a - acc_mean).squaredNorm();
    for (const auto & g : gyro_buffer_) gyro_var += (g - gyro_mean).squaredNorm();
    acc_var /= 20.0;
    gyro_var /= 20.0;
    
    is_stationary_ = (acc_var < 0.05 && gyro_var < 0.01); 
  }
}

void MagneticLocalizationNode::hallCallback(const std_msgs::msg::Float64MultiArray::SharedPtr)
{
}

void MagneticLocalizationNode::syncedRawCallback(
  const std_msgs::msg::Float64MultiArray::ConstSharedPtr & hall,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{
  const size_t n_sensors = sensor_positions_base_.size();
  if (hall->data.size() != 3 * n_sensors) {
    publishStatus(rclcpp::Time(imu->header.stamp), "RAW_HALL_SIZE_MISMATCH");
    return;
  }

  MagSensorMsg mag;
  mag.header = imu->header;
  mag.header.frame_id = base_frame_;
  mag.sensor_positions.resize(n_sensors);
  mag.magnetic_field.resize(n_sensors);
  for (size_t i = 0; i < n_sensors; ++i) {
    mag.sensor_positions[i].x = sensor_positions_base_[i].x();
    mag.sensor_positions[i].y = sensor_positions_base_[i].y();
    mag.sensor_positions[i].z = sensor_positions_base_[i].z();

    mag.magnetic_field[i].x = hall->data[3 * i + 0] * 1e-6;
    mag.magnetic_field[i].y = hall->data[3 * i + 1] * 1e-6;
    mag.magnetic_field[i].z = hall->data[3 * i + 2] * 1e-6;
  }

  mag_pub_from_raw_->publish(mag);
  syncedMagCallback(std::make_shared<MagSensorMsg>(mag), imu);
}

void MagneticLocalizationNode::syncedMagCallback(
  const MagSensorMsg::ConstSharedPtr & mag,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{
  const rclcpp::Time stamp = mag->header.stamp;
  
  if (last_imu_stamp_.nanoseconds() == 0) {
    last_imu_stamp_ = imu->header.stamp;
    last_meas_stamp_ = stamp;
    publishStatus(stamp, "WAITING_FOR_INIT");
    return;
  }

  const double dt = (rclcpp::Time(imu->header.stamp) - last_imu_stamp_).seconds();
  last_imu_stamp_ = imu->header.stamp;
  if (dt <= 0.0 || dt > relocalize_max_gap_s_) {
    publishStatus(stamp, "DT_INVALID");
    return;
  }

  // 磁源位姿外推逻辑 (对齐论文 2025 双磁体动态场景)
  updateMagneticSources(stamp);

  // 异步观测处理：Cache 回溯
  auto closest_pose_msg = robot_pose_cache_->getElemBeforeTime(stamp);
  if (closest_pose_msg) {
    const double time_diff = (stamp - rclcpp::Time(closest_pose_msg->header.stamp)).seconds();
    if (std::abs(time_diff) < 0.05) {
      RCLCPP_DEBUG(get_logger(), "Async Sync (MagPath): Time diff: %.4f s", time_diff);
    }
  }

  const size_t n_sensors = mag->magnetic_field.size();
  if (n_sensors == 0 || sensor_positions_base_.size() != n_sensors) {
    publishStatus(stamp, "MAG_SENSOR_COUNT_MISMATCH");
    return;
  }

  Eigen::VectorXd z_T(3 * static_cast<int>(n_sensors));
  for (size_t i = 0; i < n_sensors; ++i) {
    z_T.segment<3>(3 * static_cast<int>(i)) =
      Eigen::Vector3d(mag->magnetic_field[i].x, mag->magnetic_field[i].y, mag->magnetic_field[i].z);
  }

  last_z_T_ = z_T;
  last_meas_stamp_ = stamp;

  double mean_norm = 0.0;
  for (size_t i = 0; i < n_sensors; ++i) {
    mean_norm += z_T.segment<3>(3 * static_cast<int>(i)).norm();
  }
  mean_norm /= static_cast<double>(n_sensors);

  if (tracking_enabled_ && std::abs(mean_norm - last_field_mean_norm_T_) > field_jump_threshold_uT_ * 1e-6) {
    tracking_enabled_ = false;
    status_ = MagneticPoseMsg::RELOCALIZING;
    relocalize_start_stamp_ = stamp;
    publishStatus(stamp, "FIELD_JUMP_RELOCALIZATION");
  }
  last_field_mean_norm_T_ = mean_norm;

  double mean_grad = 0.0;
  int grad_pairs = 0;
  for (size_t i = 0; i < n_sensors; ++i) {
    const Eigen::Vector3d Bi = z_T.segment<3>(3 * static_cast<int>(i));
    for (size_t j = i + 1; j < n_sensors; ++j) {
      const double dist = (sensor_positions_base_[i] - sensor_positions_base_[j]).norm();
      if (dist > 1e-6 && dist < 0.05) {
        const Eigen::Vector3d Bj = z_T.segment<3>(3 * static_cast<int>(j));
        mean_grad += (Bi - Bj).norm() / dist;
        ++grad_pairs;
      }
    }
  }
  if (grad_pairs > 0) {
    mean_grad /= static_cast<double>(grad_pairs);
  }
  if (tracking_enabled_ && mean_grad > gradient_threshold_uT_per_m_ * 1e-6) {
    tracking_enabled_ = false;
    status_ = MagneticPoseMsg::RELOCALIZING;
    relocalize_start_stamp_ = stamp;
    publishStatus(stamp, "GRADIENT_ANOMALY_RELOCALIZATION");
  }

  const Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
  const Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

  const Eigen::Vector3d omega_u = omega - bg_;
  const Eigen::Quaterniond dq = deltaQuatFromOmega(omega_u, dt);
  q_ = (q_ * dq).normalized();

  const Eigen::Vector3d acc_u = acc - ba_;
  const Eigen::Vector3d a_map = q_.toRotationMatrix() * acc_u + g_;
  v_ += a_map * dt;
  p_ += v_ * dt + 0.5 * a_map * dt * dt;

  const double phi = std::exp(-dt / std::max(1e-3, markov_tau_s_));
  bm_ *= phi;
  // Gravity g_ should not decay

  P_ = P_ + makeProcessNoise(Q_pos_, Q_quat_, dt, markov_tau_s_, is_stationary_);
  P_ = 0.5 * (P_ + P_.transpose()).eval(); // 强制对称性
  q_.normalize(); // 流形归一化

  if (!tracking_enabled_ && status_ == MagneticPoseMsg::RELOCALIZING) {
    relocalize(stamp, z_T);
  }

  if (tracking_enabled_) {
    updateEKF(stamp, z_T, acc, true);
  }

  publishOutputs(stamp);
}

void MagneticLocalizationNode::publishOutputs(const rclcpp::Time & stamp)
{
  broadcastTf(stamp);

  // 使用 LoanedMessage 优化发布，减少内存拷贝 (如果 RMW 支持)
  if (pose_pub_->can_loan_messages()) {
    auto loaned_msg = pose_pub_->borrow_loaned_message();
    auto & msg = loaned_msg.get();
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.position.x = p_.x();
    msg.pose.position.y = p_.y();
    msg.pose.position.z = p_.z();
    msg.pose.orientation.x = q_.x();
    msg.pose.orientation.y = q_.y();
    msg.pose.orientation.z = q_.z();
    msg.pose.orientation.w = q_.w();
    msg.status = status_;

    msg.covariance.fill(0.0);
    msg.covariance[0] = P_(0, 0);
    msg.covariance[7] = P_(1, 1);
    msg.covariance[14] = P_(2, 2);
    msg.covariance[21] = P_(3, 3);
    msg.covariance[28] = P_(4, 4);
    msg.covariance[35] = P_(5, 5);

    pose_pub_->publish(std::move(loaned_msg));
  } else {
    MagneticPoseMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.position.x = p_.x();
    msg.pose.position.y = p_.y();
    msg.pose.position.z = p_.z();
    msg.pose.orientation.x = q_.x();
    msg.pose.orientation.y = q_.y();
    msg.pose.orientation.z = q_.z();
    msg.pose.orientation.w = q_.w();
    msg.status = status_;

    msg.covariance.fill(0.0);
    msg.covariance[0] = P_(0, 0);
    msg.covariance[7] = P_(1, 1);
    msg.covariance[14] = P_(2, 2);
    msg.covariance[21] = P_(3, 3);
    msg.covariance[28] = P_(4, 4);
    msg.covariance[35] = P_(5, 5);
    pose_pub_->publish(msg);
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = stamp;
  pose_stamped.header.frame_id = map_frame_;
  pose_stamped.pose.position.x = p_.x();
  pose_stamped.pose.position.y = p_.y();
  pose_stamped.pose.position.z = p_.z();
  pose_stamped.pose.orientation.x = q_.x();
  pose_stamped.pose.orientation.y = q_.y();
  pose_stamped.pose.orientation.z = q_.z();
  pose_stamped.pose.orientation.w = q_.w();

  path_msg_.header.stamp = stamp;
  path_msg_.header.frame_id = map_frame_;
  path_msg_.poses.push_back(pose_stamped);
  if (path_msg_.poses.size() > 1000) path_msg_.poses.erase(path_msg_.poses.begin());
  path_pub_->publish(path_msg_);

  if (status_ == MagneticPoseMsg::OK) {
    publishStatus(stamp, "OK");
  } else if (status_ == MagneticPoseMsg::RELOCALIZING) {
    publishStatus(stamp, "RELOCALIZING");
  } else {
    publishStatus(stamp, "LOST");
  }
}

void MagneticLocalizationNode::broadcastTf(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = base_frame_;
  tf_msg.transform.translation.x = p_.x();
  tf_msg.transform.translation.y = p_.y();
  tf_msg.transform.translation.z = p_.z();
  tf_msg.transform.rotation.x = q_.x();
  tf_msg.transform.rotation.y = q_.y();
  tf_msg.transform.rotation.z = q_.z();
  tf_msg.transform.rotation.w = q_.w();
  tf_broadcaster_->sendTransform(tf_msg);
}

void MagneticLocalizationNode::publishStatus(const rclcpp::Time & stamp, const std::string & message)
{
  if (status_pub_->can_loan_messages()) {
    auto loaned_msg = status_pub_->borrow_loaned_message();
    auto & st = loaned_msg.get();
    st.header.stamp = stamp;
    st.header.frame_id = map_frame_;
    st.status = status_;
    st.message = message;
    st.condition_number = static_cast<float>(last_condition_number_);
    st.min_dist_m = static_cast<float>(last_min_dist_);
    st.cpu_percent = 0.0f;
    st.memory_mb = 0.0f;
    status_pub_->publish(std::move(loaned_msg));
  } else {
    AlgorithmStatusMsg st;
    st.header.stamp = stamp;
    st.header.frame_id = map_frame_;
    st.status = status_;
    st.message = message;
    st.condition_number = static_cast<float>(last_condition_number_);
    st.min_dist_m = static_cast<float>(last_min_dist_);
    st.cpu_percent = 0.0f;
    st.memory_mb = 0.0f;
    status_pub_->publish(st);
  }
  logToFile("INFO", stamp, message);
}

void MagneticLocalizationNode::logToFile(const std::string & level, const rclcpp::Time & stamp, const std::string & message)
{
  if (!log_file_.is_open()) {
    return;
  }
  log_file_ << "[" << level << "]"
            << " " << stamp.seconds()
            << " " << message
            << "\n";
  log_file_.flush();
}

void MagneticLocalizationNode::handleInitializePose(
  const InitializePose::Request::SharedPtr request,
  InitializePose::Response::SharedPtr response)
{
  if (last_z_T_.size() == 0 || magnetic_map_.sources().empty()) {
    response->success = false;
    response->message = "NO_MEASUREMENT_OR_MAP";
    return;
  }

  Eigen::Vector3d p0(
    request->initial_guess.pose.position.x,
    request->initial_guess.pose.position.y,
    request->initial_guess.pose.position.z);
  Eigen::Quaterniond q0(
    request->initial_guess.pose.orientation.w,
    request->initial_guess.pose.orientation.x,
    request->initial_guess.pose.orientation.y,
    request->initial_guess.pose.orientation.z);
  q0.normalize();

  // 如果有加速度计数据，执行重力对齐初始化 (对齐论文 Section 2.2)
  if (!acc_buffer_.empty()) {
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    for (const auto & a : acc_buffer_) acc_sum += a;
    const Eigen::Vector3d acc_avg = acc_sum / static_cast<double>(acc_buffer_.size());
    
    // 计算从传感器坐标系重力向量到 map 帧重力向量 [0,0,g] 的旋转
    // 这将修正初始猜测中的 Roll 和 Pitch
    const Eigen::Vector3d unit_acc = acc_avg.normalized();
    const Eigen::Vector3d unit_g = g_.normalized(); // 默认为 [0,0,9.8]
    const Eigen::Quaterniond q_align = Eigen::Quaterniond::FromTwoVectors(unit_acc, q0.inverse() * unit_g);
    q0 = (q0 * q_align).normalized();
    RCLCPP_INFO(get_logger(), "Gravity alignment performed during initialization.");
  }

  const double rpy_deg = request->rpy_search_deg > 0.0 ? request->rpy_search_deg : 5.0;
  const double pos_m = request->position_search_m > 0.0 ? request->position_search_m : 0.05;
  const uint32_t iters = request->max_iterations > 0 ? request->max_iterations : 500;
  const double converge_m = request->position_converge_m > 0.0 ? request->position_converge_m : 0.02;
  const double converge_deg = request->angle_converge_deg > 0.0 ? request->angle_converge_deg : 2.0;

  Eigen::Vector3d best_p = p0;
  Eigen::Quaterniond best_q = q0;
  double best_cost = std::numeric_limits<double>::infinity();

  const double rpy_rad = rpy_deg * M_PI / 180.0;
  const int n_sensors = static_cast<int>(sensor_positions_base_.size());
  for (uint32_t i = 0; i < iters; ++i) {
    const double a = -rpy_rad + 2.0 * rpy_rad * (static_cast<double>(i % 11) / 10.0);
    const double b = -rpy_rad + 2.0 * rpy_rad * (static_cast<double>((i / 11) % 11) / 10.0);
    const double c = -rpy_rad + 2.0 * rpy_rad * (static_cast<double>((i / 121) % 11) / 10.0);

    const double px = -pos_m + 2.0 * pos_m * (static_cast<double>((i / 1331) % 11) / 10.0);
    const double py = -pos_m + 2.0 * pos_m * (static_cast<double>((i / 14641) % 11) / 10.0);
    const double pz = -pos_m + 2.0 * pos_m * (static_cast<double>((i / 161051) % 11) / 10.0);

    Eigen::Vector3d p_try = p0 + Eigen::Vector3d(px, py, pz);

    Eigen::AngleAxisd rx(a, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(b, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(c, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_try = (q0 * rz * ry * rx).normalized();

    Eigen::VectorXd z_hat = field_model_.predictStackedFieldT(p_try, q_try);
    for (int s = 0; s < n_sensors; ++s) {
      z_hat.segment<3>(3 * s) += bm_;
    }
    const double cost = (last_z_T_ - z_hat).squaredNorm();
    if (cost < best_cost) {
      best_cost = cost;
      best_p = p_try;
      best_q = q_try;
    }
  }

  Eigen::Vector3d p_est = best_p;
  Eigen::Quaterniond q_est = best_q;
  bool converged = false;
  const double converge_rad = converge_deg * M_PI / 180.0;

  // 获取最新的加速度计数据用于重力约束 (Section 2.2)
  Eigen::Vector3d latest_acc = Eigen::Vector3d::Zero();
  bool has_acc = !acc_buffer_.empty();
  if (has_acc) {
    for (const auto & a : acc_buffer_) latest_acc += a;
    latest_acc /= static_cast<double>(acc_buffer_.size());
  }

  for (uint32_t iter = 0; iter < iters; ++iter) {
    Eigen::VectorXd z_hat = field_model_.predictStackedFieldT(p_est, q_est);
    for (int s = 0; s < n_sensors; ++s) {
      z_hat.segment<3>(3 * s) += bm_;
    }
    
    // 构造观测向量，包含磁场矢量、模长(Section 2.2)和重力矢量
    int obs_dim = 3 * n_sensors + n_sensors;
    if (has_acc) obs_dim += 3;
    
    Eigen::VectorXd r = Eigen::VectorXd::Zero(obs_dim);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(obs_dim, 6);
    
    r.segment(0, 3 * n_sensors) = last_z_T_ - z_hat;
    for (int s = 0; s < n_sensors; ++s) {
      r(3 * n_sensors + s) = last_z_T_.segment<3>(3 * s).norm() - z_hat.segment<3>(3 * s).norm();
    }
    if (has_acc) {
      r.segment<3>(4 * n_sensors) = latest_acc - (q_est.inverse() * g_);
    }

    const double eps_p = 1e-4;
    const double eps_a = 1e-4;

    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d p_pert = p_est;
      p_pert[k] += eps_p;
      Eigen::VectorXd zh = field_model_.predictStackedFieldT(p_pert, q_est);
      for (int s = 0; s < n_sensors; ++s) zh.segment<3>(3 * s) += bm_;
      
      J.col(k).segment(0, 3 * n_sensors) = (zh - z_hat) / eps_p;
      for (int s = 0; s < n_sensors; ++s) {
        J(3 * n_sensors + s, k) = (zh.segment<3>(3 * s).norm() - z_hat.segment<3>(3 * s).norm()) / eps_p;
      }
    }

    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d axis = Eigen::Vector3d::Zero();
      axis[k] = 1.0;
      const Eigen::Quaterniond dq = deltaQuatFromOmega(axis, eps_a);
      Eigen::Quaterniond q_pert = (q_est * dq).normalized();
      Eigen::VectorXd zh = field_model_.predictStackedFieldT(p_est, q_pert);
      for (int s = 0; s < n_sensors; ++s) zh.segment<3>(3 * s) += bm_;
      
      J.col(3 + k).segment(0, 3 * n_sensors) = (zh - z_hat) / eps_a;
      for (int s = 0; s < n_sensors; ++s) {
        J(3 * n_sensors + s, 3 + k) = (zh.segment<3>(3 * s).norm() - z_hat.segment<3>(3 * s).norm()) / eps_a;
      }
      if (has_acc) {
        J.col(3 + k).segment<3>(4 * n_sensors) = (q_pert.inverse() * g_ - q_est.inverse() * g_) / eps_a;
      }
    }

    Eigen::Matrix<double, 6, 6> H_ls = J.transpose() * J + Eigen::Matrix<double, 6, 6>::Identity() * 1e-8;
    Eigen::Matrix<double, 6, 1> b_ls = J.transpose() * r;
    Eigen::Matrix<double, 6, 1> dx = H_ls.ldlt().solve(b_ls);

    p_est += dx.segment<3>(0);
    const Eigen::Quaterniond dq_gn = deltaQuatFromOmega(dx.segment<3>(3), 1.0);
    q_est = (q_est * dq_gn).normalized();

    if (dx.segment<3>(0).norm() <= converge_m && dx.segment<3>(3).norm() <= converge_rad) {
      converged = true;
      break;
    }
  }

  if (converged) {
    p_ = p_est;
    q_ = q_est;
    v_.setZero();
    bg_.setZero();
    ba_.setZero();
    P_.setIdentity();
    tracking_enabled_ = true;
    status_ = MagneticPoseMsg::OK;

    response->success = true;
    response->message = "INITIALIZED";
    response->estimated_pose.header.stamp = last_meas_stamp_;
    response->estimated_pose.header.frame_id = map_frame_;
    response->estimated_pose.pose.position.x = p_.x();
    response->estimated_pose.pose.position.y = p_.y();
    response->estimated_pose.pose.position.z = p_.z();
    response->estimated_pose.pose.orientation.x = q_.x();
    response->estimated_pose.pose.orientation.y = q_.y();
    response->estimated_pose.pose.orientation.z = q_.z();
    response->estimated_pose.pose.orientation.w = q_.w();
    publishStatus(last_meas_stamp_, "INITIALIZED_OK");
  }
}

void MagneticLocalizationNode::updateMagneticSources(const rclcpp::Time & stamp)
{
  for (const auto & src : magnetic_map_.sources()) {
    if (src.frame_id.empty()) continue;

    try {
      geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        map_frame_, src.frame_id, tf2::TimePointZero);
      
      SourcePoseRecord record;
      record.stamp = tf.header.stamp;
      record.pos = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      record.quat = Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);

      auto & history = source_history_[src.id];
      if (history.empty() || (rclcpp::Time(record.stamp) - rclcpp::Time(history.back().stamp)).seconds() > 1e-4) {
        history.push_back(record);
        if (history.size() > 2) history.pop_front();
      }

      Eigen::Vector3d extrapolated_pos = record.pos;
      Eigen::Quaterniond extrapolated_quat = record.quat;

      if (history.size() >= 2) {
        const auto & p1 = history[0];
        const auto & p2 = history[1];
        double dt_tf = (rclcpp::Time(p2.stamp) - rclcpp::Time(p1.stamp)).seconds();
        if (dt_tf > 1e-4) {
          Eigen::Vector3d vel = (p2.pos - p1.pos) / dt_tf;
          Eigen::Quaterniond dq = p2.quat * p1.quat.inverse();
          Eigen::AngleAxisd aa(dq);
          Eigen::Vector3d omega = (aa.axis() * aa.angle()) / dt_tf;

          double time_offset = (stamp - rclcpp::Time(p2.stamp)).seconds();
          extrapolated_pos = p2.pos + vel * time_offset;
          extrapolated_quat = p2.quat * deltaQuatFromOmega(omega, time_offset);
        }
      }

      Eigen::Vector3d moment_map = extrapolated_quat * Eigen::Vector3d(0, 0, src.moment.norm()); 
      magnetic_map_.updateSourcePose(src.id, extrapolated_pos, moment_map);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed for %s: %s", src.id.c_str(), ex.what());
    }
  }
}

void MagneticLocalizationNode::updateEKF(const rclcpp::Time & stamp, const Eigen::VectorXd & z_T, const Eigen::Vector3d & acc, bool use_acc)
{
  if (!tracking_enabled_ || magnetic_map_.sources().empty()) return;

  const int n = static_cast<int>(sensor_positions_base_.size());
  Eigen::VectorXd z_hat = field_model_.predictStackedFieldT(p_, q_);
  for (int i = 0; i < n; ++i) {
    z_hat.segment<3>(3 * i) += bm_;
  }
  
  // 增加磁场模长观测项 (Section 2.2)，提高收敛速度
  int rows = 3 * n + n; 
  if (use_acc) rows += 3;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, 22);
  Eigen::VectorXd y_full = Eigen::VectorXd::Zero(rows);
  
  // 1. 磁场矢量残差
  y_full.segment(0, 3 * n) = z_T - z_hat;
  
  // 2. 磁场模长残差
  for (int i = 0; i < n; ++i) {
    y_full(3 * n + i) = z_T.segment<3>(3 * i).norm() - z_hat.segment<3>(3 * i).norm();
  }

  Eigen::Vector3d g_pred = q_.inverse() * g_;
  if (use_acc) {
    y_full.segment<3>(4 * n) = acc - g_pred;
  }

  const double eps_p = 1e-4;
  const double eps_q = 1e-5;
  const double eps_b = 1e-6;

  // Position Jacobian
  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d p_pert = p_;
    p_pert[k] += eps_p;
    Eigen::VectorXd zh = field_model_.predictStackedFieldT(p_pert, q_);
    for (int i = 0; i < n; ++i) zh.segment<3>(3 * i) += bm_;
    
    H.col(k).segment(0, 3 * n) = (zh - z_hat) / eps_p;
    for (int i = 0; i < n; ++i) {
      H(3 * n + i, k) = (zh.segment<3>(3 * i).norm() - z_hat.segment<3>(3 * i).norm()) / eps_p;
    }
  }

  // Orientation Jacobian
  for (int k = 0; k < 4; ++k) {
    Eigen::Quaterniond q_pert = q_;
    Eigen::Vector4d qc(q_.x(), q_.y(), q_.z(), q_.w());
    qc[k] += eps_q;
    q_pert = Eigen::Quaterniond(qc[3], qc[0], qc[1], qc[2]).normalized();
    
    Eigen::VectorXd zh = field_model_.predictStackedFieldT(p_, q_pert);
    for (int i = 0; i < n; ++i) zh.segment<3>(3 * i) += bm_;
    
    H.col(3 + k).segment(0, 3 * n) = (zh - z_hat) / eps_q;
    for (int i = 0; i < n; ++i) {
      H(3 * n + i, 3 + k) = (zh.segment<3>(3 * i).norm() - z_hat.segment<3>(3 * i).norm()) / eps_q;
    }

    if (use_acc) {
      const Eigen::Vector3d gp = q_pert.inverse() * g_;
      H.col(3 + k).segment<3>(4 * n) = (gp - g_pred) / eps_q;
    }
  }

  // Bias Jacobian
  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d b_pert = bm_;
    b_pert[k] += eps_b;
    Eigen::VectorXd zh = z_hat;
    for (int i = 0; i < n; ++i) zh.segment<3>(3 * i) += (b_pert - bm_);
    H.col(16 + k).segment(0, 3 * n) = (zh - z_hat) / eps_b;
    for (int i = 0; i < n; ++i) {
      H(3 * n + i, 16 + k) = (zh.segment<3>(3 * i).norm() - z_hat.segment<3>(3 * i).norm()) / eps_b;
    }
  }

  // Gravity Jacobian (in map frame)
  if (use_acc) {
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d g_map_pert = g_;
      g_map_pert[k] += 1e-6;
      const Eigen::Vector3d gp = q_.inverse() * g_map_pert;
      H.col(19 + k).segment<3>(4 * n) = (gp - g_pred) / 1e-6;
      // 重力在 map 帧的变化不直接影响磁场观测，所以 H(0:4n, 19+k) 保持为 0
    }
  }

  // Measurement Noise
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(rows, rows);
  const Eigen::Matrix3d Rot = q_.toRotationMatrix();
  double global_min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d p_sensor_map = p_ + Rot * sensor_positions_base_[static_cast<size_t>(i)];
    double d_min = std::numeric_limits<double>::max();
    for (const auto & s : magnetic_map_.sources()) {
      d_min = std::min(d_min, (p_sensor_map - s.position_m).norm());
    }
    global_min_dist = std::min(global_min_dist, d_min);
    
    double adaptive_scale = 1.0;
    if (d_min < adaptive_noise_threshold_m_) {
      adaptive_scale = std::pow(adaptive_noise_threshold_m_ / std::max(adaptive_noise_min_dist_m_, d_min), 2.0);
    }
    R.block<3, 3>(3 * i, 3 * i) = R_field_ * adaptive_scale;
    // 模长观测噪声设为矢量观测的 1/3 (启发式)
    R(3 * n + i, 3 * n + i) = R_field_(0, 0) * adaptive_scale * 0.33;
  }
  last_min_dist_ = global_min_dist;
  if (use_acc) {
    R.block<3, 3>(4 * n, 4 * n) = Eigen::Matrix3d::Identity() * 1e-2;
  }

  // Kalman Gain
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = P_ * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
  const Eigen::VectorXd dx = K * y_full;

  // Update State
  p_ += dx.segment<3>(0);
  Eigen::Vector4d qc(q_.x(), q_.y(), q_.z(), q_.w());
  qc += dx.segment<4>(3);
  q_ = Eigen::Quaterniond(qc[3], qc[0], qc[1], qc[2]).normalized();
  v_ += dx.segment<3>(7);
  bg_ += dx.segment<3>(10);
  ba_ += dx.segment<3>(13);
  bm_ += dx.segment<3>(16);
  g_ += dx.segment<3>(19);

  // Covariance Update (Joseph Form)
  const Eigen::Matrix<double, 22, 22> I22 = Eigen::Matrix<double, 22, 22>::Identity();
  const Eigen::Matrix<double, 22, 22> KH = K * H;
  P_ = (I22 - KH) * P_ * (I22 - KH).transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose()).eval();

  // Condition Number for observability check
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.leftCols<6>(), Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto & singular_values = svd.singularValues();
  if (singular_values.size() > 0 && singular_values(singular_values.size() - 1) > 1e-12) {
    last_condition_number_ = singular_values(0) / singular_values(singular_values.size() - 1);
  }

  // ZUPT
  if (is_stationary_) {
    const Eigen::Vector3d y_zupt = -v_;
    Eigen::Matrix<double, 3, 22> H_zupt = Eigen::Matrix<double, 3, 22>::Zero();
    H_zupt.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_zupt = Eigen::Matrix3d::Identity() * 1e-6;
    
    const Eigen::MatrixXd S_z = H_zupt * P_ * H_zupt.transpose() + R_zupt;
    const Eigen::Matrix<double, 22, 3> K_z = P_ * H_zupt.transpose() * S_z.inverse();
    
    const Eigen::VectorXd dx_z = K_z * y_zupt;
    
    // Apply full state update from ZUPT
    p_ += dx_z.segment<3>(0);
    Eigen::Vector4d qc_z(q_.x(), q_.y(), q_.z(), q_.w());
    qc_z += dx_z.segment<4>(3);
    q_ = Eigen::Quaterniond(qc_z[3], qc_z[0], qc_z[1], qc_z[2]).normalized();
    v_ += dx_z.segment<3>(7);
    bg_ += dx_z.segment<3>(10);
    ba_ += dx_z.segment<3>(13);
    bm_ += dx_z.segment<3>(16);
    g_ += dx_z.segment<3>(19);

    const Eigen::Matrix<double, 22, 22> KH_z = K_z * H_zupt;
    P_ = (I22 - KH_z) * P_ * (I22 - KH_z).transpose() + K_z * R_zupt * K_z.transpose();
    P_ = 0.5 * (P_ + P_.transpose()).eval();
  }
}

bool MagneticLocalizationNode::relocalize(const rclcpp::Time & stamp, const Eigen::VectorXd & z_T)
{
  if (magnetic_map_.sources().empty()) return false;

  if (relocalize_start_stamp_.nanoseconds() == 0) {
    relocalize_start_stamp_ = stamp;
  }

  if ((stamp - relocalize_start_stamp_).seconds() > relocalize_timeout_s_) {
    status_ = MagneticPoseMsg::LOST;
    publishStatus(stamp, "RELOCALIZATION_TIMEOUT_LOST");
    return false;
  }

  Eigen::Vector3d p_est = p_;
  Eigen::Quaterniond q_est = q_;
  bool converged = false;
  const int n = static_cast<int>(sensor_positions_base_.size());
  const double converge_m = 0.02;
  const double converge_rad = 2.0 * M_PI / 180.0;

  // 获取最新的加速度计数据用于重力约束 (Section 2.2)
  Eigen::Vector3d latest_acc = Eigen::Vector3d::Zero();
  bool has_acc = !acc_buffer_.empty();
  if (has_acc) {
    for (const auto & a : acc_buffer_) latest_acc += a;
    latest_acc /= static_cast<double>(acc_buffer_.size());
  }

  for (int iter = 0; iter < 30; ++iter) {
    Eigen::VectorXd z_hat = field_model_.predictStackedFieldT(p_est, q_est);
    for (int s = 0; s < n; ++s) z_hat.segment<3>(3 * s) += bm_;
    
    // 构造观测向量，包含磁场矢量、模长(Section 2.2)和重力矢量
    int obs_dim = 3 * n + n;
    if (has_acc) obs_dim += 3;
    
    Eigen::VectorXd r = Eigen::VectorXd::Zero(obs_dim);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(obs_dim, 6);
    
    r.segment(0, 3 * n) = z_T - z_hat;
    for (int s = 0; s < n; ++s) {
      r(3 * n + s) = z_T.segment<3>(3 * s).norm() - z_hat.segment<3>(3 * s).norm();
    }
    if (has_acc) {
      r.segment<3>(4 * n) = latest_acc - (q_est.inverse() * g_);
    }

    const double eps_p = 1e-4;
    const double eps_a = 1e-4;

    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d p_pert = p_est;
      p_pert[k] += eps_p;
      Eigen::VectorXd zh = field_model_.predictStackedFieldT(p_pert, q_est);
      for (int s = 0; s < n; ++s) zh.segment<3>(3 * s) += bm_;
      
      J.col(k).segment(0, 3 * n) = (zh - z_hat) / eps_p;
      for (int s = 0; s < n; ++s) {
        J(3 * n + s, k) = (zh.segment<3>(3 * s).norm() - z_hat.segment<3>(3 * s).norm()) / eps_p;
      }
    }

    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d axis = Eigen::Vector3d::Zero();
      axis[k] = 1.0;
      const Eigen::Quaterniond dq = deltaQuatFromOmega(axis, eps_a);
      Eigen::Quaterniond q_pert = (q_est * dq).normalized();
      Eigen::VectorXd zh = field_model_.predictStackedFieldT(p_est, q_pert);
      for (int s = 0; s < n; ++s) zh.segment<3>(3 * s) += bm_;
      
      J.col(3 + k).segment(0, 3 * n) = (zh - z_hat) / eps_a;
      for (int s = 0; s < n; ++s) {
        J(3 * n + s, 3 + k) = (zh.segment<3>(3 * s).norm() - z_hat.segment<3>(3 * s).norm()) / eps_a;
      }
      if (has_acc) {
        J.col(3 + k).segment<3>(4 * n) = (q_pert.inverse() * g_ - q_est.inverse() * g_) / eps_a;
      }
    }

    Eigen::Matrix<double, 6, 6> H_ls = J.transpose() * J + Eigen::Matrix<double, 6, 6>::Identity() * 1e-8;
    Eigen::Matrix<double, 6, 1> b_ls = J.transpose() * r;
    Eigen::Matrix<double, 6, 1> dx = H_ls.ldlt().solve(b_ls);

    p_est += dx.segment<3>(0);
    const Eigen::Quaterniond dq_gn = deltaQuatFromOmega(dx.segment<3>(3), 1.0);
    q_est = (q_est * dq_gn).normalized();

    if (dx.segment<3>(0).norm() <= converge_m && dx.segment<3>(3).norm() <= converge_rad) {
      converged = true;
      break;
    }
  }

  if (converged) {
    p_ = p_est;
    q_ = q_est;
    tracking_enabled_ = true;
    status_ = MagneticPoseMsg::OK;
    publishStatus(stamp, "RELOCALIZED_OK");
    relocalize_start_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return true;
  }

  return false;
}

}  // namespace magnetic_localization_dv25

RCLCPP_COMPONENTS_REGISTER_NODE(magnetic_localization_dv25::MagneticLocalizationNode)
