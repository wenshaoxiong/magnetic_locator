#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <deque>

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "magnetic_interfaces/msg/sensor_array_data.hpp"
#include "magnetic_localization_dv25_interfaces/msg/algorithm_status.hpp"
#include "magnetic_localization_dv25_interfaces/msg/mag_sensor_msg.hpp"
#include "magnetic_localization_dv25_interfaces/msg/magnetic_pose.hpp"
#include "magnetic_localization_dv25_interfaces/srv/initialize_pose.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "magnetic_localization_dv25/magnetic_field_model.hpp"
#include "magnetic_localization_dv25/magnetic_map.hpp"

namespace magnetic_localization_dv25
{

class MagneticLocalizationNode : public rclcpp::Node
{
public:
  explicit MagneticLocalizationNode(const rclcpp::NodeOptions & options);

private:
  using SensorArrayData = magnetic_interfaces::msg::SensorArrayData;
  using MagSensorMsg = magnetic_localization_dv25_interfaces::msg::MagSensorMsg;
  using MagneticPoseMsg = magnetic_localization_dv25_interfaces::msg::MagneticPose;
  using AlgorithmStatusMsg = magnetic_localization_dv25_interfaces::msg::AlgorithmStatus;
  using InitializePose = magnetic_localization_dv25_interfaces::srv::InitializePose;

  void sensorArrayCallback(const SensorArrayData::SharedPtr msg);
  void hallCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void syncedRawCallback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr & hall,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu);
  void syncedMagCallback(
    const MagSensorMsg::ConstSharedPtr & mag,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu);
  void imuVarianceCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void publishOutputs(const rclcpp::Time & stamp);
  void broadcastTf(const rclcpp::Time & stamp);
  void publishStatus(const rclcpp::Time & stamp, const std::string & message);
  void logToFile(const std::string & level, const rclcpp::Time & stamp, const std::string & message);

  void handleInitializePose(
    const InitializePose::Request::SharedPtr request,
    InitializePose::Response::SharedPtr response);

  rclcpp::Subscription<SensorArrayData>::SharedPtr sensor_array_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hall_sub_;
  rclcpp::Subscription<MagSensorMsg>::SharedPtr mag_sub_unfiltered_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_variance_;

  message_filters::Subscriber<std_msgs::msg::Float64MultiArray> hall_mf_sub_;
  message_filters::Subscriber<MagSensorMsg> mag_mf_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_mf_sub_raw_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_mf_sub_mag_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> robot_pose_sub_;
  std::shared_ptr<message_filters::Cache<geometry_msgs::msg::PoseStamped>> robot_pose_cache_;

  using RawSyncPolicy = message_filters::sync_policies::ApproximateTime<
    std_msgs::msg::Float64MultiArray, sensor_msgs::msg::Imu>;
  using MagSyncPolicy = message_filters::sync_policies::ApproximateTime<MagSensorMsg, sensor_msgs::msg::Imu>;
  std::shared_ptr<message_filters::Synchronizer<RawSyncPolicy>> raw_sync_;
  std::shared_ptr<message_filters::Synchronizer<MagSyncPolicy>> mag_sync_;

  rclcpp::Publisher<MagSensorMsg>::SharedPtr mag_pub_from_raw_;
  rclcpp::Publisher<MagneticPoseMsg>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<AlgorithmStatusMsg>::SharedPtr status_pub_;

  rclcpp::Service<InitializePose>::SharedPtr init_srv_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  struct SourcePoseRecord {
    rclcpp::Time stamp;
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
  };
  std::map<std::string, std::deque<SourcePoseRecord>> source_history_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::ofstream log_file_;

  MagneticMap magnetic_map_;
  MagneticFieldModel field_model_{&magnetic_map_};
  std::shared_ptr<MagneticFieldLUT> lut_;
  bool use_lut_{false};

  std::vector<Eigen::Vector3d> sensor_positions_base_;
  std::string map_frame_;
  std::string base_frame_;
  std::string sensor_measurements_topic_;
  std::string hall_topic_;
  std::string mag_topic_;
  std::string imu_topic_;
  std::string robot_pose_topic_;
  bool sensor_measurements_in_uT_{false};

  double sync_slop_s_{0.03};

  Eigen::Vector3d p_{0.0, 0.0, 0.0};
  Eigen::Vector3d v_{0.0, 0.0, 0.0};
  Eigen::Quaterniond q_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d bg_{0.0, 0.0, 0.0};
  Eigen::Vector3d ba_{0.0, 0.0, 0.0};
  Eigen::Vector3d g_{0.0, 0.0, -9.80665};
  Eigen::Vector3d bm_{0.0, 0.0, 0.0};

  Eigen::Matrix<double, 22, 22> P_{Eigen::Matrix<double, 22, 22>::Identity()};

  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_meas_stamp_{0, 0, RCL_ROS_TIME};
  Eigen::VectorXd last_z_T_;
  double last_field_mean_norm_T_{0.0};
  nav_msgs::msg::Path path_msg_;

  uint8_t status_{MagneticPoseMsg::RELOCALIZING};
  bool tracking_enabled_{false};
  bool is_stationary_{false};
  std::deque<Eigen::Vector3d> acc_buffer_;
  std::deque<Eigen::Vector3d> gyro_buffer_;
  
  double field_jump_threshold_uT_{15.0};
  double gradient_threshold_uT_per_m_{500.0};
  double relocalize_max_gap_s_{0.5};
  double relocalize_timeout_s_{5.0};
  double markov_tau_s_{60.0};
  double adaptive_noise_threshold_m_{0.1};
  double adaptive_noise_min_dist_m_{0.01};
  rclcpp::Time relocalize_start_stamp_{0, 0, RCL_ROS_TIME};

  Eigen::Matrix3d Q_pos_{Eigen::Matrix3d::Identity() * 1e-6};
  Eigen::Matrix4d Q_quat_{Eigen::Matrix4d::Identity() * 1e-6};
  Eigen::Matrix3d R_field_{Eigen::Matrix3d::Identity() * 1e-2};
};

}  // namespace magnetic_localization_dv25
