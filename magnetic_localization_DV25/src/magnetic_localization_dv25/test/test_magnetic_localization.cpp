#define private public
#include "magnetic_localization_dv25/magnetic_localization_node.hpp"
#undef private

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <thread>

namespace
{

class RclcppEnv : public ::testing::Environment
{
public:
  void SetUp() override
  {
    int argc = 0;
    char ** argv = nullptr;
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

std::string writeTempMagMapYaml()
{
  const auto dir = std::filesystem::temp_directory_path() / "magnetic_localization_dv25_test";
  std::filesystem::create_directories(dir);
  const auto path = dir / "magnetic_map.yaml";
  std::ofstream f(path.string(), std::ios::binary);
  f << "sources:\n";
  f << "  - id: magnet_1\n";
  f << "    position_m: [0.0, 0.0, 0.0]\n";
  f << "    moment: [0.0, 0.0, 1.0]\n";
  f << "    spherical_harmonics_path: \"\"\n";
  f.close();
  return path.string();
}

sensor_msgs::msg::Imu makeImuMsg(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = "base_link";
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;
  return imu;
}

magnetic_localization_dv25_interfaces::msg::MagSensorMsg makeMagMsg(
  const rclcpp::Time & stamp, const std::vector<Eigen::Vector3d> & sensors_base, const Eigen::VectorXd & z_T)
{
  magnetic_localization_dv25_interfaces::msg::MagSensorMsg mag;
  mag.header.stamp = stamp;
  mag.header.frame_id = "base_link";
  mag.sensor_positions.resize(sensors_base.size());
  mag.magnetic_field.resize(sensors_base.size());
  for (size_t i = 0; i < sensors_base.size(); ++i) {
    mag.sensor_positions[i].x = sensors_base[i].x();
    mag.sensor_positions[i].y = sensors_base[i].y();
    mag.sensor_positions[i].z = sensors_base[i].z();
    mag.magnetic_field[i].x = z_T.segment<3>(3 * static_cast<int>(i)).x();
    mag.magnetic_field[i].y = z_T.segment<3>(3 * static_cast<int>(i)).y();
    mag.magnetic_field[i].z = z_T.segment<3>(3 * static_cast<int>(i)).z();
  }
  return mag;
}

}  // namespace

TEST(MagneticMap, LoadFromYaml)
{
  magnetic_localization_dv25::MagneticMap map;
  ASSERT_TRUE(map.loadFromYaml(writeTempMagMapYaml()));
  ASSERT_EQ(map.sources().size(), 1u);
  EXPECT_EQ(map.sources()[0].id, "magnet_1");
  EXPECT_NEAR(map.sources()[0].position_m.x(), 0.0, 1e-12);
}

TEST(MagneticFieldModel, PredictNonZero)
{
  magnetic_localization_dv25::MagneticMap map;
  ASSERT_TRUE(map.loadFromYaml(writeTempMagMapYaml()));
  magnetic_localization_dv25::MagneticFieldModel model(&map);
  model.setSensorsBase({Eigen::Vector3d(0.1, 0.0, 0.0)});
  const auto z = model.predictStackedFieldT(Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Quaterniond::Identity());
  EXPECT_EQ(z.size(), 3);
  EXPECT_GT(z.norm(), 0.0);
}

TEST(Node, PublishPoseMessage)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml())});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);

  std::optional<magnetic_localization_dv25_interfaces::msg::MagneticPose> last;
  auto sub = node->create_subscription<magnetic_localization_dv25_interfaces::msg::MagneticPose>(
    "/magnetic_pose", rclcpp::SensorDataQoS(),
    [&](magnetic_localization_dv25_interfaces::msg::MagneticPose::ConstSharedPtr msg) { last = *msg; });

  node->tracking_enabled_ = true;
  node->status_ = magnetic_localization_dv25_interfaces::msg::MagneticPose::OK;
  const auto stamp = node->now();
  node->publishOutputs(stamp);

  for (int i = 0; i < 20 && !last.has_value(); ++i) {
    ex.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(last.has_value());
  EXPECT_EQ(last->status, magnetic_localization_dv25_interfaces::msg::MagneticPose::OK);
  EXPECT_EQ(last->header.frame_id, "map");
}

TEST(Node, BroadcastTf)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml())});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);

  std::optional<tf2_msgs::msg::TFMessage> last;
  auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(10),
    [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { last = *msg; });

  node->tracking_enabled_ = true;
  node->status_ = magnetic_localization_dv25_interfaces::msg::MagneticPose::OK;
  node->p_ = Eigen::Vector3d(0.1, 0.2, 0.3);
  node->q_ = Eigen::Quaterniond::Identity();

  const auto stamp = node->now();
  node->broadcastTf(stamp);
  for (int i = 0; i < 50 && !last.has_value(); ++i) {
    ex.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(last.has_value());
  ASSERT_FALSE(last->transforms.empty());
  EXPECT_EQ(last->transforms[0].header.frame_id, "map");
  EXPECT_EQ(last->transforms[0].child_frame_id, "base_link");
}

TEST(Node, FieldJumpTriggersRelocalize)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml())});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);

  const auto t0 = rclcpp::Time(1, 0, RCL_ROS_TIME);
  node->last_imu_stamp_ = t0;
  node->tracking_enabled_ = true;
  node->status_ = magnetic_localization_dv25_interfaces::msg::MagneticPose::OK;

  Eigen::VectorXd z1 = Eigen::VectorXd::Zero(3 * static_cast<int>(node->sensor_positions_base_.size()));
  Eigen::VectorXd z2 = z1;
  z2.segment<3>(0) = Eigen::Vector3d(1000.0e-6, 0.0, 0.0);

  auto imu = std::make_shared<sensor_msgs::msg::Imu>(makeImuMsg(t0 + rclcpp::Duration::from_seconds(0.01)));
  auto mag1 = std::make_shared<magnetic_localization_dv25_interfaces::msg::MagSensorMsg>(
    makeMagMsg(imu->header.stamp, node->sensor_positions_base_, z1));
  node->syncedMagCallback(mag1, imu);

  imu = std::make_shared<sensor_msgs::msg::Imu>(makeImuMsg(imu->header.stamp + rclcpp::Duration::from_seconds(0.01)));
  auto mag2 = std::make_shared<magnetic_localization_dv25_interfaces::msg::MagSensorMsg>(
    makeMagMsg(imu->header.stamp, node->sensor_positions_base_, z2));
  node->syncedMagCallback(mag2, imu);

  EXPECT_EQ(node->status_, magnetic_localization_dv25_interfaces::msg::MagneticPose::RELOCALIZING);
  EXPECT_FALSE(node->tracking_enabled_);
}

TEST(Node, GradientAnomalyTriggersRelocalize)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml()),
     rclcpp::Parameter("gradient_threshold_uT_per_m", 10.0)});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);

  const auto t0 = rclcpp::Time(1, 0, RCL_ROS_TIME);
  node->last_imu_stamp_ = t0;
  node->tracking_enabled_ = true;
  node->status_ = magnetic_localization_dv25_interfaces::msg::MagneticPose::OK;

  Eigen::VectorXd z = Eigen::VectorXd::Zero(3 * static_cast<int>(node->sensor_positions_base_.size()));
  z.segment<3>(0) = Eigen::Vector3d(1000.0e-6, 0.0, 0.0);
  auto imu = std::make_shared<sensor_msgs::msg::Imu>(makeImuMsg(t0 + rclcpp::Duration::from_seconds(0.01)));
  auto mag = std::make_shared<magnetic_localization_dv25_interfaces::msg::MagSensorMsg>(
    makeMagMsg(imu->header.stamp, node->sensor_positions_base_, z));
  node->syncedMagCallback(mag, imu);

  EXPECT_EQ(node->status_, magnetic_localization_dv25_interfaces::msg::MagneticPose::RELOCALIZING);
  EXPECT_FALSE(node->tracking_enabled_);
}

TEST(Node, RelocalizeTimeoutSetsLost)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml()),
     rclcpp::Parameter("relocalize_timeout_s", 0.0)});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);

  const auto t0 = rclcpp::Time(1, 0, RCL_ROS_TIME);
  node->last_imu_stamp_ = t0;
  node->tracking_enabled_ = false;
  node->status_ = magnetic_localization_dv25_interfaces::msg::MagneticPose::RELOCALIZING;
  node->relocalize_start_stamp_ = t0;

  Eigen::VectorXd z = Eigen::VectorXd::Zero(3 * static_cast<int>(node->sensor_positions_base_.size()));
  auto imu = std::make_shared<sensor_msgs::msg::Imu>(makeImuMsg(t0 + rclcpp::Duration::from_seconds(0.01)));
  auto mag = std::make_shared<magnetic_localization_dv25_interfaces::msg::MagSensorMsg>(
    makeMagMsg(imu->header.stamp, node->sensor_positions_base_, z));
  node->syncedMagCallback(mag, imu);

  EXPECT_EQ(node->status_, magnetic_localization_dv25_interfaces::msg::MagneticPose::LOST);
}

TEST(Node, InitializePoseFailsWithoutMeasurement)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml())});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);

  auto req = std::make_shared<magnetic_localization_dv25_interfaces::srv::InitializePose::Request>();
  auto res = std::make_shared<magnetic_localization_dv25_interfaces::srv::InitializePose::Response>();
  node->handleInitializePose(req, res);
  EXPECT_FALSE(res->success);
}

TEST(Node, InitializePoseSucceedsWithPerfectMeasurement)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml())});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);

  const Eigen::Vector3d p0(0.05, 0.01, 0.10);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  node->last_z_T_ = node->field_model_.predictStackedFieldT(p0, q0);
  node->last_meas_stamp_ = rclcpp::Time(2, 0, RCL_ROS_TIME);

  auto req = std::make_shared<magnetic_localization_dv25_interfaces::srv::InitializePose::Request>();
  req->initial_guess.header.stamp = node->last_meas_stamp_;
  req->initial_guess.header.frame_id = "map";
  req->initial_guess.pose.position.x = p0.x();
  req->initial_guess.pose.position.y = p0.y();
  req->initial_guess.pose.position.z = p0.z();
  req->initial_guess.pose.orientation.w = q0.w();
  req->initial_guess.pose.orientation.x = q0.x();
  req->initial_guess.pose.orientation.y = q0.y();
  req->initial_guess.pose.orientation.z = q0.z();
  req->max_iterations = 50;
  req->rpy_search_deg = 1.0;
  req->position_search_m = 0.01;
  req->position_converge_m = 0.02;
  req->angle_converge_deg = 2.0;

  auto res = std::make_shared<magnetic_localization_dv25_interfaces::srv::InitializePose::Response>();
  node->handleInitializePose(req, res);
  EXPECT_TRUE(res->success);
  EXPECT_TRUE(node->tracking_enabled_);
  EXPECT_EQ(node->status_, magnetic_localization_dv25_interfaces::msg::MagneticPose::OK);
}

TEST(Node, EkfUpdateMovesTowardTruth)
{
  auto opts = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("magnetic_map_yaml", writeTempMagMapYaml())});
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(opts);

  const Eigen::Vector3d p_true(0.06, -0.02, 0.12);
  const Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
  const Eigen::VectorXd z_T = node->field_model_.predictStackedFieldT(p_true, q_true);

  node->p_ = p_true + Eigen::Vector3d(0.02, 0.0, 0.0);
  node->q_ = q_true;
  node->tracking_enabled_ = true;
  node->status_ = magnetic_localization_dv25_interfaces::msg::MagneticPose::OK;
  node->last_imu_stamp_ = rclcpp::Time(1, 0, RCL_ROS_TIME);

  auto imu = std::make_shared<sensor_msgs::msg::Imu>(makeImuMsg(node->last_imu_stamp_ + rclcpp::Duration::from_seconds(0.01)));
  auto mag = std::make_shared<magnetic_localization_dv25_interfaces::msg::MagSensorMsg>(
    makeMagMsg(imu->header.stamp, node->sensor_positions_base_, z_T));
  node->syncedMagCallback(mag, imu);

  EXPECT_LT((node->p_ - p_true).norm(), 0.03);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RclcppEnv());
  return RUN_ALL_TESTS();
}

