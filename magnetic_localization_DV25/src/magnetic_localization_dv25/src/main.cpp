#include "magnetic_localization_dv25/magnetic_localization_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<magnetic_localization_dv25::MagneticLocalizationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
