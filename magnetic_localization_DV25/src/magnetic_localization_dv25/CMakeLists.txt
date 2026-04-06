cmake_minimum_required(VERSION 3.8)
project(magnetic_localization_dv25)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(magnetic_interfaces REQUIRED)
find_package(magnetic_localization_dv25_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

add_library(${PROJECT_NAME} SHARED
  src/magnetic_localization_component.cpp
  src/magnetic_map.cpp
  src/magnetic_field_model.cpp
  src/magnetic_field_lut.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  magnetic_interfaces
  magnetic_localization_dv25_interfaces
  yaml-cpp
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

rclcpp_components_register_nodes(${PROJECT_NAME} "magnetic_localization_dv25::MagneticLocalizationNode")

add_executable(magnetic_localization_node src/main.cpp)
target_link_libraries(magnetic_localization_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  magnetic_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_msgs REQUIRED)

  ament_add_gtest(test_magnetic_localization test/test_magnetic_localization.cpp)
  target_link_libraries(test_magnetic_localization ${PROJECT_NAME})
  ament_target_dependencies(test_magnetic_localization
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_msgs
    magnetic_interfaces
    magnetic_localization_dv25_interfaces
  )
endif()

ament_package()
