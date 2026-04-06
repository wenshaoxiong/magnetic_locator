from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    params_file = LaunchConfiguration("params_file")
    magnetic_map_yaml = LaunchConfiguration("magnetic_map_yaml")
    use_lut = LaunchConfiguration("use_lut")
    lut_path = LaunchConfiguration("lut_path")

    default_params = os.path.join(
        get_package_share_directory("magnetic_localization_dv25"),
        "config",
        "default_params.yaml",
    )
    default_map = os.path.join(
        get_package_share_directory("magnetic_localization_dv25"),
        "config",
        "magnetic_map.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("magnetic_map_yaml", default_value=default_map),
            DeclareLaunchArgument("use_lut", default_value="false"),
            DeclareLaunchArgument("lut_path", default_value=""),
            Node(
                package="magnetic_localization_dv25",
                executable="magnetic_localization_node",
                name="magnetic_localization_node",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": sim},
                    {"magnetic_map_yaml": magnetic_map_yaml},
                    {"use_lut": use_lut},
                    {"lut_path": lut_path},
                ],
            ),
        ]
    )

