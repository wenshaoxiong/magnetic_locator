import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('magnetic_locator')
    world_file_path = os.path.join(pkg_path, 'world', 'empty.world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items(),
    )

    spawn_sensor = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'my_sensor_array', '-file', os.path.join(pkg_path, 'urdf', 'sensor_array.urdf'), 
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # 关键：高度设为 0.05，确保磁铁悬浮在传感器上方，不产生遮挡或碰撞
    spawn_magnet = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'my_magnet', '-file', os.path.join(pkg_path, 'urdf', 'magnet.urdf'), 
                   '-x', '0', '-y', '0', '-z', '0.05'],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn_sensor, spawn_magnet])