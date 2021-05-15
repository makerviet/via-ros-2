import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('via_camera_nodes'),
        'config',
        'params.yaml'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('via_camera_nodes') + '/config/camera.yaml'),
        Node(
            package='via_camera_nodes',
            executable='via_camera_node',
            namespace='/camera',
            parameters=[
                {"camera_calibration_file": LaunchConfiguration('camera_calibration_file')},
                config
            ]
        )
    ])
