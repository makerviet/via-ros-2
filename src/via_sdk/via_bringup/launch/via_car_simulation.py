import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('via_simulation_bridge_node'),
        'config',
        'params.yaml'
    )
    node = Node(
        package='via_simulation_bridge_node',
        executable='via_simulation_bridge_node',
        name='via_simulation_bridge_node',
        namespace='/simulation',
        parameters=[
            config
        ]
    )
    ld.add_action(node)

    node2 = Node(
        package='lane_line_perception_node',
        executable='lane_line_perception_node',
        name='lane_line_perception_node',
        namespace='/perception/laneline'
    )
    ld.add_action(node2)

    node3 = Node(
        package='traffic_sign_detection_node',
        executable='traffic_sign_detection_node',
        name='traffic_sign_detection_node',
        namespace='/perception/traffic_sign'
    )
    ld.add_action(node3)

    node4 = Node(
        package='visualization_node',
        executable='visualization_node',
        name='visualization_node'
    )
    ld.add_action(node4)        

    return ld
