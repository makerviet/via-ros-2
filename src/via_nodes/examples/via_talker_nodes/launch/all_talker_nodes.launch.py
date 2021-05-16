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
        get_package_share_directory('via_talker_nodes'),
        'config',
        'params.yaml'
    )

    node = Node(
        package='via_talker_nodes',
        executable='via_talker_node',
        name='via_talker_node',
        namespace='/via_examples',
        parameters=[
            config
        ]
    )
    ld.add_action(node)

    node2 = Node(
        package='via_talker_nodes',
        executable='via_talker_2ints_node',
        name='via_talker_2ints_node',
        namespace='/via_examples',
        parameters=[
            config
        ]
    )
    ld.add_action(node2)

    return ld
