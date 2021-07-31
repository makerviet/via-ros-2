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

    # node2 = Node(
    #     package='simple_lane_line_perception_node',
    #     executable='simple_lane_line_perception_node',
    #     name='simple_lane_line_perception_node',
    #     namespace='/perception/laneline'
    # )
    # ld.add_action(node2)    

    return ld
