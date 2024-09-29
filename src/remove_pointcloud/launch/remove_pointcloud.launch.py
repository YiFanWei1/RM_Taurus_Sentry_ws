import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

sys.path.append(os.path.join(get_package_share_directory('remove_pointcloud'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node,SetParameter
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription 
 

    static_map_to_odom_ = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '/map', 'odom'] 

    )
    node_params = os.path.join(
        get_package_share_directory('remove_pointcloud'), 'config', 'remove_pointcloud.yaml')
    remove_pointcloud_node_ = Node(
        package='remove_pointcloud',
        executable='remove_pointcloud_node',
        output='screen',
        parameters=[node_params])
    
    return LaunchDescription([
        remove_pointcloud_node_,
        # static_map_to_odom_
    ])
