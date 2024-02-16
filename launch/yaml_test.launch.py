#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
        
    return LaunchDescription([
        
        # YAML file path
        DeclareLaunchArgument(name = "parameters_file_path",
                                description ="--params-file (name of the spot to identify)",
                                default_value = os.path.join(get_package_share_directory('ros2_yaml_parser'), 'config', 'spots.yaml')),
        # Example argument
        DeclareLaunchArgument(name = "spot_to_find",
                                description ="--spot (name of the spot to identify)",
                                default_value = "corner1"),
        
        # Parser Node example 
        Node(
            package='ros2_yaml_parser',
            executable='test.py',
            name='ros2_yaml_parser',
            output='screen',
            arguments=[
                '--params-file', LaunchConfiguration("parameters_file_path"),
                '--spot', LaunchConfiguration("spot_to_find"),
            ],
        ),
         
    ])

