import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Run this from the root of your workspace
    # Used an argument to pass the path, or default to a known relative location.
    
    return LaunchDescription([
        Node(
            package='hand_controller',
            executable='hand_controller',
            name='hand_controller',
            output='screen',
            parameters=[{
                'yaml_path': os.path.join(os.getcwd(), 'data', 'motion.yaml')
            }]
        )
    ])