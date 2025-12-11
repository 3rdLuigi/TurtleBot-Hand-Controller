import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():   
    base_path = os.getcwd() 
    motion_file = os.path.join(base_path, 'data', 'motion.yaml')
    safety_file = os.path.join(base_path, 'data', 'safety.yaml')

    return LaunchDescription([
        # Node 1: The Safety Checker
        Node(
            package='hand_controller',
            executable='safety_checker',
            name='safety_checker',
            output='screen',
            parameters=[{'safety_yaml_path': safety_file}]
        ),
        # Node 2: The Hand Controller
        Node(
            package='hand_controller',
            executable='hand_controller',
            name='hand_controller',
            output='screen',
            parameters=[{
                'motion_yaml_path': motion_file,
                'safety_yaml_path': safety_file
            }]
        )
    ])