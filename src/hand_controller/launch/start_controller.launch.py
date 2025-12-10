import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This logic finds the directory dynamically!
    # Note: For this to work, you must install the 'data' folder in CMakeLists.txt
    # OR simpler: Just point to the absolute path using Python's 'os' module relative to where you run it.
    
    # Assuming you run this from the root of your workspace or repo:
    # We will use an argument to pass the path, or default to a known relative location.
    
    return LaunchDescription([
        Node(
            package='hand_controller',
            executable='hand_controller',
            name='hand_controller',
            output='screen',
            parameters=[{
                # This is the magic. It sets the C++ parameter.
                # You can change this path or pass it as an argument.
                'yaml_path': os.path.join(os.getcwd(), 'data', 'motion.yaml')
            }]
        )
    ])