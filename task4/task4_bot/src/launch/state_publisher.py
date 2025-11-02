from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    pkg_task4_bot = get_package_share_directory('task4_bot')
    urdf_path = os.path.join(pkg_task4_bot, 'urdf', 'my_bot.urdf')
    
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_path).read()},{"use_sim_time":True}]
        ),
         Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': False,"use_sim_time":True}]
        ),
    ])