from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    path=os.path.join(get_package_share_directory("task1_bot"),"urdf","rover_description.urdf")
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="state_publisher",
            parameters=[
                {"robot_description":open(path).read()},{"use_sim_time":True}
            ]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{"use_sim_time": True}]
        )    ])
