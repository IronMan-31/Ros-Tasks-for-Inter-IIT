from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    urdf_path=os.path.join(get_package_share_directory('task3_part1_bot'),'urdf','my_bot.urdf')

    nod=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description":open(urdf_path).read()},{"use_sim_time":True}]
    )

    joint_pub=Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_gui":False}]
    )

    ld=LaunchDescription()
    
    ld.add_action(nod)
    ld.add_action(joint_pub)

    return ld