from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path=os.path.join(get_package_share_directory('task2_bot'),'urdf','Arm_Urdf.urdf')

    state_pub=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{"robot_description":open(urdf_path).read()},{"use_sim_time":True}]
    )

    joint_pub=Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher_gui",
        parameters=[{'use_gui':False}]
    )

    ld=LaunchDescription()

    ld.add_action(state_pub)

    return ld