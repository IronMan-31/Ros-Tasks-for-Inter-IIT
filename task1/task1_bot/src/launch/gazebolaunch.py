from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Paths Setup ---
    pkg_task1_bot = get_package_share_directory('task1_bot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    urdf_path = os.path.join(pkg_task1_bot, 'urdf', 'rover_description.urdf')
    world_path = os.path.join(pkg_task1_bot, 'world', 'model.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'task1_bot',
                   '-x', '0.5',   
                   '-y', '0.5',
                   '-z', '2'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])