from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_task4_bot = get_package_share_directory('task4_bot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    urdf_path = os.path.join(pkg_task4_bot, 'urdf', 'my_bot.urdf')
    # urdf_path = os.path.join(pkg_task4_bot, 'urdf', 'my_bot.urdf')
    world_path = os.path.join(
        get_package_share_directory('task4_bot'),
        'Custom_world',
        'model.world'
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    publisher=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_task4_bot,'launch','state_publisher.py')
        )
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_path, 
                   '-entity', 'task4_bot',
                   '-x', '0.5',   
                   '-y', '0.5',
                   '-z', '2'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        publisher,
        spawn_entity
    ])