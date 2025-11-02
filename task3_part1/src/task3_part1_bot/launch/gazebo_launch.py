
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node,PushRosNamespace
from launch.actions import GroupAction
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros=get_package_share_directory("gazebo_ros")
    urdf_path1=os.path.join(get_package_share_directory("task3_part1_bot"),'urdf','my_bot.urdf')
    urdf_path2=os.path.join(get_package_share_directory('task3_part1_bot'),'urdf','aruco_bot.sdf')
    world=os.path.join(get_package_share_directory('task3_part1_bot'),'world','world.sdf')
    return LaunchDescription([
        IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    ),
    launch_arguments={'world': world}.items()
),
     IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("task3_part1_bot"), 'launch', 'state_publisher.py')
        ),
    ),
        
       GroupAction([
            PushRosNamespace(''),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'main_bot',
                    '-file', urdf_path1,
                    '-x', '-2.0',
                    '-y', '0.5',
                    '-z', '0.01',
                    '-Y','1.57'
                ],
                output='screen'
            )
        ]),
        GroupAction([
            PushRosNamespace('aruco'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace='aruco',
                arguments=[
                    '-entity', 'aruco_bot',
                    '-file', urdf_path2,
                    '-x', '4.0',
                    '-y', '0.5',
                    '-z', '0.01',
                    '-Y','3.14'
                ],
                output='screen',
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
            )
        ]),
        Node(
            package="task3_part1_bot",
            executable="aruco_detector",
            name="aruco_detector"
        ),
        Node(
            package="task3_part1_bot",
            executable="follow",
            name="follower_node"
        )
    ])
