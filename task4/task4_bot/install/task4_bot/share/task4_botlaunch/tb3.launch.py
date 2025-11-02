from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('task4_bot')

    mode_arg = DeclareLaunchArgument('mode', default_value='mapping')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    mapping_params_arg = DeclareLaunchArgument('mapping_params', default_value=os.path.join(pkg_dir, 'config', 'param.yaml'))
    navig_params_arg = DeclareLaunchArgument('navig_params', default_value=os.path.join(pkg_dir, 'config', 'botnav.yaml'))

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    is_mapping = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])
    is_navigation = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'navigation'"])

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_tool',
        output='screen',
        parameters=[
            LaunchConfiguration('mapping_params'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(is_mapping)
    )

    navig_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('navig_params'),

            'map': ''
        }.items(),
        condition=IfCondition(is_navigation)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(is_navigation)  
    )

    ld = LaunchDescription()

    ld.add_action(mode_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(mapping_params_arg)
    ld.add_action(navig_params_arg)

    ld.add_action(slam_node)
    ld.add_action(navig_node)
    ld.add_action(rviz_node)

    return ld
