from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    map_file = os.path.join(pkg_dir, 'maps', 'map_1760536528.yaml')

    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = os.path.join(turtlebot3_cartographer_prefix, 'config')

    is_mapping = PythonExpression(["'", LaunchConfiguration('mode'), "' == \"mapping\""])
    is_navigation = PythonExpression(["'", LaunchConfiguration('mode'), "' == \"navigation\""])

    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'turtlebot3_lds_2d.lua'
        ],
        condition=IfCondition(is_mapping)
    )

    occ_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_cartographer_prefix, 'launch', 'occupancy_grid.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': '0.05',
            'publish_period_sec': '1.0'
        }.items(),
        condition=IfCondition(is_mapping)
    )

    navig_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('navig_params'),
            'map': map_file
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

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        mapping_params_arg,
        navig_params_arg,
        carto_node,
        occ_grid,
        navig_node,
        rviz_node
    ])
