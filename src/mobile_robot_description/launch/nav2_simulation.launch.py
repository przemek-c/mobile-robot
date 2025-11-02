import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('mobile_robot_description').find('mobile_robot_description')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='empty.world')

    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Robot state publisher
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': launch_ros.actions.Command([
                'xacro ', urdf_file
            ])
        }]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'mobile_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': os.path.join(pkg_share, 'maps', 'map.yaml'),
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )

    # RViz
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('world', default_value='empty.world', description='Gazebo world file'),
        robot_state_publisher,
        gazebo,
        spawn_entity,
        nav2_bringup,
        rviz
    ])