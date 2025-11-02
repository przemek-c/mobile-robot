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
            'robot_description': launch.substitutions.Command([
                'xacro ', urdf_file
            ])
        }]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r -s empty.sdf'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-topic', '/robot_description',
            '-name', 'mobile_robot',
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

    # RViz - commented out for headless operation
    # rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    # rviz = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    # Velocity Monitor Node
    velocity_monitor = launch_ros.actions.Node(
        package='mobile_robot_description',
        executable='velocity_monitor',
        name='velocity_monitor',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('world', default_value='empty.world', description='Gazebo world file'),
        robot_state_publisher,
        gazebo,
        spawn_entity,
        velocity_monitor,
        nav2_bringup
        # rviz  # commented out for headless operation
    ])