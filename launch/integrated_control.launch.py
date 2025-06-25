#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for all nodes'
    )
    
    robot_description_package_arg = DeclareLaunchArgument(
        'robot_description_package',
        default_value='my_robot_description',
        description='Package containing robot URDF'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='robot.urdf',
        description='URDF file name'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Direct path to URDF file (overrides package and file name)'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='1000.0',
        description='TSID control frequency in Hz'
    )
    
    trajectory_duration_arg = DeclareLaunchArgument(
        'trajectory_duration',
        default_value='0.05',
        description='Trajectory duration for bridge (seconds)'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='10.0',
        description='Maximum joint velocity (rad/s)'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='50.0',
        description='Maximum joint acceleration (rad/s^2)'
    )

    # 1. Start mujoco_ros2_control system first
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'launch',
                'mujoco.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # 2. Start TSID controller (delayed to ensure mujoco is ready)
    tsid_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robosoccer_control'),
                'launch',
                'tsid_control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description_package': LaunchConfiguration('robot_description_package'),
            'urdf_file': LaunchConfiguration('urdf_file'),
            'urdf_path': LaunchConfiguration('urdf_path'),
            'control_frequency': LaunchConfiguration('control_frequency'),
        }.items()
    )

    # 3. Enhanced TSID to Trajectory Bridge (delayed to ensure both systems are ready)
    enhanced_bridge_node = Node(
        package='robosoccer_control',
        executable='tsid_to_trajectory_bridge',
        name='enhanced_tsid_bridge',
        parameters=[
            {
                'trajectory_duration': LaunchConfiguration('trajectory_duration'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'max_acceleration': LaunchConfiguration('max_acceleration'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    # 4. Monitoring node for the complete system
    monitoring_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='control_monitoring',
        arguments=[
            '/joint_states/position[0]',
            '/joint_states/position[1]',
            '/joint_states/position[2]',
            '/bridge_status/data[0]',
            '/bridge_status/data[1]'
        ],
        output='screen',
    )

    # 5. RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='robot_visualization',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'config',
                'robot_visualization.rviz'
            ])
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    # Timer actions to ensure proper startup order
    delayed_tsid = TimerAction(
        period=2.0,  # Wait 2 seconds for mujoco to start
        actions=[tsid_control_launch]
    )
    
    delayed_bridge = TimerAction(
        period=4.0,  # Wait 4 seconds for both systems to be ready
        actions=[enhanced_bridge_node]
    )
    
    delayed_monitoring = TimerAction(
        period=6.0,  # Wait 6 seconds for everything to be running
        actions=[monitoring_node]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        robot_description_package_arg,
        urdf_file_arg,
        urdf_path_arg,
        control_frequency_arg,
        trajectory_duration_arg,
        max_velocity_arg,
        max_acceleration_arg,
        
        # Core systems
        mujoco_launch,
        delayed_tsid,
        delayed_bridge,
        
        # Optional visualization
        rviz_node,
        delayed_monitoring,
    ]) 