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
    
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='standing',  # standing, com_tracking, foot_tracking, simple_motion
        description='Test mode for TSID controller'
    )
    
    command_frequency_arg = DeclareLaunchArgument(
        'command_frequency',
        default_value='10.0',
        description='Test command frequency in Hz'
    )

    # 1. Start the integrated control system
    integrated_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robosoccer_control'),
                'launch',
                'integrated_control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # 2. Start the test commander (delayed to ensure system is ready)
    test_commander_node = Node(
        package='robosoccer_control',
        executable='tsid_test_commander',
        name='tsid_test_commander',
        parameters=[
            {
                'test_mode': LaunchConfiguration('test_mode'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'command_frequency': LaunchConfiguration('command_frequency'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    # 3. Topic monitoring for debugging
    topic_monitor_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='topic_monitor',
        arguments=[
            '/joint_states/position[0]',
            '/joint_states/position[1]',
            '/joint_states/position[2]',
            '/robosoccer/joint_commands/data[0]',
            '/robosoccer/joint_commands/data[1]',
            '/robosoccer/joint_commands/data[2]'
        ],
        output='screen',
    )

    # Timer action to ensure proper startup order
    delayed_test_commander = TimerAction(
        period=8.0,  # Wait 8 seconds for everything to be ready
        actions=[test_commander_node]
    )
    
    delayed_monitor = TimerAction(
        period=10.0,  # Wait 10 seconds for everything to be running
        actions=[topic_monitor_node]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        test_mode_arg,
        command_frequency_arg,
        
        # Core system
        integrated_control_launch,
        
        # Test components
        delayed_test_commander,
        delayed_monitor,
    ]) 