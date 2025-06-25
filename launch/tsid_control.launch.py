#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robosoccer_control'),
            'config',
            'tsid_config.yaml'
        ]),
        description='Path to the TSID controller configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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

    def launch_setup(context, *args, **kwargs):
        # TSID Controller Node
        tsid_controller_node = Node(
            package='robosoccer_control',
            executable='tsid_controller_node',
            name='tsid_controller_node',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description_package': LaunchConfiguration('robot_description_package'),
                    'urdf_file': LaunchConfiguration('urdf_file'),
                    'urdf_path': LaunchConfiguration('urdf_path'),
                    'control_frequency': LaunchConfiguration('control_frequency'),
                }
            ],
            output='screen',
            emulate_tty=True,
        )
        
        # TSID to Trajectory Bridge Node
        bridge_node = Node(
            package='robosoccer_control',
            executable='tsid_to_trajectory_bridge',
            name='tsid_to_trajectory_bridge',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
            emulate_tty=True,
        )
        
        return [tsid_controller_node, bridge_node]

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        robot_description_package_arg,
        urdf_file_arg,
        urdf_path_arg,
        control_frequency_arg,
        OpaqueFunction(function=launch_setup)
    ]) 