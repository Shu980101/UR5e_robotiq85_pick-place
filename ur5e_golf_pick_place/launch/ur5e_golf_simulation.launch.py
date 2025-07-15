#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare('ur5e_golf_pick_place').find('ur5e_golf_pick_place')
    world_file = os.path.join(pkg_share, 'worlds', 'golf_workspace.world')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz'
    )
    
    # Use the official UR5e launch file
    ur5e_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur5e.launch.py'
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': 'true',
            'launch_rviz': 'false',
            'robot_ip': '192.168.1.100'
        }.items()
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
            'gui': LaunchConfiguration('gui'),
            'server': 'true',
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot in Gazebo (get robot description from the UR control launch)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ur5e_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.75'
        ],
        output='screen'
    )
    
    # RViz
    rviz_config = os.path.join(pkg_share, 'config', 'ur5e_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )
    
    # Pick and place controller (delayed start)
    pick_place_controller = TimerAction(
        period=15.0,  # Wait 15 seconds for everything to initialize
        actions=[
            Node(
                package='ur5e_golf_pick_place',
                executable='pick_place_controller',
                name='pick_place_controller',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        ur5e_launch,
        gazebo,
        TimerAction(
            period=5.0,  # Wait for UR control to start
            actions=[spawn_robot]
        ),
        pick_place_controller,
        rviz_node,
    ])