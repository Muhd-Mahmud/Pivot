import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pivot_description = get_package_share_directory('pivot_description')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(pivot_description, 'urdf', 'pivot.urdf.xacro'),
        description='Path to robot urdf',
    )

    # Gazebo resolves package:// paths relative to this. Without it the
    # robot spawns with working physics and no visible geometry.
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(pivot_description).parent.resolve())],
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py',
            ]
        ),
        launch_arguments=[('gz_args', [' -v 4', ' empty.sdf'])],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'pivot', '-z', '0.05'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Publishes /joint_states from the hardware interface so
    # robot_state_publisher can turn wheel rotation into TF.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
    )

    # Takes Float64MultiArray on /wheel_effort_controller/commands
    # and writes it to the wheel joints as torque. [left, right].
    wheel_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_effort_controller'],
        output='screen',
    )

    balance_node = Node(
        package='pivot_control',
        executable='balance_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Startup chain. The controller manager lives inside Gazebo's update
    # loop, so activating a controller needs Gazebo actually stepping.
    # Firing the spawner too early gives "Switch controller timed out
    # after 5 seconds" — hence the delays between each stage.
    return LaunchDescription(
        [
            model_arg,
            gazebo_resource_path,
            robot_state_publisher,
            gazebo,
            spawn,
            bridge,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=[
                        TimerAction(period=3.0, actions=[joint_state_broadcaster])
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[
                        TimerAction(period=2.0, actions=[wheel_effort_controller])
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wheel_effort_controller,
                    on_exit=[TimerAction(period=1.0, actions=[balance_node])],
                )
            ),
        ]
    )
