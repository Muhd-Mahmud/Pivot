import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory('pivot_description')
    gazebo_share = get_package_share_directory('ros_gz_sim')
    model = os.path.join(description, 'urdf', 'pivot.urdf.xacro')

    resource_path = os.pathsep.join(
        filter(
            None,
            [
                os.path.dirname(description),
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            ],
        )
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', model]), value_type=str
                ),
                'use_sim_time': True,
            }
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Start before spawning, ready to process the first IMU messages.
    balance = Node(
        package='pivot_control',
        executable='balance_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                # Simulation candidate: validate speed control before position hold.
                'enable_velocity_control': True,
                'enable_station_hold': False,
                'velocity_pitch_sign': 1.0,
                'speed_kp': 0.12,
                'max_pitch_offset': 0.02,
                'speed_effort_kp': 0.0,
                'max_speed_effort': 0.0,
                'position_kp': 0.20,
                'max_hold_speed': 0.03,
                'enable_yaw_damping': False,
            }
        ],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'pivot',
            # Derived from your geometry: wheels touch ground at root z=0.
            '-z',
            '0.0',
        ],
    )

    effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'wheel_effort_controller',
            '-c',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
    )

    joint_states = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '-c',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
    )

    def after_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Robot spawn failed'))]
        # No fixed delay and no dependency on broadcaster activation.
        return [effort_controller, joint_states]

    def after_effort_controller(event, context):
        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(reason='Wheel effort controller activation failed')
                )
            ]
        return []

    return LaunchDescription(
        [
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
            # Register handlers before starting their target processes.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=after_spawn,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=effort_controller,
                    on_exit=after_effort_controller,
                )
            ),
            state_publisher,
            bridge,
            balance,
            gazebo,
            spawn,
        ]
    )
