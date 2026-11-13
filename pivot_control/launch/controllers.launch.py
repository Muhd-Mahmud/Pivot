from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Spawn the ros2_control controllers.

    Run this after the robot is spawned in Gazebo — the controller
    manager lives inside the Gazebo plugin, so it does not exist
    until the model is loaded.
    """

    # Publishes /joint_states from the hardware interface, which
    # robot_state_publisher turns into TF for the wheel joints.
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Takes Float64MultiArray on /wheel_effort_controller/commands
    # and applies it as joint torque. [left, right].
    wheel_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_effort_controller"],
        output="screen",
    )

    return LaunchDescription([
        joint_state_broadcaster,
        wheel_effort_controller,
    ])