import os

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Locate the installed share directory of the pivot_description package
    # (this is where the URDF/xacro files and meshes live after colcon build)
    pivot_description = get_package_share_directory("pivot_description")

    # Launch argument: lets the user override which URDF/xacro file to load
    # from the command line, e.g. `ros2 launch ... model:=/path/to/other.xacro`
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            pivot_description, "urdf", "pivot.urdf.xacro"
        ),
        description="Path to robot urdf"
    )

    # Gazebo needs to know where to find mesh/model resources referenced by the
    # URDF (STL files etc). RViz resolves package:// on its own; Gazebo does not,
    # so point it at the directory *containing* the package. Skip this and the
    # robot spawns with working physics and no visible geometry.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(pivot_description).parent.resolve())
        ]
    )

    # Feed the xacro file into the xacro compiler and catch whatever
    # URDF string comes out the other side. Alchemy, basically.
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model")
        ]), value_type=str
    )

    # This node's whole job: take joint states and broadcast the TF tree
    # so every part of the robot knows where it stands, literally.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    # Bring up Gazebo Sim (server + GUI) by including ros_gz_sim's own launch
    # file, rather than re-implementing the Gazebo bringup logic here.
    # empty.sdf ships with Gazebo — swap in a custom world later when the
    # physics settings need tuning.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4", " empty.sdf"])
        ]
    )

    # Entering the unknown: drop Pivot into the freshly loaded empty world.
    # Spawning 50 mm up matters here — this is an inverted pendulum on two
    # contact points, so starting it interpenetrated with the floor tends to
    # end with the robot launched across the map. Let it fall and settle.
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "pivot",
                   "-z", "0.05"],
    )

    # Bridge topics between ROS 2 and Gazebo Transport. Here we bridge just
    # the simulation clock (/clock) so ROS 2 nodes using use_sim_time can
    # stay synced with Gazebo's simulated time. Sensors and cmd_vel get added
    # to this list once the physics is trusted.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    # Hand everything to `ros2 launch` in order and hope for the best:
    # args declared, env set, robot described, Gazebo up, robot spawned,
    # clocks synced. Pivot is expected to tip over — there's no controller yet.
    # What matters is that it falls like an object, not like a glitch.
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge

    ]
    )