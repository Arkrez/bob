import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("quadruped_sim")

    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", "quadruped.urdf.xacro"]
    )

    world_file = PathJoinSubstitution(
        [pkg_share, "worlds", "quadruped_world.sdf"]
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file
        ]),
        value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": [world_file, " -r -s"]
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "quadruped",
            "-z", "0.2",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    leg_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_position_controller"],
        output="screen",
    )

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                leg_position_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,
        delayed_controllers,
    ])