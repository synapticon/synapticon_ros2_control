from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("synapticon_ros2_control"),
            "config",
            "seven_dof_controllers.yaml",
        ]
    )

    joint_limits = PathJoinSubstitution(
        [
            FindPackageShare("robin_moveit_config"),
            "config",
            "joint_limits.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {"aladdin.joint_limits_file": joint_limits}],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    nodes = [
        control_node,
    ]

    return LaunchDescription(nodes)
