from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("synapticon_ros2_control"),
                    "urdf",
                    "single_dof_in_world.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("synapticon_ros2_control"),
            "config",
            "single_dof_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("synapticon_ros2_control"), "config", "single_dof.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    inactive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--inactive",
            "forward_torque_controller",
            "forward_velocity_controller",
            "forward_position_controller",
            "quick_stop_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_inactive_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=inactive_controller_spawner,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    nodes = [
        robot_state_pub_node,
        inactive_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_inactive_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
