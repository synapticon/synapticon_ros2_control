from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import launch_testing


def generate_test_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "setup_script",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("synapticon_ros2_control"),
                    "scripts",
                    "setup_test_environment.sh",
                ]
            ),
            description="Path to the setup script for creating virtual ethernet interface",
        )
    )

    # Initialize LaunchConfigurations
    setup_script = LaunchConfiguration("setup_script")

    # Execute the setup script to create virtual ethernet interface
    setup_script_action = ExecuteProcess(
        cmd=["bash", setup_script],
        output="both",
        shell=True,
    )

    # Include the elevated permissions 1 DOF launch file
    elevated_permissions_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("synapticon_ros2_control"),
                        "launch",
                        "elevated_permissions_1_dof.launch.py",
                    ]
                )
            ]
        )
    )

    # Delay the elevated permissions launch after setup script completion
    delay_elevated_permissions_after_setup = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=setup_script_action,
            on_exit=[elevated_permissions_launch],
        )
    )

    nodes = [
        setup_script_action,
        delay_elevated_permissions_after_setup,
        # In tests where all of the procs under tests terminate themselves, it's necessary
        # to add a dummy process not under test to keep the launch alive. launch_test
        # provides a simple launch action that does this:
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]

    return LaunchDescription(declared_arguments + nodes)
