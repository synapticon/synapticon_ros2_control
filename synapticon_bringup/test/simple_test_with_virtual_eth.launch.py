import unittest

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
import launch_testing.asserts


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
    declared_arguments.append(
        DeclareLaunchArgument(
            "test_binary_dir",
            description="Path to the test binary directory (passed by add_ros_test)",
        )
    )

    # Initialize LaunchConfigurations
    setup_script = LaunchConfiguration("setup_script")
    test_binary_dir = LaunchConfiguration("test_binary_dir")

    # Execute the setup script to create virtual ethernet interface
    setup_script_action = ExecuteProcess(
        cmd=["bash", setup_script],
        output="both",
        shell=True,
    )

    elevated_permissions_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("synapticon_ros2_control"),
                        "launch",
                        "elevated_permissions_2_dof.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"eth_device": "eno0"}.items(),
    )

    # The test executable to run (located in the build directory, passed via test_binary_dir)
    test_controller_manager_integration = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [test_binary_dir, "test_controller_manager_integration"]
            )
        ],
        output="both",
    )

    # Delay the elevated permissions launch and test after setup script completion
    # The test executable will wait for the controller_manager node internally
    delay_after_setup = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=setup_script_action,
            on_exit=[
                elevated_permissions_launch,
                test_controller_manager_integration,
            ],
        )
    )

    nodes = [
        setup_script_action,
        delay_after_setup,
        # In tests where all of the procs under tests terminate themselves, it's necessary
        # to add a dummy process not under test to keep the launch alive. launch_test
        # provides a simple launch action that does this:
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]

    return (
        LaunchDescription(declared_arguments + nodes),
        {"test_controller_manager_integration": test_controller_manager_integration},
    )


class TestControllerManagerIntegration(unittest.TestCase):
    """Active test that waits for the test executable to complete."""

    def test_wait_for_test_completion(
        self, test_controller_manager_integration, proc_info
    ):
        """Wait for the test executable to complete."""
        # Wait for the gtest process to finish
        proc_info.assertWaitForShutdown(
            process=test_controller_manager_integration,
            timeout=30.0,
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        """Check that the test executable finished with exit code 0 or was terminated by SIGINT."""
        # Accept both clean exit (0) and SIGINT (-2) since launch_testing sends SIGINT
        # to shut down processes, and the gtest may not have exited before receiving it.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK, -2],
            process="test_controller_manager_integration",
        )
