# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

# Bring up an example robot with Synapticon drives.
#
# The robot comes from a robot definition: config/example_<robot>.yaml of this package, with its controllers in
# config/example_<robot>_controllers.yaml. The drives come up without power. Enable them with the drive controller.
#
# Without hardware, on simulated drives:
#   ros2 launch synapticon_bringup example_synapticon.launch.py robot:=geared hal:=mock_cia402
#
# On real drives, in impedance mode:
#   ros2 launch synapticon_bringup example_synapticon.launch.py impedance:=true network_interface:=<interface>
#
# Then move a joint, in CSP with one position per joint in rad:
#   ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.2]}"
#
# or in impedance mode with a soft spring:
#   ros2 topic pub --once /motor_command_controller/commands synapticon_control_msgs/msg/MotorCommand \
#     "{name: [joint_1], position: [1.2], velocity: [0.0], torque_feedforward: [0.0], stiffness: [5.0], damping: [0.2]}"
#
# `ros2 launch synapticon_bringup example_synapticon.launch.py --show-args` lists every argument.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration as Arg,
    IfElseSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from synapticon_bringup.transmissions import prepare_launch_files

# How long the spawner waits for the controller manager. It must outlast the bring-up, which can block the controller
# manager for readiness_timeout_s of the robot definition (20 s in the examples) until every drive streams.
CONTROLLER_MANAGER_WAIT_S = "60"


def _installed(package, *path):
    """A file that `package` installs."""
    return PathJoinSubstitution([FindPackageShare(package), *path])


def _example_config(ending):
    """The file config/example_<robot><ending> of this package."""
    return _installed(
        "synapticon_bringup", "config", ["example_", Arg("robot"), ending]
    )


def _declarations():
    """All launch arguments: name, default, allowed values and help text."""
    command_controller = IfElseSubstitution(
        Arg("impedance"), "motor_command_controller", "forward_position_controller"
    )
    slave_config = _installed(
        "synapticon_hardware", "config", "example_somanet_circulo_safe_motion.yaml"
    )
    arguments = [
        (
            "robot",
            "1dof",
            None,
            "Example robot: 1dof, 2dof, 28dof, geared, or one of your own in config/.",
        ),
        (
            "robot_config",
            _example_config(".yaml"),
            None,
            "Robot definition. Taken from robot when not given.",
        ),
        (
            "controllers_file",
            _example_config("_controllers.yaml"),
            None,
            "Controller configuration, from robot.",
        ),
        (
            "hal",
            "ethercat_soem",
            None,
            "ethercat_soem for real drives, mock or mock_cia402 for none.",
        ),
        (
            "network_interface",
            "",
            None,
            "Network interface of the drives. Empty: the robot definition's bus entry.",
        ),
        (
            "slave_config",
            slave_config,
            None,
            "slave_config of every joint that names none in the robot definition.",
        ),
        (
            "impedance",
            "false",
            ["true", "false"],
            "true runs impedance mode (0x6060 = -6), false runs CSP.",
        ),
        (
            "command_controller",
            command_controller,
            None,
            "Controller that takes the joint commands.",
        ),
        (
            "disable_transmissions",
            "",
            ["", "true", "false"],
            "true runs the robot without its transmissions.",
        ),
        ("launch_rviz", "true", ["true", "false"], "Show the robot in RViz."),
    ]
    return [
        DeclareLaunchArgument(
            name,
            default_value=default,
            description=text,
            **({"choices": allowed} if allowed else {}),
        )
        for name, default, allowed, text in arguments
    ]


def _robot_to_launch(context, *args, **kwargs):
    """Prepare the robot definition to launch and the joint lists that depend on it."""
    robot_config, param_file, disabled = prepare_launch_files(
        Arg("robot_config").perform(context),
        Arg("disable_transmissions").perform(context),
    )
    actions = [
        SetLaunchConfiguration("robot_config_in_use", robot_config),
        SetLaunchConfiguration("controller_joints_param_file", param_file),
    ]
    if disabled:
        actions.append(
            LogInfo(
                msg=f"Running without transmissions, from the robot definition {robot_config}"
            )
        )
    return actions


def _robot_description():
    """The URDF of the example robot, made by xacro."""
    xacro_arguments = {
        "robot_config": Arg("robot_config_in_use"),
        "impedance": Arg("impedance"),
        "default_slave_config": Arg("slave_config"),
        "hal": Arg("hal"),
        "network_interface": Arg("network_interface"),
    }
    command = [
        FindExecutable(name="xacro"),
        " ",
        _installed("synapticon_description", "urdf", "example_synapticon.urdf.xacro"),
    ]
    for key, value in xacro_arguments.items():
        command += [f" {key}:=", value]
    return ParameterValue(Command(command), value_type=str)


def _spawner():
    """
    One spawner for all controllers. It loads, configures and activates them one after the other, in this order, and
    stops at the first one that fails.
    """
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "drive_controller",
            "ethercat_bus_monitor_controller",
            Arg("command_controller"),
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            CONTROLLER_MANAGER_WAIT_S,
            # Holds only the joint lists of the drive controller and the joint state broadcaster. The spawner passes a
            # file only to the controllers it names.
            "--param-file",
            Arg("controller_joints_param_file"),
        ],
    )


def generate_launch_description():
    """The example robot: state publisher, controller manager, controllers and RViz."""
    nodes = [
        # The controller manager reads the robot description from the robot state publisher's topic.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": _robot_description()}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[Arg("controllers_file")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=[
                "-d",
                _installed("synapticon_bringup", "rviz", "example_synapticon.rviz"),
            ],
            condition=IfCondition(Arg("launch_rviz")),
        ),
    ]
    # Force coloured log output: launch pipes it, and the colours would be dropped.
    colours = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")
    # The robot definition is prepared before anything reads it.
    return LaunchDescription(
        [
            colours,
            *_declarations(),
            OpaqueFunction(function=_robot_to_launch),
            *nodes,
            _spawner(),
        ]
    )
