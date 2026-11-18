# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Helpers for the launch tests of the example bring-up.

The launch description of the example stack, temporary YAML files, the settings the hardware
tests take from the environment, a probe node that follows the running stack, and the base class
of the test cases that use it.
"""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
import launch.actions
import launch.launch_description_sources
import launch_testing.actions
import rclpy
from sensor_msgs.msg import JointState
from synapticon_bringup.transmissions import joints_with_drive
from synapticon_control_msgs.msg import DriveStatusArray
import yaml

BUS_MONITOR = "ethercat_bus_monitor_controller"
DRIVE_CONTROLLER = "drive_controller"


def env_flag(name):
    """True when environment variable `name` is set to anything but empty, 0 or false."""
    return os.environ.get(name, "") not in ("", "0", "false")


def hil_hal():
    """The HAL the hardware tests run on: SYNAPTICON_HIL_HAL, ethercat_soem by default."""
    return os.environ.get("SYNAPTICON_HIL_HAL", "") or "ethercat_soem"


def hil_robot(default):
    """The robot the hardware tests run: SYNAPTICON_HIL_ROBOT, or `default` when it is unset."""
    return os.environ.get("SYNAPTICON_HIL_ROBOT", "") or default


def robot_files(robot):
    """
    The robot definition and the controller configuration of `robot`.

    `robot` is the name of an example robot (for example 1dof) or the path of a robot definition YAML.
    For a path, the controllers are in <name>_controllers.yaml next to it, as generate_example_robot.py writes them.
    """
    if robot.endswith(".yaml") or os.sep in robot:
        path = os.path.abspath(robot)
        return path, path[: -len(".yaml")] + "_controllers.yaml"
    config = os.path.join(get_package_share_directory("synapticon_bringup"), "config")
    return (
        os.path.join(config, f"example_{robot}.yaml"),
        os.path.join(config, f"example_{robot}_controllers.yaml"),
    )


def hil_launch_arguments(robot, impedance):
    """The example launch arguments of a hardware test, from its settings and the environment."""
    robot_config, controllers_file = robot_files(robot)
    arguments = {
        "robot_config": robot_config,
        "controllers_file": controllers_file,
        "hal": hil_hal(),
        "impedance": "true" if impedance else "false",
        "launch_rviz": "false",
    }
    slave_config = os.environ.get("SYNAPTICON_HIL_SLAVE_CONFIG", "")
    if slave_config:
        arguments["slave_config"] = slave_config
    network_interface = os.environ.get("SYNAPTICON_HIL_NETWORK_INTERFACE", "")
    if network_interface:
        arguments["network_interface"] = network_interface
    return arguments


def example_test_description(arguments):
    """The example launch with `arguments`, followed by ReadyToTest."""
    example = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("synapticon_bringup"),
                "launch",
                "example_synapticon.launch.py",
            )
        ),
        launch_arguments=arguments.items(),
    )
    return launch.LaunchDescription([example, launch_testing.actions.ReadyToTest()])


def write_yaml(prefix, content):
    """Write `content` as YAML into a new temporary file, and return the file name."""
    fd, path = tempfile.mkstemp(prefix=prefix, suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        yaml.safe_dump(content, stream, sort_keys=False)
    return path


def _read_yaml(path):
    """The content of the YAML file `path`."""
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def robot_bus_setting(robot, key, default):
    """The entry `key` of the bus section of the robot definition of `robot` (see robot_files), or `default`."""
    bus = _read_yaml(robot_files(robot)[0]).get("bus") or {}
    return bus.get(key, default)


def example_joints(robot):
    """Joint names of the robot definition of `robot` (see robot_files), in its order."""
    return [joint["name"] for joint in _read_yaml(robot_files(robot)[0])["joints"]]


def drive_joints(robot):
    """The joints of `robot` (see robot_files) that have a drive, in their order. A transmission output has none."""
    return joints_with_drive(_read_yaml(robot_files(robot)[0]))


def commanded_joints(robot):
    """The joints of `robot` (see robot_files) that take commands, in order. A motor behind a transmission does not."""
    joints = _read_yaml(robot_files(robot)[0])["joints"]
    return [joint["name"] for joint in joints if joint.get("commanded", True)]


def hil_test_gains(robot, joint):
    """
    Stiffness [Nm/rad] and damping [Nm s/rad] the impedance tests move `joint` of `robot` with.

    They come from the `hil_tests` section of its robot definition: the entry of `joint` under `joints`, else the
    section's own values. The defaults suit a geared joint. A joint without a gear has far less inertia and needs much
    smaller values. Raises ValueError when `joints` names a joint the robot does not have.
    """
    definition = _read_yaml(robot_files(robot)[0])
    section = definition.get("hil_tests") or {}
    per_joint = section.get("joints") or {}
    unknown = set(per_joint) - {entry["name"] for entry in definition["joints"]}
    if unknown:
        raise ValueError(
            f"hil_tests of {robot} names joints the robot does not have: {sorted(unknown)}"
        )
    own = per_joint.get(joint) or {}
    return (
        float(own.get("stiffness", section.get("stiffness", 15.0))),
        float(own.get("damping", section.get("damping", 0.5))),
    )


def example_controllers(robot):
    """The controller configuration of `robot` (see robot_files), as data."""
    return _read_yaml(robot_files(robot)[1])


class StackProbe:
    """
    A node that follows the running stack.

    It keeps the latest position of every joint on /joint_states, the latest drive status of every
    drive on the drive controller's topic, and on /diagnostics the latest report of the bus monitor
    and of every drive (by joint name). It offers waits and service calls that fail with a readable
    message.
    """

    def __init__(self, name):
        """Create the node and its subscriptions."""
        self.node = rclpy.create_node(name)
        self.positions = {}
        self.drives = {}
        self.bus = None
        self.drive_reports = {}
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.node.create_subscription(
            DriveStatusArray,
            "/drive_controller/drive_status",
            self._on_drive_status,
            10,
        )
        self.node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )
        self._list_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

    def destroy(self):
        """Destroy the node."""
        self.node.destroy_node()

    def _on_joint_state(self, msg):
        self.positions.update((n, float(p)) for n, p in zip(msg.name, msg.position))

    def _on_drive_status(self, msg):
        self.drives.update((drive.name, drive) for drive in msg.drives)

    def _on_diagnostics(self, msg):
        # The controller name prefixes each report. A stale one is a placeholder from before the first reading.
        for status in msg.status:
            if status.level == DiagnosticStatus.STALE:
                continue
            if status.name.startswith(BUS_MONITOR + ": "):
                self.bus = status
            elif status.name.startswith(DRIVE_CONTROLLER + ": "):
                self.drive_reports[status.hardware_id] = status

    def spin(self, seconds, on_cycle=None):
        """Spin for `seconds`, calling `on_cycle` on every pass."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if on_cycle:
                on_cycle()

    def wait_for(self, predicate, timeout_s, what, on_cycle=None):
        """Spin until `predicate` holds, calling `on_cycle` on every pass. Raise AssertionError on timeout."""
        deadline = time.time() + timeout_s
        while not predicate():
            if time.time() >= deadline:
                raise AssertionError(f"timeout waiting for {what}")
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if on_cycle:
                on_cycle()

    def call(self, client, request, timeout_s=5.0):
        """Call a service and return its response. Raise AssertionError when it is missing or silent."""
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise AssertionError(f"{client.srv_name} not available")
        future = client.call_async(request)
        self.wait_for(future.done, timeout_s, f"an answer of {client.srv_name}")
        return future.result()

    def active_controllers(self):
        """The names of the active controllers."""
        response = self.call(self._list_client, ListControllers.Request())
        return {c.name for c in response.controller if c.state == "active"}

    def wait_for_controllers(self, names, timeout_s=30.0, on_cycle=None):
        """Wait until every controller in `names` is active, asking twice a second."""
        deadline = time.time() + timeout_s
        while True:
            missing = set(names) - self.active_controllers()
            if not missing:
                return
            if time.time() >= deadline:
                raise AssertionError(
                    f"inactive controllers: {', '.join(sorted(missing))}"
                )
            self.spin(0.5, on_cycle)


class StackTestCase(unittest.TestCase):
    """
    Base of the test cases against the launched stack.

    The ROS context lasts as long as the class. Each test gets a new StackProbe named PROBE_NAME in
    `self.probe`. A subclass that extends setUp() or tearDown() calls the base version first in
    setUp() and last in tearDown().
    """

    PROBE_NAME = "stack_probe"

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.probe = StackProbe(self.PROBE_NAME)

    def tearDown(self):
        self.probe.destroy()
