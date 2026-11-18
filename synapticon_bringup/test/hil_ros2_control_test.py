# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Hardware-in-the-loop test of the ros2_control path on one drive.

On real drives it is not registered with colcon test: it needs one drive on the bus and moves it
about 0.3 rad. Run it on purpose, with the joint free to turn and nothing else holding the master:

    launch_test $(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_ros2_control_test.py

The 1 DOF example is brought up through the example launch on the HAL that SYNAPTICON_HIL_HAL
names (default ethercat_soem, with its network interface in SYNAPTICON_HIL_NETWORK_INTERFACE).
colcon test runs it on simulated drives (mock_cia402). CSP by default, SYNAPTICON_HIL_IMPEDANCE=1
runs the same steps in impedance mode through the MotorCommandController.
SYNAPTICON_HIL_SLAVE_CONFIG selects the slave_config (default: the installed example config).

Steps: wait for the stack, hold the measured position, enable the drive, ramp 0.3 rad away,
quick stop, enable again, ramp back, disable. A watchdog cuts the voltage and fails the test
if the joint strays more than 0.8 rad from its commanded target. The drive's report on
/diagnostics follows the enable and the quick stop.
"""

import time

from diagnostic_msgs.msg import DiagnosticStatus
import launch_testing.markers
import pytest
from std_msgs.msg import Float64MultiArray
from synapticon_bringup.testing import (
    commanded_joints,
    drive_joints,
    env_flag,
    example_test_description,
    hil_launch_arguments,
    hil_robot,
    hil_test_gains,
    StackTestCase,
)
from synapticon_control_msgs.msg import DriveStatus, MotorCommand
from synapticon_control_msgs.srv import SetDriveAction

ROBOT = hil_robot("1dof")
COMMANDED, DRIVES = commanded_joints(ROBOT), drive_joints(ROBOT)
if len(COMMANDED) != 1 or len(DRIVES) != 1:
    raise ValueError(
        f"the path test needs a robot with one commanded joint and one drive, {ROBOT} has "
        f"{len(COMMANDED)} and {len(DRIVES)}"
    )
# The joint the test commands and measures, and the joint whose drive it switches. They differ behind a transmission.
JOINT, DRIVE = COMMANDED[0], DRIVES[0]
IMPEDANCE = env_flag("SYNAPTICON_HIL_IMPEDANCE")
COMMAND_CONTROLLER = (
    "motor_command_controller" if IMPEDANCE else "forward_position_controller"
)
# Interval at which the current target is re-published while the test waits [s].
REPUBLISH_S = 0.05
MOVE_RAD = 0.3
# Tracking tolerance once a ramp has ended, the joint settles against friction [rad].
TRACKING_TOLERANCE_RAD = 0.05
TRACKING_TIMEOUT_S = 5.0
REPORT_TIMEOUT_S = 2.0  # a change reaches /diagnostics within a fraction of a second
# Travel allowed after a quick stop request [rad].
QUICK_STOP_TOLERANCE_RAD = 0.1
# The watchdog fires when the joint is this far from its commanded target [rad].
WATCHDOG_RAD = 0.8
STIFFNESS, DAMPING = hil_test_gains(
    ROBOT, JOINT
)  # Nm/rad and Nm s/rad of the impedance moves
STACK_TIMEOUT_S = 90.0  # must outlast the hardware's readiness_timeout_s


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the robot on the HAL under test."""
    return example_test_description(hil_launch_arguments(ROBOT, IMPEDANCE))


class TestRos2ControlPath(StackTestCase):
    """Enable, move, quick stop and disable the drive through the running stack."""

    PROBE_NAME = "hil_ros2_control_probe"

    def setUp(self):
        """Add the command publisher and the drive service clients to the probe."""
        super().setUp()
        node = self.probe.node
        self.target = None
        self.last_publish = 0.0
        if IMPEDANCE:
            self.command_pub = node.create_publisher(
                MotorCommand, "/motor_command_controller/commands", 10
            )
        else:
            self.command_pub = node.create_publisher(
                Float64MultiArray, "/forward_position_controller/commands", 10
            )
        self.action_client = node.create_client(
            SetDriveAction, "/drive_controller/set_drive_action"
        )

    def tearDown(self):
        """Leave the drive de-energised."""
        if self._drive_state() not in (None, DriveStatus.STATE_SWITCH_ON_DISABLED):
            self._drive_action(SetDriveAction.Request.DISABLE_VOLTAGE)
        super().tearDown()

    def _position(self):
        return self.probe.positions.get(JOINT)

    def _drive_state(self):
        drive = self.probe.drives.get(DRIVE)
        return drive.device_state if drive else None

    def _on_cycle(self):
        """Keep streaming the target, so a controller activated late still gets it, and run the watchdog."""
        if self.target is not None and time.time() - self.last_publish >= REPUBLISH_S:
            self._command(self.target)
        self._watchdog()

    def _spin(self, seconds):
        self.probe.spin(seconds, self._on_cycle)

    def _wait_for(self, predicate, timeout_s, what):
        """Spin until `predicate` holds, failing with `what` and the joint's state on timeout."""
        try:
            self.probe.wait_for(predicate, timeout_s, what, self._on_cycle)
        except AssertionError as error:
            self.fail(
                f"{error} (measured {self._position()}, state {self._drive_state()})"
            )

    def _watchdog(self):
        """Cut the voltage and fail if the joint runs away from its commanded target."""
        position = self._position()
        if self.target is None or position is None:
            return
        if abs(position - self.target) > WATCHDOG_RAD:
            target = self.target
            self.target = None
            self._drive_action(SetDriveAction.Request.DISABLE_VOLTAGE)
            self.fail(
                f"watchdog: {JOINT} at {position:.3f} rad, target {target:.3f} rad"
            )

    def _drive_action(self, action):
        """Request a drive action by name on the joint under test."""
        request = SetDriveAction.Request(action=action, dof_names=[DRIVE])
        return self.probe.call(self.action_client, request)

    def _command(self, position):
        """Publish one setpoint and make it the watchdog's reference."""
        if IMPEDANCE:
            msg = MotorCommand()
            msg.name = [JOINT]
            msg.position = [position]
            msg.velocity = [0.0]
            msg.torque_feedforward = [0.0]
            msg.stiffness = [STIFFNESS]
            msg.damping = [DAMPING]
        else:
            msg = Float64MultiArray(data=[position])
        self.command_pub.publish(msg)
        self.target = position
        self.last_publish = time.time()

    def _ramp_to(self, position, duration_s=1.0, rate_hz=50.0):
        """Move the setpoint smoothly from the current target to `position`."""
        start = self.target if self.target is not None else self._position()
        steps = int(duration_s * rate_hz)
        for i in range(1, steps + 1):
            self._command(start + (position - start) * i / steps)
            self._spin(1.0 / rate_hz)

    def _at_target(self):
        """True once the joint tracks its target within the tolerance."""
        position = self._position()
        return (
            position is not None
            and abs(position - self.target) <= TRACKING_TOLERANCE_RAD
        )

    def _state_is(self, state):
        return lambda: self._drive_state() == state

    def _reported_as(self, message):
        """True once /diagnostics reports the joint's drive with `message`."""
        return (
            lambda: getattr(self.probe.drive_reports.get(DRIVE), "message", None)
            == message
        )

    def _reported_at(self, level):
        """True once /diagnostics reports the joint's drive at `level`."""
        return (
            lambda: getattr(self.probe.drive_reports.get(DRIVE), "level", None) == level
        )

    def _expect_accepted(self, action):
        response = self._drive_action(action)
        self.assertTrue(response.success, response.message)

    def test_enable_move_quick_stop_disable(self, proc_output):
        """The full path from the launch down to the drive and back."""
        self._wait_for(
            lambda: self._position() is not None and self._drive_state() is not None,
            STACK_TIMEOUT_S,
            "the stack",
        )
        self.assertTrue(self.probe.drives[DRIVE].online, f"{DRIVE} is not online")
        self.assertEqual(
            self._drive_state(),
            DriveStatus.STATE_SWITCH_ON_DISABLED,
            "the drive is not de-energised",
        )

        # The command controller is spawned last. Hold the measured position before enabling so
        # the drive does not jump to a stale setpoint.
        self.probe.wait_for_controllers([COMMAND_CONTROLLER], on_cycle=self._on_cycle)
        start = self._position()
        self._command(start)
        self._spin(0.5)

        self._expect_accepted(SetDriveAction.Request.ENABLE)
        self._wait_for(
            self._state_is(DriveStatus.STATE_OPERATION_ENABLED),
            3.0,
            "Operation Enabled",
        )
        self._wait_for(
            self._reported_as("Operation Enabled"),
            REPORT_TIMEOUT_S,
            "Operation Enabled on /diagnostics",
        )
        self.assertEqual(
            self.probe.drive_reports[DRIVE].level, DiagnosticStatus.OK, "level"
        )

        self._ramp_to(start + MOVE_RAD)
        self._wait_for(
            self._at_target,
            TRACKING_TIMEOUT_S,
            f"tracking of the target {start + MOVE_RAD:.3f} rad",
        )

        self._expect_accepted(SetDriveAction.Request.QUICK_STOP)
        self._wait_for(
            self._state_is(DriveStatus.STATE_QUICK_STOP_ACTIVE),
            1.0,
            "Quick Stop Active",
        )
        stopped_at = self._position()
        # Depending on the drive, it stays in Quick Stop Active or goes on to Switch On Disabled.
        self._wait_for(
            self._reported_at(DiagnosticStatus.WARN),
            REPORT_TIMEOUT_S,
            "a warning on /diagnostics after the quick stop",
        )
        self._spin(1.0)
        self.assertLess(
            abs(self._position() - stopped_at),
            QUICK_STOP_TOLERANCE_RAD,
            "the joint moved after the quick stop",
        )

        self._expect_accepted(SetDriveAction.Request.ENABLE)
        self._wait_for(
            self._state_is(DriveStatus.STATE_OPERATION_ENABLED),
            3.0,
            "Operation Enabled again",
        )

        self._ramp_to(start)
        self._wait_for(
            self._at_target, TRACKING_TIMEOUT_S, f"tracking back to {start:.3f} rad"
        )

        self._expect_accepted(SetDriveAction.Request.DISABLE)
        self._wait_for(
            self._state_is(DriveStatus.STATE_SWITCH_ON_DISABLED),
            3.0,
            "Switch On Disabled",
        )
        self.target = None
