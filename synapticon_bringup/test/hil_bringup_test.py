# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Hardware-in-the-loop bring-up test of a whole example robot, without energising any drive.

On real drives it is not registered with colcon test: it needs every drive of the robot
definition on the bus. It never enables a drive, so nothing moves:

    launch_test $(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_bringup_test.py

SYNAPTICON_HIL_ROBOT selects the example robot (default 1dof, for example 28dof), and
SYNAPTICON_HIL_HAL the HAL (default ethercat_soem, with its network interface in
SYNAPTICON_HIL_NETWORK_INTERFACE). colcon test runs it on simulated drives (mock_cia402). CSP by
default, SYNAPTICON_HIL_IMPEDANCE=1 brings the robot up in impedance mode, where the EtherCAT HAL
also checks the firmware version and the torque sensor of every drive.
SYNAPTICON_HIL_SLAVE_CONFIG selects the slave_config (default: the installed example config).

Checks: every drive of the robot definition streams, the bring-up prints no framed warning block,
every joint is published on /joint_states, every drive is online, de-energised, fault free and in
the requested mode, the controllers are active, /diagnostics reports every drive as de-energised
and the bus monitor reports a complete domain.
"""

import math
import os
import re

from diagnostic_msgs.msg import DiagnosticStatus
import launch_testing.markers
import pytest
from synapticon_bringup.testing import (
    env_flag,
    example_joints,
    example_test_description,
    hil_launch_arguments,
    StackProbe,
)
from synapticon_control_msgs.msg import DriveStatus

ROBOT = os.environ.get("SYNAPTICON_HIL_ROBOT", "") or "1dof"
IMPEDANCE = env_flag("SYNAPTICON_HIL_IMPEDANCE")
EXPECTED_MODE = -6 if IMPEDANCE else 8
CONTROLLERS = (
    "drive_controller",
    "ethercat_bus_monitor_controller",
    "joint_state_broadcaster",
    "motor_command_controller" if IMPEDANCE else "forward_position_controller",
)
STACK_TIMEOUT_S = 90.0  # must outlast the hardware's readiness_timeout_s
STATE_TIMEOUT_S = 10.0
# Bring-up output that fails the test: the framed warning blocks and a torque limit of zero.
BAD_OUTPUT = ("!! ", "0x2054 is 0")
BRING_UP_FAILED = "connect() failed"
JOINTS = example_joints(ROBOT)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the example robot on the HAL under test."""
    return example_test_description(hil_launch_arguments(ROBOT, IMPEDANCE))


def output_lines(proc_output):
    """All lines the launched processes printed so far."""
    text = "".join(event.text.decode(errors="replace") for event in proc_output)
    return text.splitlines()


class TestBringUp(StackTestCase):
    """Bring the robot up and check every drive without energising it."""

    PROBE_NAME = "hil_bringup_probe"

    def _wait_for_stack(self, proc_output):
        """Wait for the bring-up to succeed or fail, failing with its warnings and errors."""
        expected = f"{len(JOINTS)} drive(s) exchange valid data"
        outcome = re.compile(re.escape(expected) + "|" + re.escape(BRING_UP_FAILED))
        proc_output.waitFor(outcome, timeout=STACK_TIMEOUT_S)
        lines = output_lines(proc_output)
        if any(expected in line for line in lines):
            return
        problems = [
            line
            for line in lines
            if "[ERROR]" in line or any(bad in line for bad in BAD_OUTPUT)
        ]
        self.fail(f"no '{expected}' in the bring-up output:\n" + "\n".join(problems))

    def _check_drives(self):
        """Every drive is online, de-energised, fault free and in the requested mode."""
        for name in JOINTS:
            drive = self.probe.drives[name]
            with self.subTest(joint=name):
                self.assertTrue(drive.online, "not online")
                self.assertEqual(
                    drive.device_state,
                    DriveStatus.STATE_SWITCH_ON_DISABLED,
                    f"in {drive.device_state_name}",
                )
                self.assertEqual(drive.error_code, 0, f"error 0x{drive.error_code:04X}")
                self.assertFalse(drive.latched, "latched by the fault reaction")
                self.assertEqual(drive.mode_of_operation, EXPECTED_MODE, "mode")

    def _check_drive_reports(self):
        """/diagnostics reports every drive under its joint name, online and de-energised."""
        for name in JOINTS:
            report = self.probe.drive_reports[name]
            values = {entry.key: entry.value for entry in report.values}
            with self.subTest(joint=name):
                self.assertEqual(report.level, DiagnosticStatus.WARN, report.message)
                self.assertEqual(report.message, "Switch On Disabled")
                self.assertEqual(values.get("online"), "yes")
                self.assertEqual(values.get("al_state"), "OP")

    def test_bring_up(self, proc_output):
        """The robot comes up with every drive streaming, de-energised and healthy."""
        self._wait_for_stack(proc_output)
        self.probe.wait_for_controllers(CONTROLLERS)

        self.probe.wait_for(
            lambda: set(JOINTS) <= set(self.probe.positions),
            STATE_TIMEOUT_S,
            "every joint on /joint_states",
        )
        not_finite = [n for n in JOINTS if not math.isfinite(self.probe.positions[n])]
        self.assertEqual(not_finite, [], "joints without a finite position")

        self.probe.wait_for(
            lambda: set(JOINTS) <= set(self.probe.drives),
            STATE_TIMEOUT_S,
            "every drive on drive_status",
        )
        self._check_drives()

        self.probe.wait_for(
            lambda: set(JOINTS) <= set(self.probe.drive_reports),
            STATE_TIMEOUT_S,
            "every drive on /diagnostics",
        )
        self._check_drive_reports()

        self.probe.wait_for(
            lambda: self.probe.bus is not None, STATE_TIMEOUT_S, "bus monitor"
        )
        self.assertEqual(
            self.probe.bus.level, DiagnosticStatus.OK, self.probe.bus.message
        )
        self.assertEqual(self.probe.bus.hardware_id, "ethercat_bus")

        bad = [
            line
            for line in output_lines(proc_output)
            if any(text in line for text in BAD_OUTPUT)
        ]
        self.assertEqual(bad, [], "the bring-up printed warnings")
