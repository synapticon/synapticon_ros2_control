# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Launch test of impedance mode through a transmission, on a simulated drive (mock_cia402).

joint_1 sits behind a SimpleTransmission with a reduction of -4 and an offset of 0.1 rad on the drive of motor_1
(config/geared_offset.yaml). The test commands joint_1 with a MotorCommand, enables the drive and checks both ways
of the transmission: the motor goes to (target - offset) * reduction, and joint_1 reads back the target.
The robot definition is passed as a relative path, which the example launch has to resolve before xacro reads it.
"""

import os

import launch_testing.markers
import pytest
from synapticon_bringup.testing import example_test_description, StackTestCase
from synapticon_control_msgs.msg import DriveStatus, MotorCommand
from synapticon_control_msgs.srv import SetDriveAction

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
REDUCTION = -4.0
OFFSET_RAD = 0.1
TARGET_RAD = 0.5
JOINT_TOLERANCE_RAD = 0.01  # the spring settles against the simulated friction
STACK_UP_S = 90.0  # the whole stack comes up well within this time
MOVE_S = 10.0


def motor_position(joint_position):
    """Where the motor is for `joint_position` of joint_1."""
    return (joint_position - OFFSET_RAD) * REDUCTION


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """The test robot on simulated drives, in impedance mode."""
    return example_test_description(
        {
            "robot_config": os.path.relpath(os.path.join(CONFIG, "geared_offset.yaml")),
            "controllers_file": os.path.join(CONFIG, "geared_offset_controllers.yaml"),
            "hal": "mock_cia402",
            "impedance": "true",
            "launch_rviz": "false",
        }
    )


class TestImpedanceThroughATransmission(StackTestCase):
    PROBE_NAME = "transmission_impedance_probe"

    def setUp(self):
        super().setUp()
        node = self.probe.node
        self.publisher = node.create_publisher(
            MotorCommand, "/motor_command_controller/commands", 10
        )
        self.action_client = node.create_client(
            SetDriveAction, "/drive_controller/set_drive_action"
        )

    def tearDown(self):
        """Leave the drive de-energised, and fail loudly when the drive controller refuses."""
        try:
            response = self._drive_action(SetDriveAction.Request.DISABLE_VOLTAGE)
            if not response.success:
                raise AssertionError(
                    f"disable_voltage refused, motor_1 may still have power: {response.message}"
                )
        finally:
            super().tearDown()

    def _drive_action(self, action):
        request = SetDriveAction.Request(action=action, dof_names=["motor_1"])
        return self.probe.call(self.action_client, request)

    def _command(self):
        msg = MotorCommand(
            name=["joint_1"],
            position=[TARGET_RAD],
            velocity=[0.0],
            torque_feedforward=[0.0],
        )
        self.publisher.publish(msg)

    def test_both_ways_of_the_transmission(self):
        positions = self.probe.positions
        drives = self.probe.drives
        self.probe.wait_for(
            lambda: getattr(drives.get("motor_1"), "device_state", None)
            == DriveStatus.STATE_SWITCH_ON_DISABLED
            and {"joint_1", "motor_1"} <= set(positions),
            STACK_UP_S,
            "the stack up, motor_1 in Switch On Disabled",
        )
        # At rest the motor is at 0, so the offset alone sets joint_1.
        self.assertAlmostEqual(
            positions["joint_1"],
            OFFSET_RAD + positions["motor_1"] / REDUCTION,
            delta=1e-6,
        )

        # The target is set first, so that the drive takes it when it gets power.
        self.probe.spin(0.5, on_cycle=self._command)
        self.assertTrue(self._drive_action(SetDriveAction.Request.ENABLE).success)
        self.probe.wait_for(
            lambda: abs(positions.get("joint_1", float("nan")) - TARGET_RAD)
            <= JOINT_TOLERANCE_RAD,
            MOVE_S,
            f"joint_1 at {TARGET_RAD} rad, last positions {positions}",
            on_cycle=self._command,
        )
        self.assertAlmostEqual(
            positions["motor_1"],
            motor_position(TARGET_RAD),
            delta=abs(REDUCTION) * JOINT_TOLERANCE_RAD,
        )
        self.assertEqual(
            drives["motor_1"].device_state, DriveStatus.STATE_OPERATION_ENABLED
        )
