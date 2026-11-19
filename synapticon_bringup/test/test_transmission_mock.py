# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Launch test of the geared example robot on the mock HAL.

joint_1 sits behind a SimpleTransmission with a reduction of 4 on the drive of motor_1. The test commands joint_1 and
expects motor_1 to turn four times as far: the mock HAL reports each motor command back as the motor's state, so the
motor position shows what the transmission commanded.
"""

import launch_testing.markers
import pytest
from std_msgs.msg import Float64MultiArray
from synapticon_bringup.testing import example_test_description, StackTestCase

REDUCTION = 4.0
TARGET_RAD = 0.5
TOLERANCE_RAD = 1e-3  # the mock reports the command back exactly
STACK_UP_S = 90.0  # the whole stack comes up well within this time


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """The geared example on the mock HAL, in CSP."""
    return example_test_description(
        {"robot": "geared", "hal": "mock", "launch_rviz": "false"}
    )


class TestGearedExample(StackTestCase):
    PROBE_NAME = "geared_probe"

    def setUp(self):
        super().setUp()
        self.publisher = self.probe.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )

    def test_the_motor_turns_four_times_as_far_as_the_joint(self, proc_output):
        positions = self.probe.positions
        self.probe.wait_for(
            lambda: abs(positions.get("joint_1", float("nan")) - TARGET_RAD)
            <= TOLERANCE_RAD,
            STACK_UP_S,
            f"joint_1 at {TARGET_RAD} rad, last positions {positions}",
            on_cycle=lambda: self.publisher.publish(
                Float64MultiArray(data=[TARGET_RAD])
            ),
        )
        self.assertAlmostEqual(
            positions["motor_1"],
            REDUCTION * TARGET_RAD,
            delta=REDUCTION * TOLERANCE_RAD,
        )
