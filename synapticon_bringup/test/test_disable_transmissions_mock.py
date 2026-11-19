# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Launch test of disable_transmissions:=true with the geared example robot on the mock HAL.

Without its gearbox, joint_1 sits directly on the drive of motor_1. The test commands joint_1 and expects it to
arrive one to one, motor_1 to be gone from /joint_states, and the drive controller to report joint_1 as its
drive.
"""

import launch_testing.markers
import pytest
from std_msgs.msg import Float64MultiArray
from synapticon_bringup.testing import example_test_description, StackTestCase

TARGET_RAD = -0.3
TOLERANCE_RAD = 1e-3  # the mock reports the command back exactly
STACK_UP_S = 90.0  # the whole stack comes up well within this time


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """The geared example on the mock HAL, with its transmission left out."""
    return example_test_description(
        {
            "robot": "geared",
            "hal": "mock",
            "launch_rviz": "false",
            "disable_transmissions": "true",
        }
    )


class TestWithoutTheGearbox(StackTestCase):
    PROBE_NAME = "without_gearbox_probe"

    def setUp(self):
        super().setUp()
        self.publisher = self.probe.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )

    def test_the_joint_moves_one_to_one_on_the_drive(self, proc_output):
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
        self.assertNotIn("motor_1", positions)
        self.probe.wait_for(
            lambda: "joint_1" in self.probe.drives, 10.0, "joint_1 on ~/drive_status"
        )
        self.assertEqual(set(self.probe.drives), {"joint_1"})
