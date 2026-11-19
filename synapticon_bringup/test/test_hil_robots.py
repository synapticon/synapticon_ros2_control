# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""Tests of the joint lists the hardware tests take from a robot definition."""

from synapticon_bringup.testing import commanded_joints, drive_joints, example_joints


def test_a_transmission_output_has_no_drive():
    """The geared example has two joints, but one drive: the runner treats it as a robot with one drive."""
    assert example_joints("geared") == ["joint_1", "motor_1"]
    assert drive_joints("geared") == ["motor_1"]
    assert commanded_joints("geared") == ["joint_1"]


def test_without_transmissions_every_joint_has_a_drive():
    assert (
        drive_joints("2dof")
        == commanded_joints("2dof")
        == example_joints("2dof")
        == ["joint_1", "joint_2"]
    )
