# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""Tests of hil_test_gains: the impedance gains of the hardware tests, per joint, from a robot definition."""

import pytest
import yaml

from synapticon_bringup.testing import hil_test_gains

JOINTS = [{"name": "geared", "position": 0}, {"name": "direct", "position": 1}]


def robot(tmp_path, hil_tests=None):
    """A robot definition with two joints and `hil_tests`, as a file. Returns its path."""
    path = tmp_path / "bench.yaml"
    content = {"joints": JOINTS} | (
        {"hil_tests": hil_tests} if hil_tests is not None else {}
    )
    path.write_text(yaml.safe_dump(content))
    return str(path)


def test_defaults_without_section(tmp_path):
    assert hil_test_gains(robot(tmp_path), "geared") == (15.0, 0.5)


def test_joint_entry_overrides_the_section(tmp_path):
    path = robot(
        tmp_path,
        {"stiffness": 10.0, "joints": {"direct": {"stiffness": 2.5, "damping": 0.03}}},
    )
    assert hil_test_gains(path, "geared") == (10.0, 0.5)
    assert hil_test_gains(path, "direct") == (2.5, 0.03)


def test_refuses_an_unknown_joint(tmp_path):
    path = robot(tmp_path, {"joints": {"drect": {"stiffness": 2.5}}})
    with pytest.raises(ValueError, match="drect"):
        hil_test_gains(path, "geared")
