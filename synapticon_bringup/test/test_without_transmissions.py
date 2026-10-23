# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Tests of running a robot without its transmissions.

The test robot is a gripper on a lift. Two motors move the two fingers through one differential, and the lift has a
drive of its own. A second robot adds passive idlers, which take part in a transmission without a drive.
"""

import copy
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import pytest

import yaml

from synapticon_bringup.transmissions import prepare_launch_files, without_transmissions


def gripper():
    """Two fingers on one differential, driven by left_motor and right_motor, and a lift with its own drive."""
    return {
        "bus": {"network_interface": "eth3", "control_frequency": 250},
        "joints": [
            {"name": "lift", "position": 4, "slave_config": "/drives/lift.yaml"},
            {
                "name": "left_motor",
                "position": 5,
                "slave_config": "/drives/left.yaml",
                "max_joint_torque": 2.0,
                "commanded": False,
            },
            {
                "name": "right_motor",
                "position": 6,
                "slave_config": "/drives/right.yaml",
                "invert": True,
                "commanded": False,
            },
            {"name": "finger_open", "drive": False},
            {"name": "finger_tilt", "drive": False},
        ],
        "transmissions": [
            {
                "name": "finger_diff",
                "plugin": "transmission_interface/DifferentialTransmission",
                "actuators": [
                    {"name": "left_motor", "role": "actuator1"},
                    {"name": "right_motor", "role": "actuator2"},
                ],
                "joints": [
                    {"name": "finger_open", "role": "joint1"},
                    {"name": "finger_tilt", "role": "joint2"},
                ],
            },
        ],
    }


def gripper_with_idlers():
    """The gripper, with a passive idler next to each motor in the transmission."""
    robot = gripper()
    robot["joints"] += [
        {"name": "idler_a", "drive": False, "commanded": False},
        {"name": "idler_b", "drive": False, "commanded": False},
    ]
    robot["transmissions"][0]["joints"] += [
        {"name": "idler_a", "role": "idler1"},
        {"name": "left_motor", "role": "actuator1"},
        {"name": "idler_b", "role": "idler2"},
    ]
    return robot


def plain_gripper():
    """The gripper as one would write it without the differential: each finger sits on the drive of one motor."""
    return {
        "bus": {"network_interface": "eth3", "control_frequency": 250},
        "joints": [
            {"name": "lift", "position": 4, "slave_config": "/drives/lift.yaml"},
            {
                "name": "finger_open",
                "position": 5,
                "slave_config": "/drives/left.yaml",
                "max_joint_torque": 2.0,
            },
            {
                "name": "finger_tilt",
                "position": 6,
                "slave_config": "/drives/right.yaml",
                "invert": True,
            },
        ],
        "transmissions": [],
    }


def joint(robot, name):
    """The entry of joint `name`."""
    return next(entry for entry in robot["joints"] if entry["name"] == name)


def joint_names(robot):
    return [entry["name"] for entry in robot["joints"]]


# The plain variant


@pytest.mark.parametrize(
    "make_robot", [gripper, gripper_with_idlers], ids=["differential", "with idlers"]
)
def test_the_plain_variant_is_the_robot_written_without_transmissions(make_robot):
    """
    Motors and idlers leave the joint list, the fingers take over the motors' drives, and everything else stays.

    The comparison covers the order of the joints and that no 'commanded' or 'drive' key comes along.
    """
    robot = make_robot()
    before = copy.deepcopy(robot)
    assert without_transmissions(robot) == plain_gripper()
    assert robot == before


def own_torque_limit(robot):
    joint(robot, "finger_open")["max_joint_torque"] = 0.5


def lift_not_energized(robot):
    joint(robot, "lift")["energize"] = False


VARIANTS = {
    "a finger's own setting wins over its motor's": (
        own_torque_limit,
        own_torque_limit,
    ),
    "a motor's explicit drive flag stays behind": (
        lambda robot: joint(robot, "right_motor").update(drive=True),
        lambda plain: None,
    ),
    "a joint outside the transmissions comes as it is": (
        lift_not_energized,
        lift_not_energized,
    ),
}


@pytest.mark.parametrize("change, expect", VARIANTS.values(), ids=VARIANTS.keys())
def test_variants_of_the_gripper(change, expect):
    """The same change to the gripper changes the plain variant as `expect` says."""
    robot, plain = gripper(), plain_gripper()
    change(robot)
    expect(plain)
    assert without_transmissions(robot) == plain


def test_a_robot_without_transmissions_comes_back_as_it_is():
    robot = gripper()
    robot["transmissions"] = []
    assert without_transmissions(robot) == robot


# Robots that cannot run without their transmissions


def add_second_transmission(robot, outputs, actuators, first=False):
    """Add a transmission named 'second' with new output joints and the given actuators."""
    robot["joints"] += [{"name": name, "drive": False} for name in outputs]
    second = {
        "name": "second",
        "plugin": "transmission_interface/DifferentialTransmission",
        "actuators": [
            {"name": name, "role": f"actuator{i + 1}"}
            for i, name in enumerate(actuators)
        ],
        "joints": [
            {"name": name, "role": f"joint{i + 1}"} for i, name in enumerate(outputs)
        ],
    }
    robot["transmissions"].insert(0 if first else 1, second)


def chain(first):
    """A second transmission driven by the fingers, declared before or after the differential."""
    return lambda robot: add_second_transmission(
        robot, ["tip_a", "tip_b"], ["finger_open", "finger_tilt"], first
    )


# Per robot: how it is broken, and words the error must contain, mostly the joints at fault.
REFUSED = {
    "a finger without a motor": (
        gripper,
        lambda robot: robot["transmissions"][0]["joints"].pop(),
        ["finger_diff", "exactly one actuator"],
    ),
    "a motor that is not in the joint list": (
        gripper,
        lambda robot: robot["transmissions"][0]["actuators"][0].update(
            name="spare_motor"
        ),
        ["spare_motor"],
    ),
    "a motor in two transmissions": (
        gripper,
        lambda robot: add_second_transmission(
            robot, ["thumb_a", "thumb_b"], ["left_motor", "right_motor"]
        ),
        ["left_motor", "'finger_diff' and 'second'"],
    ),
    "a chain, declared last": (gripper, chain(first=False), ["finger_open"]),
    "a chain, declared first": (gripper, chain(first=True), ["finger_open"]),
    "a finger with a drive of its own": (
        gripper,
        lambda robot: joint(robot, "finger_open").pop("drive"),
        ["finger_open", "already has a drive"],
    ),
    "a motor that the transmission lists as a joint only": (
        gripper_with_idlers,
        lambda robot: robot["transmissions"][0]["actuators"].pop(0),
        ["left_motor", "must be one of the actuators"],
    ),
    "an entry without a name": (
        gripper,
        lambda robot: robot["joints"].append({"position": 7}),
        ["'joints'", "no name"],
    ),
    "a joint listed twice": (
        gripper,
        lambda robot: robot["joints"].append({"name": "lift"}),
        ["lift", "more than once"],
    ),
}


@pytest.mark.parametrize(
    "make_robot, change, words", REFUSED.values(), ids=REFUSED.keys()
)
def test_robots_that_cannot_run_without_transmissions(make_robot, change, words):
    robot = make_robot()
    change(robot)
    with pytest.raises(ValueError) as error:
        without_transmissions(robot)
    for word in words:
        assert word in str(error.value)


def test_every_problem_is_reported_at_once():
    """A robot with two problems gets one error that lists both."""
    robot = gripper()
    robot["transmissions"][0]["actuators"][0]["name"] = "spare_motor"
    robot["joints"].append({"name": "lift"})
    with pytest.raises(ValueError) as error:
        without_transmissions(robot)
    message = str(error.value)
    assert "spare_motor" in message and "more than once" in message
    assert message.count("\n- ") == 2


# Through the example xacro and the launch helper


def as_file(robot):
    """Write `robot` to a temporary robot definition and return its path."""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as stream:
        yaml.safe_dump(robot, stream)
        return stream.name


def xacro_block(robot):
    """The <ros2_control> element of the example URDF for `robot`, and the robot definition file it names."""
    path = as_file(robot)
    xacro = (
        get_package_share_directory("synapticon_description")
        + "/urdf/example_synapticon.urdf.xacro"
    )
    command = [
        "xacro",
        xacro,
        f"robot_config:={path}",
        "impedance:=true",
        "default_slave_config:=/drives/default.yaml",
    ]
    urdf = subprocess.run(
        command + ["hal:=mock"], check=True, capture_output=True, text=True
    ).stdout
    return ET.fromstring(urdf).find("ros2_control"), path


def test_the_urdf_of_the_plain_variant_has_no_transmission():
    """With transmissions the URDF declares them and the motors. The plain variant declares neither."""
    block, _ = xacro_block(gripper())
    assert block.find("transmission") is not None
    assert "left_motor" in {entry.get("name") for entry in block.findall("joint")}

    block, path = xacro_block(without_transmissions(gripper()))
    assert block.find("transmission") is None
    joints = {entry.get("name"): entry for entry in block.findall("joint")}
    assert set(joints) == {"lift", "finger_open", "finger_tilt"}
    commands = {
        command.get("name")
        for command in joints["finger_open"].findall("command_interface")
    }
    assert {"position", "drive_action"} <= commands
    # The drive settings stay in the robot definition, which the hardware reads.
    assert joints["finger_open"].findall("param") == []
    hardware = {
        param.get("name"): param.text
        for param in block.find("hardware").findall("param")
    }
    assert hardware["robot_config"] == path
    assert (hardware["default_slave_config"], hardware["mode_of_operation"]) == (
        "/drives/default.yaml",
        "-6",
    )


def drive_controller_joints(param_file):
    with open(param_file, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["drive_controller"]["ros__parameters"]["joints"]


@pytest.mark.parametrize(
    "with_transmission, override, joints",
    [
        (False, "", ["lift", "left_motor", "right_motor"]),
        (True, "true", ["lift", "finger_open", "finger_tilt"]),
    ],
    ids=["kept as it is", "transmissions left out"],
)
def test_the_files_for_the_launch(with_transmission, override, joints):
    """The launch gets the robot definition to use, and the drive controller gets the joints that have a drive."""
    robot = gripper()
    if not with_transmission:
        robot["transmissions"] = []
    path = as_file(robot)
    robot_config, param_file, disabled = prepare_launch_files(path, override)
    assert disabled == with_transmission
    assert (robot_config == path) != with_transmission
    with open(robot_config, encoding="utf-8") as stream:
        launched = yaml.safe_load(stream)
    assert [
        name
        for name in joint_names(launched)
        if joint(launched, name).get("drive", True)
    ] == joints
    assert drive_controller_joints(param_file) == joints
