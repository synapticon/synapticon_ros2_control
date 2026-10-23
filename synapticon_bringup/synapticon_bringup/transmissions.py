# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Run a robot without its transmissions.

A transmission connects motors (its actuators) to the joints they move (its outputs). Without it, each output joint
sits directly on the drive of one motor, as if the gearbox were not there. That helps to tell whether a problem comes
from the transmission or from the rest of the setup.

The example launch uses this module to prepare its files.
"""

from dataclasses import dataclass, field
import copy
import os
import tempfile

import yaml

# Keys that say what an entry is. An output joint does not take them over from its motor.
ROLE_KEYS = ("name", "commanded", "drive")


@dataclass
class Transmission:
    """The joint names of one transmission. `joints` leaves out the joints that are also its actuators."""

    name: str
    actuators: list
    joints: list
    outputs: list = field(default_factory=list)


def _name_of(entry, where, problems):
    """The name of a list entry, or None after noting in `problems` that it has none."""
    name = entry.get("name") if isinstance(entry, dict) else None
    if not name:
        problems.append(f"An entry of {where} has no name: {entry!r}")
    return name


def _names_of(entries, where, problems):
    """The names of the list entries that have one."""
    names = [_name_of(entry, where, problems) for entry in entries or []]
    return [name for name in names if name]


def _drives_a_motor(joint):
    return joint.get("drive", True)


def _joints_by_name(joints, problems):
    """The 'joints' list as a dictionary by name. A missing or repeated name is a problem."""
    table = {}
    for joint in joints:
        name = _name_of(joint, "'joints'", problems)
        if name in table:
            problems.append(f"Joint '{name}' appears more than once in 'joints'.")
        elif name:
            table[name] = joint
    return table


def _read_transmission(entry, problems):
    """The names in one entry of 'transmissions'."""
    label = entry.get("name", "(no name)")
    actuators = _names_of(
        entry.get("actuators"), f"the actuators of transmission '{label}'", problems
    )
    joints = _names_of(
        entry.get("joints"), f"the joints of transmission '{label}'", problems
    )
    return Transmission(
        label, actuators, [name for name in joints if name not in actuators]
    )


def _listed(name, transmission, table, problems):
    """The entry of joint `name` in 'joints', or None after noting that it is not there."""
    joint = table.get(name)
    if joint is None:
        problems.append(
            f"Transmission '{transmission.name}' names joint '{name}', which 'joints' does not list."
        )
    return joint


def _sort_joints(transmissions, table, problems):
    """
    Check every joint the transmissions name against its entry, and sort out the outputs.

    An actuator must have a drive. Any other joint of a transmission is an output when it takes commands, and then
    must not have a drive of its own. A joint without drive and commands, for example an idler, is passive. A drive
    without commands belongs among the actuators. Fills in the outputs and returns the names of the passive joints.
    """
    passive = set()
    for transmission in transmissions:
        label = transmission.name
        for name in transmission.actuators:
            joint = _listed(name, transmission, table, problems)
            if joint is not None and not _drives_a_motor(joint):
                problems.append(
                    f"Transmission '{label}' lists '{name}' as an actuator, but the robot definition gives it "
                    "no drive."
                )
        for name in transmission.joints:
            joint = _listed(name, transmission, table, problems)
            if joint is None:
                continue
            if joint.get("commanded", True):
                transmission.outputs.append(name)
                if _drives_a_motor(joint):
                    problems.append(
                        f"Transmission '{label}': output '{name}' already has a drive of its own."
                    )
            elif _drives_a_motor(joint):
                problems.append(
                    f"Transmission '{label}': joint '{name}' has a drive but takes no commands, so it must be one "
                    "of the actuators."
                )
            else:
                passive.add(name)
    return passive


def _check_single_use(transmissions, problems):
    """Each actuator and each output may appear once, in one transmission."""
    uses = {}
    for transmission in transmissions:
        for name in transmission.actuators + transmission.outputs:
            uses.setdefault(name, []).append(f"'{transmission.name}'")
    for name, labels in uses.items():
        if len(labels) > 1:
            problems.append(
                f"Joint '{name}' is used more than once as actuator or output, by transmissions "
                + " and ".join(labels)
                + "."
            )


def _motor_of_each_output(transmissions, problems):
    """
    The motor whose drive each output takes over: the actuator at the same place in its transmission.

    Only transmissions with as many outputs as actuators can be undone this way.
    """
    motor_of = {}
    for transmission in transmissions:
        if len(transmission.outputs) != len(transmission.actuators):
            problems.append(
                f"Transmission '{transmission.name}' cannot become plain joints: each output joint needs exactly "
                f"one actuator, and it has {len(transmission.outputs)} output(s) for "
                f"{len(transmission.actuators)} actuator(s)."
            )
            continue
        motor_of.update(zip(transmission.outputs, transmission.actuators))
    return motor_of


def _on_the_motor_drive(output, actuator):
    """The output's entry moved onto the motor's drive. The output's own settings win over the motor's."""
    from_motor = {key: value for key, value in actuator.items() if key not in ROLE_KEYS}
    own = {key: value for key, value in output.items() if key != "drive"}
    return {"name": output["name"]} | from_motor | own


def without_transmissions(config):
    """
    A copy of the robot definition `config` that has no transmissions.

    - Each output joint takes over the drive of its motor, and its own settings win over the motor's.
    - Left out: the motors, the passive joints and the transmissions.
    - The joints that remain keep their order.

    When a transmission cannot be undone this way, for example with more outputs than motors, a ValueError lists
    all the problems found.
    """
    plain = copy.deepcopy(config)
    transmissions = plain.get("transmissions") or []
    plain["transmissions"] = []
    if not transmissions:
        return plain

    # Collect all problems before anything changes. Checks on the original entries give the same answer in any
    # order of the transmissions.
    problems = []
    table = _joints_by_name(plain.get("joints") or [], problems)
    transmissions = [_read_transmission(entry, problems) for entry in transmissions]
    passive = _sort_joints(transmissions, table, problems)
    _check_single_use(transmissions, problems)
    motor_of = _motor_of_each_output(transmissions, problems)
    if problems:
        raise ValueError(
            "The robot cannot run without its transmissions:\n"
            + "\n".join(f"- {problem}" for problem in problems)
        )

    for output, motor in motor_of.items():
        table[output] = _on_the_motor_drive(table[output], table[motor])
    left_out = passive | set(motor_of.values())
    plain["joints"] = [joint for name, joint in table.items() if name not in left_out]
    return plain


def joints_with_drive(config):
    """The names of the joints that have a drive, in their order."""
    return [
        joint["name"] for joint in config.get("joints") or [] if _drives_a_motor(joint)
    ]


def _read_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _write_temporary_yaml(prefix, content, sort_keys=True):
    """Write `content` to a new temporary YAML file and return its path."""
    fd, path = tempfile.mkstemp(prefix=prefix, suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        yaml.safe_dump(content, stream, sort_keys=sort_keys)
    return path


def robot_config_to_launch(path, override=""):
    """
    The robot definition to launch, as (path, disabled).

    `disabled` comes from `override` ('true' or 'false'), and when that is empty from the file's
    'disable_transmissions' (default false). `path` is the file itself, unless it has transmissions to disable: then
    it is a temporary copy without them.
    """
    config = _read_yaml(path)
    choice = (
        override
        if override
        else str(config.get("disable_transmissions", False)).lower()
    )
    disabled = choice == "true"
    if not (disabled and config.get("transmissions")):
        return path, disabled
    return (
        _write_temporary_yaml(
            "robot_1to1_", without_transmissions(config), sort_keys=False
        ),
        True,
    )


def prepare_launch_files(path, override=""):
    """
    Everything the example launch needs from a robot definition.

    Returns (robot_config_path, param_file_path, disabled). The parameter file lists the joints of the drive
    controller, which change when the transmissions are left out.
    """
    robot_config, disabled = robot_config_to_launch(path, override)
    joints = joints_with_drive(_read_yaml(robot_config))
    param_file = _write_temporary_yaml(
        "drive_controller_joints_",
        {"drive_controller": {"ros__parameters": {"joints": joints}}},
    )
    return robot_config, param_file, disabled
