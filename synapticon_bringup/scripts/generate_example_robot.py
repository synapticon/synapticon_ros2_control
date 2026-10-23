#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Create an example robot with as many joints as you like.

It writes two files into the config folder of this package: the robot definition example_<name>.yaml and the
controller configuration example_<name>_controllers.yaml. Joint n sits on the drive at ring position n - 1.
Rebuild the package afterwards, then launch the robot with robot:=<name>.

Examples:
    python3 scripts/generate_example_robot.py --joints 5
    python3 scripts/generate_example_robot.py --joints 3 --robot gantry --names x_axis,y_axis,z_axis
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

# Where the files go: the config folder next to this script's folder.
CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


@dataclass
class Hint:
    """A setting written as a comment, for the reader to switch on."""

    value: object
    note: str = ""


def robot_data(names):
    """The robot definition as data, with a note per key."""
    bus = {
        "control_frequency": (
            500,
            "Hz, equal to the update_rate of the controller manager",
        ),
        "readiness_timeout_s": (
            20.0,
            "the time start() gives every drive to send valid data",
        ),
        "network_interface": (
            Hint("eth0"),
            "network interface of the drives, for ethercat_soem",
        ),
        "quick_stop_hold": (
            Hint(False),
            "tests only: no frozen target after a quick stop in impedance mode",
        ),
    }
    joints = [
        {"name": name, "position": position} for position, name in enumerate(names)
    ]
    return {"bus": bus, "joints": joints}


JOINT_KEYS_HELP = """\
Each joint needs a name. The other keys are optional:
  position          ring position of its drive on the bus (default 0)
  alias             EtherCAT alias, only 0 works: drives are found by ring position
  slave_config      slave_config file of the drive (default: the launch argument slave_config)
  max_joint_torque  torque limit in Nm (default: the limit stored on the drive)
  energize: false   read and report the drive, but never power it
  invert: true      flip position, velocity and effort, for a drive mounted the other way round"""

CONTROLLER_TYPES = {
    "joint_state_broadcaster": ("joint_state_broadcaster/JointStateBroadcaster", ""),
    "drive_controller": (
        "synapticon_controllers/DriveController",
        "drive states, drive actions, fault reaction",
    ),
    "ethercat_bus_monitor_controller": (
        "synapticon_controllers/EthercatBusMonitorController",
        "bus health on /diagnostics",
    ),
    "forward_position_controller": (
        "forward_command_controller/ForwardCommandController",
        "commands in CSP",
    ),
    "motor_command_controller": (
        "synapticon_controllers/MotorCommandController",
        "commands in impedance mode",
    ),
}


def controller_data(names):
    """The controller configuration as data, with a note per key."""
    manager = {
        "update_rate": (500, "Hz, equal to control_frequency of the robot definition"),
        "lock_memory": (
            True,
            "keep the process in RAM, so it never waits for the disk",
        ),
        "thread_priority": (60, "real-time priority of the control loop"),
        "cpu_affinity": (1, "the CPU core that runs the control loop"),
        "hardware_components_initial_state": {
            "shutdown_on_initial_state_failure": (
                False,
                "keep running when the hardware fails to start",
            ),
        },
    }
    for controller, (kind, note) in CONTROLLER_TYPES.items():
        manager[controller] = ({"type": kind}, note)
    return {
        "controller_manager": {"ros__parameters": manager},
        "drive_controller": {
            "ros__parameters": {
                "joints": names,
                "fault_reaction": (
                    Hint({"enabled": True}),
                    "quick stop all joints when one fails",
                ),
            }
        },
        "forward_position_controller": {
            "ros__parameters": {"joints": names, "interface_name": "position"}
        },
        "motor_command_controller": {"ros__parameters": {"joints": names}},
    }


def scalar(value):
    """A value in YAML notation."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def flow(items, indent, width=118):
    """A list of names in brackets, wrapped to `width` characters."""
    lines, line = [], "["
    for item in items:
        if len(line) + len(item) + 2 > width - indent and line != "[":
            lines.append(line.rstrip())
            line = " " * (indent + 2)
        line += item + ", "
    lines.append(line[:-2] + "]")
    return "\n".join(lines)


def yaml_lines(data, indent=0):
    """
    The lines of `data` in YAML, with the notes as comments.

    A value may be (value, note). A Hint becomes a commented-out line.
    """
    pad = " " * indent
    lines = []
    for key, item in data.items():
        value, note = item if isinstance(item, tuple) else (item, "")
        comment = f"  # {note}" if note else ""
        if isinstance(value, Hint):
            if isinstance(value.value, dict):
                lines.append(f"{pad}# {key}:{comment}")
                lines += [f"{pad}#   {k}: {scalar(v)}" for k, v in value.value.items()]
            else:
                lines.append(f"{pad}# {key}: {scalar(value.value)}{comment}")
        elif isinstance(value, dict):
            lines.append(f"{pad}{key}:{comment}")
            lines += yaml_lines(value, indent + 2)
        elif isinstance(value, list) and value and isinstance(value[0], dict):
            lines.append(f"{pad}{key}:{comment}")
            for entry in value:
                lines.append(
                    pad
                    + "  - {"
                    + ", ".join(f"{k}: {scalar(v)}" for k, v in entry.items())
                    + "}"
                )
        elif isinstance(value, list):
            lines.append(f"{pad}{key}: {flow(value, len(pad) + len(key) + 2)}{comment}")
        else:
            lines.append(f"{pad}{key}: {scalar(value)}{comment}")
    return lines


def document(header, data):
    """A YAML file: the header as comments, then the data."""
    comments = "".join(f"# {line}".rstrip() + "\n" for line in header.splitlines())
    sections = ["\n".join(yaml_lines({key: value})) for key, value in data.items()]
    return comments + "\n" + "\n\n".join(sections) + "\n"


def robot_definition(name, names):
    """The text of example_<name>.yaml for joints `names`."""
    header = (
        f"Example robot '{name}' with {len(names)} joint(s), made by generate_example_robot.py.\n"
        f"Its controllers are in example_{name}_controllers.yaml.\n\n{JOINT_KEYS_HELP}"
    )
    return document(header, robot_data(names))


def controller_config(name, names):
    """The text of example_<name>_controllers.yaml for joints `names`."""
    header = (
        f"Controllers of the example robot '{name}', made by generate_example_robot.py.\n"
        f"The joint lists follow example_{name}.yaml. forward_position_controller takes one position in rad per\n"
        "joint, motor_command_controller takes synapticon_control_msgs/MotorCommand."
    )
    return document(header, controller_data(names))


def joint_names(count, given):
    """The joint names: the comma separated `given` ones, or joint_1 to joint_<count>."""
    if given is None:
        return [f"joint_{number}" for number in range(1, count + 1)]
    names = [name.strip() for name in given.split(",") if name.strip()]
    if len(names) != count:
        raise ValueError(f"--names lists {len(names)} name(s) for --joints {count}")
    return names


def write(path, text, replace):
    """Write `text` to `path`. Raises FileExistsError when the file exists and `replace` is false."""
    if path.exists() and not replace:
        raise FileExistsError(
            f"{path} is already there, left as it is (--force overwrites)."
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    print(f"Created {path}")


def parse(argv):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--joints", type=int, required=True, help="how many joints, at least 1"
    )
    parser.add_argument(
        "--robot",
        help="the robot's name for robot:= and the file names (default: <joints>dof)",
    )
    parser.add_argument(
        "--names", help="comma separated joint names (default: joint_1, joint_2, ...)"
    )
    parser.add_argument("--force", action="store_true", help="replace files that exist")
    return parser.parse_args(argv)


def main(argv=None):
    """Write both files. Exit code 0 on success, 1 when a file exists, 2 when the robot cannot be made."""
    args = parse(argv)
    try:
        if args.joints < 1:
            raise ValueError("--joints needs at least 1 joint")
        names = joint_names(args.joints, args.names)
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    name = args.robot or f"{args.joints}dof"
    try:
        write(
            CONFIG_DIR / f"example_{name}.yaml",
            robot_definition(name, names),
            args.force,
        )
        write(
            CONFIG_DIR / f"example_{name}_controllers.yaml",
            controller_config(name, names),
            args.force,
        )
    except FileExistsError as error:
        print(error, file=sys.stderr)
        return 1
    print(
        f"Rebuild the package, then run: ros2 launch synapticon_bringup example_synapticon.launch.py robot:={name}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
