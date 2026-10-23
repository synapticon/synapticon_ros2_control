#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Run the hardware-in-the-loop tests one after the other and print a summary.

The tests ENERGISE AND MOVE the drive at the ring position of joint_1 (see the HIL suite and the
path test). Run it with the joint free to turn about one revolution and the ROS stack not running,
inside net_raw_shell.sh --interface <if>:

    ros2 run synapticon_bringup run_hil_tests.py --network-interface <if>   # one drive
    ros2 run synapticon_bringup run_hil_tests.py --network-interface <if> --two-drives --robot 28dof

Steps: the HIL suite of the EtherCAT HAL in strict mode, the ros2_control path test in CSP and in
impedance mode, and the bring-up test of the example robot in both modes. Every step runs on the HAL
chosen with --hal. Each step's output goes to the console and to a log file. The exit code is 0 when
every step passed.
"""

import argparse
from dataclasses import dataclass, field
import datetime
import os
import pathlib
import re
import signal
import subprocess
import sys
import threading
import time

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)

# Time for the master to release the bus between two steps [s].
BUS_RELEASE_S = 8.0
# Time a step gets to exit after SIGINT, before it is killed [s].
INTERRUPT_GRACE_S = 20.0


@dataclass
class Step:
    """One test run: a command, its extra environment and its time limit."""

    name: str
    command: list[str]
    env: dict[str, str] = field(default_factory=dict)
    timeout_s: float = 300.0


# The package and the HIL suite executable of each EtherCAT HAL.
SUITES = {
    "ethercat_soem": ("synapticon_ethercat_soem", "ethercat_soem_hil_test"),
}


def suite_binary(hal):
    """Path of the HIL suite of `hal`, or an empty string when it is not built."""
    package, executable = SUITES[hal]
    try:
        prefix = get_package_prefix(package)
    except PackageNotFoundError:
        return ""
    path = os.path.join(prefix, "lib", package, executable)
    return path if os.path.exists(path) else ""


def hal_environment(args):
    """The environment that selects the HAL in every step."""
    env = {"SYNAPTICON_HIL_HAL": args.hal}
    if args.network_interface:
        env["SYNAPTICON_HIL_NETWORK_INTERFACE"] = args.network_interface
    return env


def build_steps(args):
    """The steps selected on the command line, in run order."""
    test_dir = os.path.join(get_package_share_directory("synapticon_bringup"), "test")
    path_test = os.path.join(test_dir, "hil_ros2_control_test.py")
    bringup_test = os.path.join(test_dir, "hil_bringup_test.py")
    hal = hal_environment(args)
    impedance = hal | {"SYNAPTICON_HIL_IMPEDANCE": "1"}
    robot = hal | {"SYNAPTICON_HIL_ROBOT": args.robot}
    steps = {
        "suite": [
            Step(
                "HIL suite" + (", two drives" if args.two_drives else ""),
                [suite_binary(args.hal)]
                + (["--two-drives"] if args.two_drives else []),
                hal | {"SYNAPTICON_HIL_STRICT": "1"},
                timeout_s=900.0,
            )
        ],
        "path": [
            Step("path test, CSP", ["launch_test", path_test], hal),
            Step("path test, impedance", ["launch_test", path_test], impedance),
        ],
        "bringup": [
            Step(f"bring-up {args.robot}, CSP", ["launch_test", bringup_test], robot),
            Step(
                f"bring-up {args.robot}, impedance",
                ["launch_test", bringup_test],
                robot | impedance,
            ),
        ],
    }
    return [step for name in args.only for step in steps[name]]


def stop(process):
    """Interrupt the step's process group, and kill it if it does not exit in time."""
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=INTERRUPT_GRACE_S)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def ignore_stop_signals():
    """Ignore further SIGINT and SIGTERM, so that a repeated Ctrl-C cannot cut short a step's stop."""
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, signal.SIG_IGN)


def run_step(step, log_path):
    """
    Run one step, copying its output to the console and the log.

    SIGINT or SIGTERM stops the step as its own Ctrl-C would. Returns its exit code, its output and whether it was
    interrupted.
    """
    lines = []
    interrupted = False
    with open(log_path, "w", encoding="utf-8") as log:
        process = subprocess.Popen(
            step.command,
            env=os.environ | step.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            start_new_session=True,
        )

        def copy():
            for line in process.stdout:
                sys.stdout.write(line)
                log.write(line)
                lines.append(line)

        reader = threading.Thread(target=copy)
        reader.start()
        try:
            process.wait(timeout=step.timeout_s)
        except subprocess.TimeoutExpired:
            lines.append(f"TIMEOUT after {step.timeout_s:.0f} s\n")
            stop(process)
        except KeyboardInterrupt:
            ignore_stop_signals()
            lines.append("INTERRUPTED by SIGINT or SIGTERM\n")
            stop(process)
            interrupted = True
        reader.join()
    return process.returncode, lines, interrupted


def summarize(lines):
    """A short result line from a step's output: the gtest counts and the skipped tests."""
    text = "".join(lines)
    counts = re.findall(r"\[ +(PASSED|SKIPPED|FAILED) +\] (\d+) tests?", text)
    parts = [f"{count} {state.lower()}" for state, count in counts]
    for state in ("SKIPPED", "FAILED"):
        names = sorted(set(re.findall(rf"\[ +{state} +\] (\w+\.\w+)", text)))
        if names:
            parts.append(f"{state.lower()}: " + ", ".join(names))
    parts += re.findall(
        r"^(FAILED \(.*\)|TIMEOUT after .*|INTERRUPTED .*)$", text, re.MULTILINE
    )
    return ", ".join(parts)


def parse_args():
    """Command line options."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument(
        "--two-drives",
        action="store_true",
        help="run the suite's two drive tests (a second drive at the next ring position)",
    )
    parser.add_argument(
        "--hal",
        choices=sorted(SUITES),
        default="ethercat_soem",
        help="EtherCAT HAL every step runs on (default ethercat_soem)",
    )
    parser.add_argument(
        "--network-interface",
        default=os.environ.get("SYNAPTICON_HIL_NETWORK_INTERFACE", ""),
        help="network interface of the drives, needed by ethercat_soem "
        "(default SYNAPTICON_HIL_NETWORK_INTERFACE)",
    )
    parser.add_argument(
        "--robot",
        default="1dof",
        help="example robot of the bring-up test, every drive of it must be on the bus",
    )
    parser.add_argument(
        "--only",
        nargs="+",
        choices=["suite", "path", "bringup"],
        default=["suite", "path", "bringup"],
        help="run only these steps",
    )
    default_log_dir = pathlib.Path.home() / ".ros" / "log" / "hil"
    parser.add_argument(
        "--log-dir",
        type=pathlib.Path,
        default=default_log_dir,
        help=f"parent of the log directory of this run (default {default_log_dir})",
    )
    return parser.parse_args()


def main():
    """Run the selected steps and print the summary."""
    args = parse_args()
    if args.hal == "ethercat_soem" and not args.network_interface:
        sys.exit("ethercat_soem needs --network-interface")
    steps = build_steps(args)
    if any(not step.command[0] for step in steps):
        switch = (
            " with SYNAPTICON_HAL_ETHERCAT_ICUBE=ON"
            if args.hal == "ethercat_icube"
            else ""
        )
        sys.exit(
            f"{SUITES[args.hal][1]} is not installed: build {SUITES[args.hal][0]}{switch}"
        )
    log_dir = args.log_dir / datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir.mkdir(parents=True)

    # SIGTERM stops the runner like Ctrl-C.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    results = []
    for index, step in enumerate(steps):
        if index > 0:
            try:
                time.sleep(BUS_RELEASE_S)
            except KeyboardInterrupt:
                break
        print(f"\n===== {step.name}", flush=True)
        log_path = (
            log_dir / f"{index + 1}_{re.sub(r'[^a-z0-9]+', '_', step.name.lower())}.log"
        )
        code, lines, interrupted = run_step(step, log_path)
        results.append(
            (step.name, code == 0 and not interrupted, summarize(lines), log_path)
        )
        if interrupted:
            break

    print("\n===== Summary")
    for name, passed, summary, log_path in results:
        print(f"{'PASS' if passed else 'FAIL'}  {name}: {summary or '-'}  ({log_path})")
    sys.exit(0 if all(passed for _, passed, _, _ in results) else 1)


if __name__ == "__main__":
    main()
