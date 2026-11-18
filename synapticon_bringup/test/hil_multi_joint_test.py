# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Hardware-in-the-loop test of what only a robot with several drives shows, through ros2_control.

On real drives it is not registered with colcon test: it needs every drive of the robot on the bus
and moves each joint about 0.2 rad. Run it on purpose, with every joint free to turn:

    SYNAPTICON_HIL_ROBOT=2dof launch_test \\
      $(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_multi_joint_test.py

SYNAPTICON_HIL_ROBOT selects the example robot (default 2dof, for example 28dof) or names a robot definition
file, SYNAPTICON_HIL_HAL the HAL
(default ethercat_soem, with its network interface in SYNAPTICON_HIL_NETWORK_INTERFACE, or ethercat_icube).
SYNAPTICON_HIL_SLAVE_CONFIG selects the slave_config (default: the installed example config). colcon test runs it on
simulated drives (mock_cia402). The robot runs in impedance mode, so every move goes through a spring, with the
drive controller's fault reaction switched on.

Steps:
1. The control loop with every drive on the bus, unpowered, for SYNAPTICON_HIL_SOAK_S seconds (default 60): the bus
   report on /diagnostics stays OK with a complete working counter, and the controller manager warns about overruns
   at most SYNAPTICON_HIL_MAX_OVERRUNS times (default 10 per minute of soak, which a host without real-time
   scheduling meets, or 0 for a strict check on a real-time host).
2. Drive actions on one named joint: enable and quick stop the first joint, the others stay unpowered.
3. One command moves all joints at once, there and back. Then a sweep moves one joint after the other by 0.3 rad
   and back, while the others hold, so every joint can be seen to move and each command reaches its own drive.
   SYNAPTICON_HIL_SWEEP_ONLY=1 runs only the sweep.
4. The fault reaction across joints: an injected emergency stop holds every joint, latched. An enable is refused
   until the reaction is cleared, and the joints then come back without a jump.
5. Disable all.

A watchdog cuts the voltage of every joint and fails the test if one strays more than 0.8 rad from its target.
"""

import math
import os
import re
import time

from diagnostic_msgs.msg import DiagnosticStatus
import launch_testing.markers
import pytest
from synapticon_bringup.testing import (
    drive_joints,
    example_controllers,
    example_joints,
    example_test_description,
    hil_launch_arguments,
    hil_robot,
    hil_test_gains,
    StackTestCase,
    write_yaml,
)
from synapticon_control_msgs.msg import DriveStatus, MotorCommand
from synapticon_control_msgs.srv import (
    ClearFaultReaction,
    InjectFaultEvent,
    SetDriveAction,
)

ROBOT = hil_robot("2dof")
JOINTS = example_joints(ROBOT)
if drive_joints(ROBOT) != JOINTS:
    raise ValueError(
        f"the multi-joint test needs a robot without transmissions, every joint with its own drive: {ROBOT}"
    )
SOAK_S = float(os.environ.get("SYNAPTICON_HIL_SOAK_S", "") or "60")
# A host without real-time scheduling misses a few cycles per minute, which the drives ride through.
OVERRUNS_PER_MINUTE = 10
MAX_OVERRUNS = int(
    os.environ.get("SYNAPTICON_HIL_MAX_OVERRUNS", "")
    or math.ceil(OVERRUNS_PER_MINUTE * SOAK_S / 60.0)
)
# Time after the last controller is active before the soak starts: switching controllers may overrun [s].
SETTLE_S = 3.0
REPUBLISH_S = 0.05
MOVE_RAD = 0.2
# Travel of each joint in the sweep, one joint after the other, large enough to see [rad].
SWEEP_RAD = 0.3
# SYNAPTICON_HIL_SWEEP_ONLY=1: only enable, sweep and disable, to watch every joint move.
SWEEP_ONLY = os.environ.get("SYNAPTICON_HIL_SWEEP_ONLY", "") not in ("", "0", "false")
TRACKING_TOLERANCE_RAD = 0.05
TRACKING_TIMEOUT_S = 5.0
# Travel allowed while a joint is held, after a quick stop or in the fault reaction [rad].
HOLD_TOLERANCE_RAD = 0.1
WATCHDOG_RAD = 0.8
GAINS = {
    name: hil_test_gains(ROBOT, name) for name in JOINTS
}  # Nm/rad and Nm s/rad of the impedance moves
STACK_TIMEOUT_S = 120.0  # must outlast the hardware's readiness_timeout_s
STATE_TIMEOUT_S = 3.0
# The two overrun warnings of the controller manager: a slow cycle, and a missed period.
OVERRUN = re.compile(rb"Overrun might occur|Overrun detected")


def controllers_with_fault_reaction():
    """The example robot's controller configuration with the fault reaction switched on, as a temporary file."""
    config = example_controllers(ROBOT)
    config["drive_controller"]["ros__parameters"]["fault_reaction"] = {"enabled": True}
    return write_yaml("hil_controllers_", config)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the example robot in impedance mode on the HAL under test."""
    arguments = hil_launch_arguments(ROBOT, True)
    arguments["controllers_file"] = controllers_with_fault_reaction()
    return example_test_description(arguments)


class TestMultiJoint(StackTestCase):
    """Drive actions, motion and the fault reaction over several joints."""

    PROBE_NAME = "hil_multi_joint_probe"

    def setUp(self):
        """Add the command publisher and the service clients to the probe."""
        super().setUp()
        node = self.probe.node
        self.targets = None
        self.last_publish = 0.0
        self.command_pub = node.create_publisher(
            MotorCommand, "/motor_command_controller/commands", 10
        )
        self.action_client = node.create_client(
            SetDriveAction, "/drive_controller/set_drive_action"
        )
        self.inject_client = node.create_client(
            InjectFaultEvent, "/drive_controller/inject_fault_event"
        )
        self.clear_client = node.create_client(
            ClearFaultReaction, "/drive_controller/clear_fault_reaction"
        )

    def tearDown(self):
        """Leave every drive de-energised."""
        powered = [
            name
            for name in JOINTS
            if self._state(name) not in (None, DriveStatus.STATE_SWITCH_ON_DISABLED)
        ]
        if powered:
            self._drive_action(SetDriveAction.Request.DISABLE_VOLTAGE, powered)
        super().tearDown()

    # --- Following the stack ------------------------------------------------------------------------------------

    def _state(self, name):
        drive = self.probe.drives.get(name)
        return drive.device_state if drive else None

    def _states(self, names=JOINTS):
        return {name: self._state(name) for name in names}

    def _positions(self):
        return {name: self.probe.positions.get(name) for name in JOINTS}

    def _on_cycle(self):
        """Keep streaming the targets and run the watchdog."""
        if self.targets is not None and time.time() - self.last_publish >= REPUBLISH_S:
            self._command(self.targets)
        self._watchdog()

    def _spin(self, seconds):
        self.probe.spin(seconds, self._on_cycle)

    def _wait_for(self, predicate, timeout_s, what):
        """Spin until `predicate` holds, failing with `what` and the drive states on timeout."""
        try:
            self.probe.wait_for(predicate, timeout_s, what, self._on_cycle)
        except AssertionError as error:
            self.fail(f"{error} (states {self._states()})")

    def _watchdog(self):
        """Cut the voltage of every joint and fail if one runs away from its target."""
        if self.targets is None:
            return
        for name, target in self.targets.items():
            position = self.probe.positions.get(name)
            if position is not None and abs(position - target) > WATCHDOG_RAD:
                self.targets = None
                self._drive_action(SetDriveAction.Request.DISABLE_VOLTAGE, JOINTS)
                self.fail(
                    f"watchdog: {name} at {position:.3f} rad, target {target:.3f} rad"
                )

    def _all_in(self, state, names=JOINTS):
        return lambda: all(self._state(name) == state for name in names)

    # --- Requests -------------------------------------------------------------------------------------------------

    def _drive_action(self, action, names):
        request = SetDriveAction.Request(action=action, dof_names=list(names))
        return self.probe.call(self.action_client, request)

    def _expect_accepted(self, action, names=JOINTS):
        response = self._drive_action(action, names)
        self.assertTrue(response.success, f"{action}: {response.message}")

    def _command(self, targets):
        """Publish one MotorCommand for every joint and make it the watchdog's reference."""
        msg = MotorCommand()
        msg.name = list(targets)
        msg.position = list(targets.values())
        msg.velocity = [0.0] * len(targets)
        msg.torque_feedforward = [0.0] * len(targets)
        msg.stiffness = [GAINS[name][0] for name in targets]
        msg.damping = [GAINS[name][1] for name in targets]
        self.command_pub.publish(msg)
        self.targets = dict(targets)
        self.last_publish = time.time()

    def _ramp_to(self, goal, duration_s=1.0, rate_hz=50.0):
        """Move every target smoothly to `goal`, all joints at once."""
        start = dict(self.targets)
        steps = int(duration_s * rate_hz)
        for i in range(1, steps + 1):
            self._command(
                {n: start[n] + (goal[n] - start[n]) * i / steps for n in JOINTS}
            )
            self._spin(1.0 / rate_hz)

    def _all_at_targets(self):
        positions = self._positions()
        return all(
            positions[n] is not None
            and abs(positions[n] - self.targets[n]) <= TRACKING_TOLERANCE_RAD
            for n in JOINTS
        )

    def _assert_held(self, reference, what):
        """Every joint is still within the hold tolerance of `reference`."""
        positions = self._positions()
        moved = {
            n: round(positions[n] - reference[n], 3)
            for n in JOINTS
            if abs(positions[n] - reference[n]) > HOLD_TOLERANCE_RAD
        }
        self.assertFalse(moved, f"joints moved {what}: {moved}")

    # --- Steps ----------------------------------------------------------------------------------------------------

    def _wait_for_stack(self):
        self._wait_for(
            lambda: all(
                self.probe.positions.get(n) is not None
                and self.probe.drives.get(n) is not None
                and self.probe.drives[n].online
                for n in JOINTS
            ),
            STACK_TIMEOUT_S,
            f"all {len(JOINTS)} joints of {ROBOT} online",
        )
        self.probe.wait_for_controllers(
            ["motor_command_controller", "drive_controller"], on_cycle=self._on_cycle
        )
        self._command(self._positions())

    def _soak(self, proc_output):
        """Run the loop unpowered: count overrun warnings and follow the bus report."""
        self._spin(SETTLE_S)
        overruns_before = sum(
            len(OVERRUN.findall(output.text)) for output in proc_output
        )
        worst = None
        deadline = time.time() + SOAK_S
        while time.time() < deadline:
            self._spin(0.5)
            bus = self.probe.bus
            if bus is not None and (worst is None or bus.level > worst.level):
                worst = bus
        overruns = (
            sum(len(OVERRUN.findall(output.text)) for output in proc_output)
            - overruns_before
        )
        print(
            f"Soak of {SOAK_S:.0f} s with {len(JOINTS)} drives: {overruns} overrun warning(s)",
            flush=True,
        )
        self.assertIsNotNone(worst, "no bus report on /diagnostics")
        values = {v.key: v.value for v in worst.values}
        self.assertEqual(
            worst.level,
            DiagnosticStatus.OK,
            f"bus report: {worst.message} {values}",
        )
        self.assertLessEqual(
            overruns,
            MAX_OVERRUNS,
            "overrun warnings of the controller manager during the soak",
        )

    def _one_named_joint(self):
        first, others = JOINTS[0], JOINTS[1:]
        self._expect_accepted(SetDriveAction.Request.ENABLE, [first])
        self._wait_for(
            self._all_in(DriveStatus.STATE_OPERATION_ENABLED, [first]),
            STATE_TIMEOUT_S,
            f"{first} in Operation Enabled",
        )
        self._spin(0.5)
        self.assertTrue(
            self._all_in(DriveStatus.STATE_SWITCH_ON_DISABLED, others)(),
            f"an enable of {first} powered others: {self._states(others)}",
        )
        self._expect_accepted(SetDriveAction.Request.QUICK_STOP, [first])
        self._wait_for(
            self._all_in(DriveStatus.STATE_QUICK_STOP_ACTIVE, [first]),
            STATE_TIMEOUT_S,
            f"{first} in Quick Stop Active",
        )
        self._spin(0.5)
        self.assertTrue(
            self._all_in(DriveStatus.STATE_SWITCH_ON_DISABLED, others)(),
            f"a quick stop of {first} changed others: {self._states(others)}",
        )
        self._expect_accepted(SetDriveAction.Request.DISABLE_VOLTAGE, [first])
        self._wait_for(
            self._all_in(DriveStatus.STATE_SWITCH_ON_DISABLED),
            STATE_TIMEOUT_S,
            "every joint in Switch On Disabled",
        )

    def _enable_all(self):
        """Hold every joint where it is and enable all of them."""
        self._command(self._positions())
        self._spin(0.5)
        self._expect_accepted(SetDriveAction.Request.ENABLE)
        self._wait_for(
            self._all_in(DriveStatus.STATE_OPERATION_ENABLED),
            STATE_TIMEOUT_S,
            "every joint in Operation Enabled",
        )

    def _disable_all(self):
        self._expect_accepted(SetDriveAction.Request.DISABLE)
        self._wait_for(
            self._all_in(DriveStatus.STATE_SWITCH_ON_DISABLED),
            STATE_TIMEOUT_S,
            "every joint in Switch On Disabled",
        )
        self.targets = None

    def _move_all_at_once(self):
        start = dict(self.targets)
        self._ramp_to({n: start[n] + MOVE_RAD for n in JOINTS})
        self._wait_for(
            self._all_at_targets, TRACKING_TIMEOUT_S, "every joint at its target"
        )
        self._ramp_to(start)
        self._wait_for(
            self._all_at_targets, TRACKING_TIMEOUT_S, "every joint back at its start"
        )

    def _sweep(self):
        """Move one joint after the other out and back, while the others hold. Each command must move its joint."""
        for name in JOINTS:
            print(f"Sweep: {name}", flush=True)
            start = dict(self.targets)
            self._ramp_to(start | {name: start[name] + SWEEP_RAD}, duration_s=0.8)
            self._wait_for(
                self._all_at_targets, TRACKING_TIMEOUT_S, f"{name} at its sweep target"
            )
            self._ramp_to(start, duration_s=0.8)
            self._wait_for(
                self._all_at_targets, TRACKING_TIMEOUT_S, f"{name} back at its start"
            )

    def _fault_reaction(self):
        before = self._positions()
        response = self.probe.call(
            self.inject_client,
            InjectFaultEvent.Request(event="emergency_stop", dof_names=[]),
        )
        self.assertTrue(response.accepted, response.message)
        self._wait_for(
            lambda: all(
                self._state(n) == DriveStatus.STATE_QUICK_STOP_ACTIVE
                and self.probe.drives[n].latched
                for n in JOINTS
            ),
            STATE_TIMEOUT_S,
            "every joint held by the fault reaction",
        )
        response = self._drive_action(SetDriveAction.Request.ENABLE, JOINTS)
        self.assertFalse(
            response.success, "an enable was accepted during the fault reaction"
        )
        self.assertIn("fault reaction", response.message)
        self._spin(1.0)
        self._assert_held(before, "during the fault reaction")

        response = self.probe.call(
            self.clear_client, ClearFaultReaction.Request(dof_names=[])
        )
        self.assertTrue(response.success, response.message)
        self._wait_for(
            lambda: not any(self.probe.drives[n].latched for n in JOINTS),
            STATE_TIMEOUT_S,
            "the fault reaction cleared on every joint",
        )
        self._expect_accepted(SetDriveAction.Request.ENABLE)
        self._wait_for(
            self._all_in(DriveStatus.STATE_OPERATION_ENABLED),
            STATE_TIMEOUT_S,
            "every joint in Operation Enabled after the clear",
        )
        self._spin(1.0)
        self._assert_held(before, "after the enable that followed the clear")

    def test_multi_joint(self, proc_output):
        """The steps in order, on one launch of the robot."""
        self.assertGreater(len(JOINTS), 1, f"{ROBOT} has one joint only")
        self._wait_for_stack()
        if SWEEP_ONLY:
            self._enable_all()
            self._sweep()
            self._disable_all()
            return
        self._soak(proc_output)
        self._one_named_joint()
        self._enable_all()
        self._move_all_at_once()
        self._sweep()
        self._fault_reaction()
        self._disable_all()
