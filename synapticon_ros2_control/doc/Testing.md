# Testing

This guide shows how to test the packages.
It starts with tests that need no hardware and ends with tests that move a real drive.

| Test | What you need | Does the motor move? | Section |
|---|---|---|---|
| Unit and mock launch tests | nothing | no | [1](#1-unit-and-mock-launch-tests) |
| Hardware-in-the-loop suite | one drive, two for the two-drive tests | yes | [3](#3-hardware-in-the-loop-suite) |
| ros2_control path test | one drive | yes | [4](#4-ros2_control-path-test) |
| Robot bring-up test | every drive of the robot | no | [5](#5-robot-bring-up-test) |
| Multi-joint test | every drive of a robot with two or more | yes | [6](#6-multi-joint-test) |
| Sections 3 to 6 in one run | as above | yes | [7](#7-all-automated-hardware-tests-in-one-run) |
| Drive check | one drive | only `hold` and `move` | [8](#8-drive-check) |
| Manual tests | a person, one drive | yes | [9](#9-manual-tests) |

`colcon test` runs only section 1.
The hardware tests are built together with the unit tests, but `colcon test` never starts them.
They run only when you start them yourself.

Each section is a small test plan: what you need, the commands, what you should see, and how to leave the drive.
The appendix holds the details of the suite: what it checks, and how the drive behaves on the bench.

## 1. Unit and mock launch tests

The unit tests need no drive and no bus, and no IgH master for the build without it.
They check the CiA 402 state machine, the drive actions, the HALs, the SOEM HAL's process image, the ros2_control
layer, and the example launch on the mock HAL.

They also cover failures that a bench rarely shows:

- bring-up against a scripted drive: old firmware, no torque sensor, objects that cannot be read, and for the
  EtherCAT ICube HAL a bus that does not activate
- `Cia402Hal` with drives that fault, never come up, or sit on a failing bus
- the controllers with missing parameters or interfaces
- `esi_to_slave_config.py`: for a small ESI file written inline in the test, it must write the bundled example
  slave_config
- `generate_example_robot.py`: it must describe the example robots

The drive action suite of [section 3](#3-hardware-in-the-loop-suite) runs here too, on simulated drives
(`mock_cia402`).
It takes about a minute.
So a change to the suite or to the drive logic shows up before you go to the bench.

1. Build and run the tests in your colcon workspace:

   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   colcon test --packages-select synapticon_hardware synapticon_ethercat_icube synapticon_ethercat_soem synapticon_controllers synapticon_bringup synapticon_description
   colcon test-result --verbose
   ```

2. Expect a summary with `0 errors, 0 failures`.
   When a test fails, `colcon test-result --verbose` lists it with its output.

Each launch test runs in its own ROS domain, with discovery on this machine only.
So a ROS stack elsewhere on the network never gets its requests, and cannot disturb it.

## 2. Before any hardware test

All hardware tests run on one of the two EtherCAT HALs:

- `ethercat_soem`, an EtherCAT master that runs as a normal program, the default.
  It needs a shell with the raw socket capability.
- `ethercat_icube`, on the IgH master, see [With the EtherCAT ICube HAL](#with-the-ethercat-icube-hal)

Sections 3 to 8 say how to switch each test to the EtherCAT ICube HAL.

### What you need

- A SOMANET drive on firmware 5.6.11 or newer.
  For a joint in impedance mode, bring-up refuses older firmware, or firmware it cannot read.
  For the other modes it only warns.
  [FirmwareUpdate.md](FirmwareUpdate.md) shows how to update.
- The drive powered and wired to a network interface of the host.
- For `ethercat_soem`: no other EtherCAT master on that interface. If the IgH master is installed, the script of
  step 2 stops it.
- For `ethercat_icube`: the IgH EtherCAT master installed, and access to `/dev/EtherCAT0`.
  That means root, a udev rule, or a group membership.
- Room for the joint to turn about one output revolution both ways.
  Nothing attached that a sudden stop could damage.
  Keep a hand near the power switch or the e-stop.
- For the impedance tests: a torque sensor configured on the drive, so `0x2150:1` (sensor type) is not 0.
- The workspace built with the HAL under test and with tests (the default), in `Release`.

### Steps

1. Find the network interface of the drive with `ip link`.

2. In your colcon workspace, open a shell with the raw socket capability.
   sudo resets the environment, so source the workspace again inside it:

   ```bash
   source install/setup.bash
   "$(ros2 pkg prefix synapticon_ethercat_soem)/lib/synapticon_ethercat_soem/net_raw_shell.sh" --interface <interface>
   grep CapAmb /proc/self/status   # expect 0000000000002000
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

   With `--interface`, the script does three things.
   It stops the IgH master if it runs, brings the interface up, and sets `SYNAPTICON_HIL_NETWORK_INTERFACE` for the
   tests.
   Run the SOEM tests in this shell, and `exit` it when you are done.
   If the script stopped the IgH master, it starts it again when the shell ends.

3. Check that the build is optimised.
   The packages fall back to `Release` when no build type is given.
   But an old build directory can still hold another type.

   ```bash
   grep CXX_FLAGS build/synapticon_hardware/CMakeFiles/synapticon_hardware.dir/flags.make
   grep CXX_FLAGS build/synapticon_ethercat_soem/CMakeFiles/synapticon_ethercat_soem.dir/flags.make
   grep CXX_FLAGS build/synapticon_ethercat_icube/CMakeFiles/synapticon_ethercat_icube.dir/flags.make
   ```

   Each line should contain `-O3 -DNDEBUG`.
   If not, rebuild with `--cmake-args -DCMAKE_BUILD_TYPE=Release`.
   If the type does not change, remove the package's `build` directory first.

4. Choose the slave_config.
   The example fits the Circulo it was generated for.
   For another drive, generate one from the drive's ESI file with `esi_to_slave_config.py` of `synapticon_hardware`.

   ```bash
   CFG="$(ros2 pkg prefix synapticon_hardware)/share/synapticon_hardware/config/example_somanet_circulo_safe_motion.yaml"
   ```

When a step fails, see [Troubleshooting](#10-troubleshooting).

### With the EtherCAT ICube HAL

The EtherCAT ICube HAL runs on the IgH master, so it needs no raw socket shell.
Use a normal terminal with the workspace sourced in place of step 2, not the shell of step 2: that shell stops the
IgH master.

1. Start the master, and check that it sees the drive:

   ```bash
   sudo systemctl start ethercat
   ethercat slaves -v
   ```

   Expect your drive with its product code, its revision and the state `PREOP`.
   The Circulo of the example slave_config has the product code `0x00000302`.
   Note the alias and the ring position.
   For one drive directly on the network interface, both are usually `0`.

2. Source the workspace, then go on with steps 3 and 4 above:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

## 3. Hardware-in-the-loop suite

`ethercat_soem_hil_test` runs `EthercatSoemHal` directly at 500 Hz, the default bus rate, without ros2_control.
`ethercat_icube_hil_test` does the same with `EthercatIcubeHal`.
They hold two kinds of tests:

- the drive action suite, which works on any HAL and also runs on simulated drives in `colcon test`
- the EtherCAT tests, which bring the drive up with one change each and check what bring-up reports

Each test asks for the drive settings it needs: quick stop option code and deceleration, following error reaction,
mode.
The suite turns them into `sdo:` entries of a slave_config, built on a base config.

A watchdog protects the joint.
When the joint travels further than the test expects, it cuts the voltage and stops the test.
After a trip, and after any other fatal failure, every later cycle of that test keeps the voltage cut.
Ctrl-C or SIGTERM fails the running test the same way, then the suite releases the bus and skips every later test.

### Steps

1. Run the suite in strict mode, with a kill timeout on the binary itself.
   Use the shell of [section 2](#2-before-any-hardware-test).

   ```bash
   SYNAPTICON_HIL_STRICT=1 timeout -s KILL 900 \
     "$(ros2 pkg prefix synapticon_ethercat_soem)/lib/synapticon_ethercat_soem/ethercat_soem_hil_test"
   ```

   A run takes about five minutes.
   The usual gtest options work, for example `--gtest_filter=*QuickStop*`.
   The timeout ends a bring-up that never returns (see [Releasing the bus](#releasing-the-bus)).
   Call the binary directly as above, because `ros2 run` does not pass the timeout's signal on.

2. With one drive, expect:

   ```text
   [  PASSED  ] 53 tests.
   [  SKIPPED ] 4 tests, listed below:
   [  SKIPPED ] DrivePair.ADisableVoltageOfOneLeavesTheOtherInQuickStop
   [  SKIPPED ] DrivePair.AFaultOnOneLeavesTheOtherRunning
   [  SKIPPED ] DrivePair.AQuickStopOfOneLeavesTheOtherEnabledAndMoving
   [  SKIPPED ] DrivePair.BothComeUpAndGoDownTogether
   ```

   A failed test is listed under `[  FAILED  ]`.
   When the watchdog trips, it prints `WATCHDOG: travelled more than ...` with the statusword of that moment.

   For the EtherCAT ICube HAL, run its binary the same way, in the terminal of [With the EtherCAT ICube HAL](#with-the-ethercat-icube-hal):

   ```bash
   SYNAPTICON_HIL_STRICT=1 timeout -s KILL 900 \
     "$(ros2 pkg prefix synapticon_ethercat_icube)/lib/synapticon_ethercat_icube/ethercat_icube_hil_test"
   ```

   Expect the same, with 52 passed: the SOEM HAL has one bring-up test more.
   A test stuck in the bus activation of the EtherCAT driver ignores SIGINT and SIGTERM, so keep `-s KILL`.

3. With a second drive on the bus, also run the two-drive tests.
   A drive of another model needs its own slave_config, given in a robot definition (see
   [Your own bench](#your-own-bench)):

   ```bash
   timeout -s KILL 900 "$(ros2 pkg prefix synapticon_ethercat_soem)/lib/synapticon_ethercat_soem/ethercat_soem_hil_test" \
     --two-drives --gtest_filter=DrivePair.*
   ```

   Expect 4 passed.
   Here the watchdog cuts both drives when either one strays too far from its commanded position.
   With only one drive on the bus, `--two-drives` fails the bring-up at once, before any drive gets power.
   For the EtherCAT ICube HAL, use `ethercat_icube_hil_test` of `synapticon_ethercat_icube` in the same way.

Without `SYNAPTICON_HIL_STRICT=1`, an impedance test skips when the impedance bring-up fails.
That keeps a bench without a torque sensor usable.
In strict mode the same case is a failure.
On a commissioned bench, always run strict.

An acceptance run passes when only the four `DrivePair` tests skip (without `--two-drives`).
One more skip is allowed, because it comes from a timing race:
`AStopRequestedDuringTheFaultReactionDoesNotResetTheFault` may skip with
"the drive went to Fault before a request could reach Fault Reaction Active".
Write it down, and look into it when it comes back.

### Settings

| Variable | Default | Meaning |
|---|---|---|
| `SYNAPTICON_HIL_ROBOT` | unset | robot definition file of the bench, see [Your own bench](#your-own-bench). Its first joint is the drive under test, its second the second drive. Its `bus` section sets the bus, except `control_frequency`: the suite always runs at 500 Hz |
| `SYNAPTICON_HIL_SLAVE_CONFIG` | the installed example slave_config | base slave_config of a drive that names none |
| `SYNAPTICON_HIL_NETWORK_INTERFACE` | the robot file's `network_interface` | network interface of the drives, needed by `ethercat_soem_hil_test` |
| `SYNAPTICON_HIL_MASTER` | the robot file's `master_id`, or `0` | IgH master index, `ethercat_icube_hil_test` only |
| `SYNAPTICON_HIL_ALIAS` | `0` | slave alias, only `0` is supported |
| `SYNAPTICON_HIL_POSITION` | `0`, or the robot file's | ring position, overrides the robot file |
| `SYNAPTICON_HIL_POSITION_2` | the robot file's second joint, or `SYNAPTICON_HIL_POSITION` + 1 | ring position of the second drive, for `--two-drives` |
| `SYNAPTICON_HIL_STRICT` | unset | `1` turns a failed impedance bring-up into a failure instead of a skip |

The impedance tests move a joint with the test gains of the robot definition's `hil_tests` section.
Without them, they take stiffness 15 Nm/rad and damping 0.5 Nm s/rad, with a torque ceiling of 50 Nm unless the joint
sets `max_joint_torque`.
A robot definition that gives a joint its own test gains must also give it `max_joint_torque`, per joint or in
`command_defaults`.
Otherwise every impedance test fails with a message that says so, also outside strict mode.

### What the suite leaves on the drive

A drive keeps its SDO values until it is power-cycled or they are written again.
The suite writes these:

- the quick stop option code `0x605A` and the quick stop deceleration `0x6085`
- the following error objects `0x2017:1`, `0x6065` and `0x6066`

Every bring-up writes the vendor defaults of the following error objects, unless the test sets them itself.
So no test starts with the armed detection of an earlier fault test.

At the end, the suite writes back the `0x6085` it found on each drive.
`0x605A` keeps the value of the base config (6 for the example config).
Four bring-up tests change drive objects for a bring-up and write the old values back right after:

- MaxJointTorque (`0x2054`), set to 0
- the torque sensor type (`0x2150:1`), set to None
- the max motor speed (`0x6080`) and the following error window (`0x6065`), set in SI units
- the drive's impedance gains (`0x2060:1` and `0x2060:2`), set to marked values

A test that fails or is killed between the change and the write-back leaves the changed value on the drive.

To get the drive back exactly as commissioned, power-cycle it after the run.

The drive does a few surprising things during these tests.
[The appendix](#how-the-drive-behaves-on-the-bench) lists them, with the numbers measured on the bench.

### Adding a test

The suite lives in `synapticon_hardware`, as headers for test executables:

- `testing/hal_bench.hpp` holds the fixtures `SingleDrive` and `DrivePair`.
- `testing/cia402_hal_tests.hpp` pulls in the tests, by topic, from `testing/suite/`.

A test asks for its drive settings in `start()`, for example `start({ .quick_stop_deceleration = 2.0 })`.
A test that moves the joint arms the watchdog with `limit_travel()` right after `energize()`, before the first motion.
It calls each fixture helper that cycles in `ASSERT_NO_FATAL_FAILURE`, and waits for a state with `reach()`.
So a watchdog trip or a stop request ends the test there, and its own checks add no failures.
It uses the HAL only through `Cia402Hal`.

Each HAL brings a `HalBench`, which applies the settings in its own way.
A test that only makes sense for one HAL goes into that HAL's package, next to its bench.

The EtherCAT HALs share one bench.
`testing/ethercat_hal_tests.hpp` holds it, together with the EtherCAT tests and the `main` of the test executable.
Each HAL's test executable only names its HAL and sets it up on the bench's bus.

With the IgH master, read drive settings (SDO) before `start()`, or while the bench cycles the bus.
Once the bus is started, IgH answers a blocking read only while the program sends cycles.
Without them, the read never returns: the test hangs in the kernel, ignores even `kill -9`, and only a reboot frees
the master.

## 4. ros2_control path test

`hil_ros2_control_test.py` runs the whole path a user runs: the example launch, `SynapticonSystemInterface` and the
controllers.
It brings up the 1 DOF example on the HAL of `SYNAPTICON_HIL_HAL` (`ethercat_soem` by default, or `ethercat_icube`) and
then:

1. enables the drive
2. ramps 0.3 rad away
3. quick stops
4. enables again
5. ramps back
6. disables

A watchdog cuts the voltage when the joint strays more than 0.8 rad from its commanded target.
On a robot with a transmission, the test commands the output joint and switches the drive of its motor.
Along the way, the test checks that the drive's entry on `/diagnostics` follows the enable and the quick stop.

### Steps

1. Run it in CSP, in the shell of [section 2](#2-before-any-hardware-test):

   ```bash
   launch_test "$(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_ros2_control_test.py"
   ```

2. Wait about 8 s.
   Right after the bus is released, the drive's mailbox is not ready for the next bring-up.

3. Run it in impedance mode, through `MotorCommandController`:

   ```bash
   SYNAPTICON_HIL_IMPEDANCE=1 launch_test \
     "$(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_ros2_control_test.py"
   ```

4. Expect `OK` at the end of each run.
   A failure names the step and the measured position, for example a target not reached within the tracking
   tolerance.

The test ends with the drive disabled and the bus released.

Settings:

- `SYNAPTICON_HIL_SLAVE_CONFIG` chooses the slave_config for the launch.
- `SYNAPTICON_HIL_ROBOT` chooses the robot: an example robot with one joint (default `1dof`, its drive at ring
  position 0), or a robot definition file with one joint. For another drive, use a file, see
  [Your own bench](#your-own-bench).
- `SYNAPTICON_HIL_NETWORK_INTERFACE` names the network interface of the drives, and becomes the launch argument
  `network_interface`. The shell of [section 2](#2-before-any-hardware-test) sets it.
- `SYNAPTICON_HIL_HAL` names the HAL (default `ethercat_soem`).
  For `ethercat_icube`, run in the terminal of [With the EtherCAT ICube HAL](#with-the-ethercat-icube-hal), without `SYNAPTICON_HIL_NETWORK_INTERFACE`.

`colcon test` runs this test on simulated drives (`mock_cia402`) in both modes.
So a change to the test or to the stack shows up before you go to the bench.

## 5. Robot bring-up test

`hil_bringup_test.py` brings up a whole example robot and checks every drive.
It gives no drive power.
Run it on a multi-drive setup after a change to the robot definition, the slave_config or the bring-up.

It passes when:

- bring-up reports every drive of the robot definition streaming
- bring-up prints no framed warning block and no torque limit of zero
- every joint is on `/joint_states` with a finite position
- every drive is online, without power, free of errors, not latched, and in the requested mode
- `/diagnostics` has an entry for every drive, online and without power
- the controllers are active, and the bus monitor reports a complete domain

In impedance mode, bring-up also checks the firmware version and the torque sensor of every drive.
So an impedance run doubles as the commissioning check of a multi-drive setup.

### Steps

1. Choose the example robot whose drives are all on the bus, for example `28dof`.
   Every drive of its robot definition must answer at its ring position.

2. Run it in CSP:

   ```bash
   SYNAPTICON_HIL_ROBOT=28dof launch_test \
     "$(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_bringup_test.py"
   ```

3. Wait about 8 s, then run it in impedance mode:

   ```bash
   SYNAPTICON_HIL_ROBOT=28dof SYNAPTICON_HIL_IMPEDANCE=1 launch_test \
     "$(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_bringup_test.py"
   ```

4. Expect `OK` at the end of each run.
   A failed bring-up fails the test at once, and lists its errors and warning blocks.

`SYNAPTICON_HIL_ROBOT` defaults to `1dof`, and also takes a robot definition file.
`SYNAPTICON_HIL_HAL`, `SYNAPTICON_HIL_NETWORK_INTERFACE` and `SYNAPTICON_HIL_SLAVE_CONFIG` work as in the path test.
`colcon test` runs this test with the 2 DOF example on simulated drives, in both modes.

## 6. Multi-joint test

`hil_multi_joint_test.py` checks what only a robot with several drives shows, through the running stack.
Everything that one drive covers stays with the suite and the path test.
The robot runs in impedance mode, with the fault reaction of the drive controller switched on.
Each joint moves about 0.2 rad.

It runs these steps on one launch:

1. The control loop with every drive on the bus, without power, for 60 s.
   The bus report on `/diagnostics` stays OK with a complete domain, and the controller manager prints at most 10
   overrun warnings per minute.
2. Drive actions on one named joint: the first joint is enabled and quick stopped, the others stay without power.
3. One command moves all joints at once, and back.
   Then a sweep moves one joint after the other by 0.3 rad and back, while the others hold.
   You can watch every joint move, and the test checks that each command moves its own joint and no other.
4. The fault reaction across joints: an injected `emergency_stop` holds every joint, latched.
   An enable is refused until the reaction is cleared. After the clear, an enable brings every joint back without a
   jump.
5. All joints are disabled.

A watchdog cuts the voltage of every joint and fails the test when one strays more than 0.8 rad from its target.

### Steps

1. Choose the example robot whose drives are all on the bus, for example `2dof` or `28dof`.
   Every joint must be free to turn by about 1 rad.
   The bring-up test of [section 5](#5-robot-bring-up-test) in impedance mode checks the commissioning first.

2. Run it:

   ```bash
   SYNAPTICON_HIL_ROBOT=28dof launch_test \
     "$(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/test/hil_multi_joint_test.py"
   ```

3. Expect `OK` at the end, and a line with the overrun count of the soak, for example
   `Soak of 60 s with 28 drives: 3 overrun warning(s)`.

To only watch every joint move, run the sweep alone with `SYNAPTICON_HIL_SWEEP_ONLY=1`, or
`run_hil_tests.py --robot <robot> --only sweep` (about 3 s per joint).
`SYNAPTICON_HIL_SOAK_S` sets the length of the soak in seconds, `SYNAPTICON_HIL_MAX_OVERRUNS` the number of overrun
warnings it allows (default 10 per minute of soak).
The default suits a PC without real-time scheduling, which misses a cycle now and then.
On a real-time PC, set it to 0 for a strict check.
Overruns also come from other load on the machine (a build, for example), so run the soak on an otherwise idle machine.
If the limit is exceeded, launch the 1 DOF example for a minute without power and count its overrun warnings:
a similar count means the machine, not the number of drives.
`SYNAPTICON_HIL_ROBOT` defaults to `2dof`, and also takes a robot definition file.
`SYNAPTICON_HIL_HAL`, `SYNAPTICON_HIL_NETWORK_INTERFACE` and `SYNAPTICON_HIL_SLAVE_CONFIG` work as in the path test.
`colcon test` runs this test with the 2 DOF example on simulated drives, with a 5 s soak and no overrun limit.

## 7. All automated hardware tests in one run

`run_hil_tests.py` runs the automated hardware tests that fit the robot, waits between the runs, prints a summary,
and exits with 0 only when every run passed.
It is installed in every build, together with the tests it runs.
It counts the robot's drives, not its joints: the output of a transmission has no drive.
Its runs keep ROS discovery on this machine, so they never reach a stack on another bench.

Ctrl-C, a closed terminal or a lost SSH session stop the running test the way its own Ctrl-C would.
A test that runs out of time is stopped the same way and fails.
An interrupted run exits with 1 and says how many tests did not run.

With one joint (the default `--robot 1dof`), it runs:

- the suite, in strict mode
- the path test, in CSP and in impedance mode
- the bring-up test, in CSP and in impedance mode

With several joints, for example `--robot 2dof` or `--robot 28dof`, it runs only what needs several drives:

- the two-drive tests of the suite, on the robot's first two drives
- the bring-up test of the whole robot, in CSP and in impedance mode
- the multi-joint test of [section 6](#6-multi-joint-test)

Prepare the drives as in [section 2](#2-before-any-hardware-test) first.
Every run moves drives, except the bring-up test.

Run it in the shell of [section 2](#2-before-any-hardware-test):

```bash
ros2 run synapticon_bringup run_hil_tests.py --network-interface <interface>                   # one drive
ros2 run synapticon_bringup run_hil_tests.py --network-interface <interface> --robot 28dof     # 28 drives
```

To run every step on the EtherCAT ICube HAL, use `--hal ethercat_icube` without `--network-interface`, in the
terminal of [With the EtherCAT ICube HAL](#with-the-ethercat-icube-hal).

At the end, on one drive (about 7 minutes), expect this, each line followed by the path of its log:

```text
===== Summary
PASS  HIL suite: 53 passed, 4 skipped, skipped: DrivePair.ADisableVoltageOfOneLeavesTheOtherInQuickStop, ...
PASS  path test, CSP: -
PASS  path test, impedance: -
PASS  bring-up 1dof, CSP: -
PASS  bring-up 1dof, impedance: -
```

On the EtherCAT ICube HAL the suite has one test less, so 52 pass.

On a robot with several joints, expect:

```text
===== Summary
PASS  HIL suite, two-drive tests: 4 passed
PASS  bring-up 28dof, CSP: -
PASS  bring-up 28dof, impedance: -
PASS  multi-joint test 28dof: -
```

`--only` runs a subset, from `suite`, `path`, `bringup`, `pairs` (the two-drive tests), `joints` (the
multi-joint test) and `sweep` (the sweep of the multi-joint test alone, to watch every joint move).
`--two-drives` adds the two-drive tests to the full suite of a one joint run.
Each run logs to a directory under `~/.ros/log/hil`, which the summary names.
Ctrl-C or SIGTERM passes on to the running step, which stops as on its own Ctrl-C.
No later step runs, and the summary marks the step as interrupted.

### Your own bench

The examples expect their drives at ring position 0 and up, each on the example slave_config.
For any other bench, describe it in a robot definition file of your own and pass that file with `--robot`.
Every step then takes the drives from it, so a different bench never needs a code change.

Make the two files with the robot generator, outside the package:

```bash
python3 synapticon_bringup/scripts/generate_example_robot.py --joints 1 --robot my_bench --out-dir ~/bench
```

Then edit `~/bench/example_my_bench.yaml`:

- `position`: the drive's ring position, as `ethercat slaves` lists it
- `slave_config`: the drive's own slave_config, as an absolute path
- under `bus`: `master_id` (ICube HAL) or `network_interface` (SOEM HAL) of your bench.
  Every step takes the bus settings from here, only the suite always runs at 500 Hz.
- `max_joint_torque` under `command_defaults`: a torque ceiling in Nm that suits the drive and the test gains below.
  A joint with its own test gains needs one, or the impedance tests of the suite fail.
- a section `hil_tests` with the `stiffness` and `damping` the impedance tests move the joint with.
  Without it they use 15 Nm/rad and 0.5 Nm s/rad, which suit a geared joint.
  A motor without a gear has far less inertia, so these values make it chatter loudly.
  Start well below them, for example 2.5 Nm/rad and 0.03 Nm s/rad, and lower the `damping` under
  `command_defaults` too.
  When the joints need different values, give each its own under `joints`.
  A joint without an entry there takes the section's values:

  ```yaml
  hil_tests:
    stiffness: 15.0  # Nm/rad, every joint without an entry below
    damping: 0.5  # Nm s/rad
    joints:
      joint_2: {stiffness: 2.5, damping: 0.03}  # the gearless motor
  ```

The torque ceiling holds in impedance mode. In CSP the drive's own limit applies. For a low limit there, add the
drive's max torque in Nm at the motor to the `sdo_si:` list of its slave_config, for example
`- { index: 0x6072, sub_index: 0, type: uint16, si_value: 0.96 }`.

```bash
ros2 run synapticon_bringup run_hil_tests.py --network-interface <interface> --robot ~/bench/example_my_bench.yaml
```

With two joints in the file, the two-drive tests use them as the pair, each with its own slave_config.
So two drives of different models work too: the suite learns the units, the position increment and the brake
delay of each drive on its own.

## 8. Drive check

`drive_check` talks to one drive through a HAL, with no ros2_control in between.
Use it first on a new drive or a new slave_config.

Choose the HAL with `--hal`:

- `ethercat_soem` (the default), with `--network-interface`, in the shell of
  [section 2](#2-before-any-hardware-test)
- `ethercat_icube`, with `--master-id` (0 by default), in the terminal of [With the EtherCAT ICube HAL](#with-the-ethercat-icube-hal)
- `mock_cia402`, a simulated drive, which `colcon test` uses for every check

There are four checks:

| Check | Powers the drive? | What it does |
|---|---|---|
| `read` | no | prints the drive state twice a second |
| `timing` | no | measures how long `read()` and `write()` take |
| `hold` | yes | holds the position the joint had at the start |
| `move` | yes | swings the joint back and forth over `--travel-deg` at 1 rad/s |

`read` and `timing` set the joint parameter `energize` to `false`, so the HAL itself keeps the drive without power.
They also work on a drive whose units bring-up cannot read.
`hold` and `move` use CSP, or impedance mode with `--impedance`.

A check ends after `--duration` seconds (10 by default), on Ctrl-C or SIGTERM, or when the drive faults.
It then takes the power off and releases the bus.
A Ctrl-C during the bring-up ends `hold` and `move` before the drive gets power.
`move` also ends as soon as the joint gets more than 0.3 rad outside its travel.

### Steps

1. Read the drive without power, and turn the joint by hand meanwhile:

   ```bash
   IF=<interface>   # the network interface of the drive, for all steps
   ros2 run synapticon_hardware drive_check read --config "$CFG" --network-interface "$IF" --ring-position 0
   ```

   Three things show that the drive is fine.
   The log names the units it read from the drive ("one raw step is").
   The printed position, in rad, follows your hand.
   And the statusword says Switch On Disabled: in its low byte, bit 6 is set and bits 0 to 3 are clear.

   On the EtherCAT ICube HAL, use `--hal ethercat_icube --master-id 0` in place of `--network-interface "$IF"`, here
   and in the steps below.

2. Time the exchange, still without power:

   ```bash
   ros2 run synapticon_hardware drive_check timing --config "$CFG" --network-interface "$IF"
   ```

   For `read()` and for `write()` it prints the median, the times that 99 % and 99.9 % of the calls stay below, and
   the worst time.
   No call should fail, and hardly a cycle should come late.
   [Realtime.md](Realtime.md) shows how to prepare the host for that.

3. Now give the drive power, in CSP. Clear the space around the joint first.
   `hold` keeps the joint where it is: push it, and it pushes back.
   `move` then swings it 30 degrees and back.

   ```bash
   ros2 run synapticon_hardware drive_check hold --config "$CFG" --network-interface "$IF"
   ros2 run synapticon_hardware drive_check move --config "$CFG" --network-interface "$IF"
   ```

   A joint with power but no torque has a torque limit of 0 on the drive (`0x2054`).
   Give it a limit for the run with `--max-torque <Nm>`, or store one on the drive.

4. Impedance mode needs the torque sensor configured on the drive, and `--stiffness`, `--damping` and `--max-torque`.
   Hold with a soft spring, push the joint, and it gives way and comes back.
   For `move`, keep the travel small: the target is where the spring pulls to.

   ```bash
   ros2 run synapticon_hardware drive_check hold --impedance --stiffness 15 --damping 0.5 --max-torque 10 --config "$CFG" --network-interface "$IF"
   ros2 run synapticon_hardware drive_check move --impedance --stiffness 15 --damping 0.5 --max-torque 10 --travel-deg 10 --config "$CFG" --network-interface "$IF"
   ```

A fault stays on the drive until a `reset_fault`, or until the drive is power-cycled.

`--rt` asks for real-time scheduling, locked memory and no deep CPU idle states.
It runs at the real-time priority 60 of the example controller manager, `--rt-priority N` picks another one.
It needs root.
`drive_check --help` lists every option.

## 9. Manual tests

### Holding against a load

This test checks that a drive in Quick Stop Active really holds.
The suite can only infer that from positions.
Run it once in CSP and once in impedance mode.
Use a slave_config that sets `0x605A = 6`, as the example config does.

1. Terminal 1, in the shell of [section 2](#2-before-any-hardware-test), launch. For impedance mode, add
   `impedance:=true`:

   ```bash
   ros2 launch synapticon_bringup example_synapticon.launch.py robot:=1dof launch_rviz:=false \
     slave_config:=<your slave_config> network_interface:=<interface>
   ```

   On the EtherCAT ICube HAL, use `hal:=ethercat_icube` in place of `network_interface:=<interface>`, in the
   terminal of [With the EtherCAT ICube HAL](#with-the-ethercat-icube-hal).

2. Terminal 2, watch the drive state (`device_state_name`, once a second and on every change):

   ```bash
   ros2 topic echo /drive_controller/drive_status --field drives
   ```

3. Terminal 3, read the joint's position (`position:` in the output):

   ```bash
   ros2 topic echo /joint_states --once
   ```

4. Command that position, so that no old setpoint applies when the drive gets power. In CSP:

   ```bash
   ros2 topic pub --once /forward_position_controller/commands std_msgs/Float64MultiArray "{data: [<position>]}"
   ```

   In impedance mode, use a soft spring that a hand can push away, here 0.4 Nm/rad:

   ```bash
   ros2 topic pub --once /motor_command_controller/commands synapticon_control_msgs/msg/MotorCommand \
     "{name: [joint_1], position: [<position>], velocity: [0.0], torque_feedforward: [0.0], stiffness: [0.4], damping: [0.1]}"
   ```

   The spring pulls the joint back only when its torque beats the joint's friction.
   That torque is the stiffness times the deflection.
   To change the stiffness, command the current position again with the new value.

5. Enable. Terminal 2 shows Operation Enabled.

   ```bash
   ros2 service call /drive_controller/set_drive_action synapticon_control_msgs/srv/SetDriveAction "{action: enable, dof_names: []}"
   ```

6. Push the joint by hand and let go.
   - Pass in CSP: it does not give way.
   - Pass in impedance mode: it gives way like a spring and springs back to the commanded position.
     Push it far, by tens of degrees.
     With a soft spring, a small deflection pulls too weakly to beat the friction, and the joint stays where it is.
     On a motor without a gearbox, 0.4 Nm/rad feels limp: with 0.06 Nm of friction, it stops about 9 degrees short.
     Use the stiffness of the `hil_tests` section there, for example 2.5 Nm/rad.
   - When it is hard to judge by hand, push while watching
     `ros2 topic echo /dynamic_joint_states --field interface_values`.
     In impedance mode, `joint_torque_demand` follows the stiffness times the deflection.

7. Quick stop. Terminal 2 shows Quick Stop Active.

   ```bash
   ros2 service call /drive_controller/set_drive_action synapticon_control_msgs/srv/SetDriveAction \
     "{action: quick_stop, dof_names: []}"
   ```

8. Push the joint again, or hang a load on it, for a few seconds.
   - Pass in both modes: it behaves as in step 6, and returns to where it stopped.
   - The state stays Quick Stop Active.
     A limp joint points to the drive configuration, for example its torque limits, not to the state machine.

9. With the load still on, read the position again and command it as in steps 3 and 4. Then enable again:

   ```bash
   ros2 topic echo /joint_states --once
   ros2 topic pub --once /forward_position_controller/commands std_msgs/Float64MultiArray "{data: [<position>]}"
   ros2 service call /drive_controller/set_drive_action synapticon_control_msgs/srv/SetDriveAction "{action: enable, dof_names: []}"
   ```

   In impedance mode, command the position with the `MotorCommand` of step 4 instead.
   Pass: Operation Enabled, and the joint does not jump.

10. Take the load off, or support the joint, while the drive still holds it.
    A disable removes the power after the ramp, and a joint without a brake would drop the load.
    Then disable and stop the launch with Ctrl-C:

    ```bash
    ros2 service call /drive_controller/set_drive_action synapticon_control_msgs/srv/SetDriveAction "{action: disable, dof_names: []}"
    ```

On a robot with several drives, hold against a load on one joint, and on every joint whose drive model,
slave_config or mechanics differ.
The automated tests cover what several drives add: the two-drive tests of the suite
([section 3](#3-hardware-in-the-loop-suite), step 3), the bring-up of every drive
([section 5](#5-robot-bring-up-test)), and drive actions, motion and the fault reaction across joints
([section 6](#6-multi-joint-test)).

## 10. Troubleshooting

Problems show up at one of four moments. Find the moment first, then your symptom.

**Before the bus starts**

- A hardware test started by hand with `launch_test` brings up another robot, or its controllers fail.
  Another program runs on the same `ROS_DOMAIN_ID`, often a leftover `robot_state_publisher`.
  Find it with `ros2 node list` and stop it, or use another ID.
- pluginlib reports `Failed to find plugin synapticon_ethercat_icube/SynapticonDrive`.
  The shell lacks the workspace overlay: `source install/setup.bash` and launch again.
- `/dev/EtherCAT0` refuses access: you have no rights on the device.
  Use a udev rule or a group. A new group counts only after a new login.
- SOEM cannot open the network interface.
  The shell has no `CAP_NET_RAW`, or the name is wrong.
  Use the shell of [section 2](#2-before-any-hardware-test), and check the name with `ip link`.
- Bring-up refuses a joint that gets power, because its slave_config maps no velocity state.
  The HAL needs the measured velocity (`0x606C`) to tell when the joint stands still.
  Map it in the `tpdo` section, as the example slave_config does.

**While the bus comes up**

- The IgH master does not activate.
  Another program holds it, or the `ec_` kernel modules are not loaded.
  Stop that program, or check the modules with `lsmod | grep ec_`.
- SOEM finds no EtherCAT slave.
  If the interface is down, the message says so: bring it up, as the script of section 2 does.
  Otherwise the IgH master may still hold the interface (stop it), or the drive has no power or sits on another
  interface (check power and cable).
- The drive does not reach OP.
  It refuses an index or a type in the PDO mapping of its slave_config. On the IgH master, `dmesg` and
  `ethercat slaves` tell why.
  Generate the slave_config from the drive's ESI file.
- Settings reads (SDO) fail at random with `Input/output error`, on a drive with Ethernet over EtherCAT such as the
  Integro. NetworkManager sends traffic through the drive's `eoe` interface.
  See [NewDriveModel.md](NewDriveModel.md#ethernet-over-ethercat).
- Bring-up warns `No valid answer to the SDO read` and tries again, or refuses with `UNITS OF A DRIVE UNKNOWN`.
  After a rescan of the bus, the IgH master reads each drive's object dictionary, and meanwhile the drive answers no
  other read. A rescan can follow when a program releases the master.
  Bring-up tries each read up to `sdo_read_attempts` times (5), `sdo_retry_wait_s` apart (1 s).
  Once a drive used up its tries, its later reads get one try each. The other drives keep theirs.
  If it still gives up, raise them in the `bus` section of the robot definition.
- Bring-up refuses a joint that gets power, because its torque on the position shaft is unknown.
  The gear ratio (`0x6091`) cannot be read or is 0, or the encoders `0x2012:9` and `0x2010:12` do not name encoder 1
  or 2. Unlike the drive's units, these have no `units:` override in the slave_config. Fix them on the drive.
- Bring-up refuses an impedance joint because of its firmware.
  The drive runs a version older than 5.6.11. Update it, see [FirmwareUpdate.md](FirmwareUpdate.md).

**While the data starts to flow**

- `start()` fails, because no drive sends a statusword other than 0.
  On a large bus the drives need more time after OP. Raise `readiness_timeout_s` (20 s by default).
- `start()` fails, because a joint runs another mode (`0x6061`) than its `mode_of_operation`.
  The drive refused the mode, or the slave_config's default for `0x6060` does not fit the joint (impedance mode takes
  it from there). Fix the slave_config or the joint, and check that the slave_config maps `0x6061`.

**Once the drive has power**

- Bit 3 of the statusword is set: the drive is in Fault.
  `error_code` (`0x603F`) names the cause. Remove it, then reset the fault.
- The joint has power, but no torque.
  The drive's torque limit `0x2054` is 0. Pass `--max-torque <Nm>`, or store a limit on the drive.
- Impedance mode faults on enable, while CSP works.
  The torque sensor type `0x2150:1` is not set. Configure the torque sensor on the drive.

## Appendix: The suite in detail

### What the suite covers

- **Quick stop.**
  Option code 6 stays powered and holds.
  Option code 2 stops, then switches off.
  Any other code falls back to `disable`.
  The tests also check the ramp at `0x6085`, and that bring-up reads `0x605A` and `0x6085` back.
- **Holding.**
  In CSP the drive holds its position by itself.
  That stays true while the motion controller keeps streaming, and after a ramp past the target.
  In impedance mode the client holds the target where the quick stop began.
  With that hold switched off, the joint stops at the last target that was forwarded.
- **Setpoint forwarding.**
  A stopped or faulted drive gets no controller setpoints until it is enabled again.
  The first target the drive follows is the one streamed during the enable walk.
  The joint does not move before Operation Enabled.
- **Faults.**
  A fault is reset only when someone asks for it.
  `disable`, `disable_voltage` (also during Fault Reaction Active) and the lifecycle `disable()` leave the drive in
  Fault, with its error code.
  An enable is refused until a `reset_fault`.
- **Following error.**
  Three tests measure and print figures: the drive's own reaction to a position step, a reported following error
  that the client answers with a quick stop, and one that is left alone.
  The tests set the window as an angle (0.006 rad), converted into the drive's own position increments, so a jump
  trips it on any drive.
- **Disable.**
  `disable` and the lifecycle `disable()` cut the power only once the joint has stopped.
  `disable_voltage` cuts at once.
  A drive that controls a brake (`0x2004:4` not 0) first engages it and keeps control for its controller disable
  delay (`0x2004:5`, for example 100 ms). The tests allow that time.
  A monitored joint (`energize: false`) refuses every request that would give it power.
- **Tracking.**
  A small streamed move in CSP, with the measured velocity at the streamed one, so the drive applies its gear ratio
  to velocities as the HAL assumes.
  A small streamed velocity in CSV: followed, stopped by a zero target, and braked by a quick stop while targets keep
  coming.
  In impedance mode with both gains at zero, `effort` is the joint torque demand.
- **Lifecycle.**
  The readiness query reports connected, streaming and enabled at each stage.
  The link health reports the drive online and in OP.
- **Two drives.**
  An action on one drive leaves the other alone: bring-up, quick stop, `disable_voltage` and a fault.
- **Bring-up, EtherCAT HALs only.**
  None of these tests gives the drive power.
  - The exchange starts again after a stop.
  - An inverted joint reads the mirrored position.
  - A configured torque limit is streamed in place of the drive's own.
  - A drive with MaxJointTorque 0 comes up, with a warning.
  - An impedance joint whose torque sensor type is None is refused.
  - A joint at a ring position without a drive, or with a missing slave_config, fails the bring-up.
  - A startup SDO that the drive refuses fails the bring-up (SOEM HAL only).

### How the drive behaves on the bench

These observations explain why some tests retry, wait or allow for things.

#### Quick stop

- **The ramp starts from a varying speed.**
  A quick stop ramps down from the drive's velocity demand at the moment of the request.
  On a host without real-time scheduling, setpoint frames arrive unevenly.
  So the demand varies, and can even be zero, which stops the joint abruptly.
  Tests that need a real ramp try up to three times.
- **In impedance mode there is no ramp.**
  The drive reports Quick Stop Active, but keeps applying the impedance law to whatever position target it gets.
  `0x6085` plays no part.
  The client makes the stop by holding the target where the quick stop began.
  In CSP and CSV the drive stops and stays stopped by itself.
- **The ramp acts on the velocity demand, not on the real speed.**
  Take a following error that is only reported (`0x2017:1 = 0`, statusword bit 13 in CSP), and a client that quick
  stops on it.
  The joint ends up holding in Quick Stop Active.
  But a position step saturates the demand.
  From a 1 rad step, the travel after Quick Stop Active was 1.4 rad at 200 rad/s^2 and 1.1 rad at 1000 rad/s^2.
  That is two to ten times v^2 / 2a of the real speed.
  Setting the target back to the actual position one cycle before the quick stop cut the travel to 0.2 rad.
  But the reversal tripped the drive's phase fault (`0x5300`, "PhFault") 86 ms later, so the tests do not do it.

#### Following error

- **The drive's own reaction ramps at `0x6085`, then takes the power off** (`0x2017:1 = 1`: quick stop, then Fault).
  It comes about 7 ms after a position step.
  The drive stays powered in Fault Reaction Active for a time that scales with 1 / `0x6085`.
  In Fault the joint then coasts on friction alone, at about 36 rad/s^2 on the bench.

  | Step | `0x6085` as deceleration | Time in Fault Reaction Active | Then |
  |---|---|---|---|
  | 0.05 rad | 20 rad/s^2 | 250 ms | 0.67 rad of travel |
  | 0.05 rad | 200 rad/s^2 | 29 ms | stopped |
  | 0.05 rad | 1000 rad/s^2 | 10 ms | a 0.5 rad coast |
  | 1 rad | 200 rad/s^2 | | a 7 rad coast |

  The drive finishes its reaction even when the voltage is cut during it.
  So neither the stack nor the watchdog can make it shorter.
  The measuring test uses a 0.2 rad step, to stay within one output revolution.
- **After a Fault Reset of a following error,** the drive was seen to turn the motor about 0.6 rad on its own, while
  still reporting Fault.
  The fault test allows for it.
- **A following error window left armed faults any later step.**
  With the window of a fault test (0.006 rad, 1 ms) still armed, a 0.05 rad step 200 ms after a re-enable
  faulted the drive 7 ms later.
  Holding still did not fault it.
  That is why every bring-up writes the vendor defaults back.

#### Releasing the bus

- **Right after the bus is released, the drive's mailbox may not be ready.**
  Then either the first config SDO of the next bring-up gets lost, or every SDO read fails and bring-up refuses.
  The EtherCAT ICube HAL then goes on, with only "Failed to download config SDO" in the log.
  The SOEM HAL writes a lost SDO once more, and refuses the bring-up when that fails too.
  Every `sdo_si` entry without `verify: false` is read back after the write, written once more when the drive kept
  another value, and the bring-up is refused when the second write does not take either.
  The suite waits after every release and writes `0x605A` first.
  It repeats a bring-up that failed, or whose read-back shows that a value did not take.
- **A bring-up on the EtherCAT ICube HAL can wait forever,** with the slave in PREOP and its error flag set.
  It happens now and then right after a test released the bus, for example with the drive in a following error
  fault raised just after a reset and enable.
  The bus activation in the EtherCAT driver has no timeout.
  That is why step 1 of [section 3](#3-hardware-in-the-loop-suite) runs the suite under a kill timeout.
