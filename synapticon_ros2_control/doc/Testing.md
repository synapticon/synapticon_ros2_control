# Testing

This guide shows how to test the HAL packages.
It starts with tests that need no hardware and ends with tests that move a real drive.

| Test | What you need | Does the motor move? | Section |
|---|---|---|---|
| Unit tests | nothing | no | [1](#1-unit-tests) |
| Hardware-in-the-loop suite | one drive, two for the two-drive tests | yes | [3](#3-hardware-in-the-loop-suite) |
| Drive check | one drive | only `hold` and `move` | [4](#4-drive-check) |

`colcon test` runs only section 1.
The hardware tests are built together with the unit tests, but `colcon test` never starts them.
They run only when you start them yourself.

Each section is a small test plan: what you need, the commands, what you should see, and how to leave the drive.
The appendix holds the details of the suite: what it checks, and how the drive behaves on the bench.

## 1. Unit tests

The unit tests need no drive and no bus.
They check the CiA 402 state machine, the drive actions, the HALs and the SOEM HAL's process image.

They also cover failures that a bench rarely shows:

- bring-up against a scripted drive: old firmware, no torque sensor, objects that cannot be read
- `Cia402Hal` with drives that fault, never come up, or sit on a failing bus
- `esi_to_slave_config.py`: for a small ESI file written inline in the test, it must write the bundled example
  slave_config

The drive action suite of [section 3](#3-hardware-in-the-loop-suite) runs here too, on simulated drives
(`mock_cia402`).
It takes about a minute.
So a change to the suite or to the drive logic shows up before you go to the bench.

1. Build and run the tests in your colcon workspace:

   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   colcon test --packages-select synapticon_hardware synapticon_ethercat_soem
   colcon test-result --verbose
   ```

2. Expect a summary with `0 errors, 0 failures`.
   When a test fails, `colcon test-result --verbose` lists it with its output.

## 2. Before any hardware test

All hardware tests run on the SOEM HAL, `ethercat_soem`.
SOEM is an EtherCAT master that runs as a normal program.
It needs a shell with the raw socket capability.

### What you need

- A SOMANET drive on firmware 5.6.11 or newer.
  For a joint in impedance mode, bring-up refuses older firmware, or firmware it cannot read.
  For the other modes it only warns.
- The drive powered and wired to a network interface of the host.
- No other EtherCAT master on that interface. If the IgH master is installed, the script of step 2 stops it.
- Room for the joint to turn about one output revolution both ways.
  Nothing attached that a sudden stop could damage.
  Keep a hand near the power switch or the e-stop.
- For the impedance tests: a torque sensor configured on the drive, so `0x2150:1` (sensor type) is not 0.
- The workspace built with tests (the default), in `Release`.

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
   Run the hardware tests in this shell, and `exit` it when you are done.
   If the script stopped the IgH master, it starts it again when the shell ends.

3. Check that the build is optimised.
   The packages fall back to `Release` when no build type is given.
   But an old build directory can still hold another type.

   ```bash
   grep CXX_FLAGS build/synapticon_hardware/CMakeFiles/synapticon_hardware.dir/flags.make
   grep CXX_FLAGS build/synapticon_ethercat_soem/CMakeFiles/synapticon_ethercat_soem.dir/flags.make
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

When a step fails, see [Troubleshooting](#5-troubleshooting).

## 3. Hardware-in-the-loop suite

`ethercat_soem_hil_test` runs `EthercatSoemHal` directly at 1 kHz, without ros2_control.
It holds two kinds of tests:

- the drive action suite, which works on any HAL and also runs on simulated drives in `colcon test`
- the EtherCAT tests, which bring the drive up with one change each and check what bring-up reports

Each test asks for the drive settings it needs: quick stop option code and deceleration, following error reaction,
mode.
The suite turns them into `sdo:` entries of a slave_config, built on a base config.

A watchdog protects the joint.
When the joint travels further than the test expects, it cuts the voltage and fails the test.

### Steps

1. Run the suite in strict mode, with a kill timeout on the binary itself.
   Use the shell of [section 2](#2-before-any-hardware-test).

   ```bash
   SYNAPTICON_HIL_STRICT=1 timeout -s KILL 900 \
     "$(ros2 pkg prefix synapticon_ethercat_soem)/lib/synapticon_ethercat_soem/ethercat_soem_hil_test"
   ```

   A run takes about five minutes.
   The usual gtest options work, for example `--gtest_filter=*QuickStop*`.
   The timeout ends a bring-up that never returns.
   Call the binary directly as above, because `ros2 run` does not pass the timeout's signal on.

2. With one drive, expect:

   ```text
   [  PASSED  ] 50 tests.
   [  SKIPPED ] 4 tests, listed below:
   [  SKIPPED ] DrivePair.ADisableVoltageOfOneLeavesTheOtherInQuickStop
   [  SKIPPED ] DrivePair.AFaultOnOneLeavesTheOtherRunning
   [  SKIPPED ] DrivePair.AQuickStopOfOneLeavesTheOtherEnabledAndMoving
   [  SKIPPED ] DrivePair.BothComeUpAndGoDownTogether
   ```

   A failed test is listed under `[  FAILED  ]`.
   When the watchdog trips, it prints `WATCHDOG: travelled more than ...` with the statusword of that moment.

3. With a second drive of the same model on the bus, also run the two-drive tests:

   ```bash
   timeout -s KILL 900 "$(ros2 pkg prefix synapticon_ethercat_soem)/lib/synapticon_ethercat_soem/ethercat_soem_hil_test" \
     --two-drives --gtest_filter=DrivePair.*
   ```

   Expect 4 passed.
   Here the watchdog cuts both drives when either one strays too far from its commanded position.
   With only one drive on the bus, `--two-drives` fails the bring-up at once, before any drive gets power.

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
| `SYNAPTICON_HIL_SLAVE_CONFIG` | the installed example slave_config | base slave_config |
| `SYNAPTICON_HIL_NETWORK_INTERFACE` | unset | network interface of the drives, required |
| `SYNAPTICON_HIL_ALIAS` | `0` | slave alias |
| `SYNAPTICON_HIL_POSITION` | `0` | slave ring position |
| `SYNAPTICON_HIL_POSITION_2` | `SYNAPTICON_HIL_POSITION` + 1 | ring position of the second drive, for `--two-drives` |
| `SYNAPTICON_HIL_STRICT` | unset | `1` turns a failed impedance bring-up into a failure instead of a skip |

### What the suite leaves on the drive

A drive keeps its SDO values until it is power-cycled or they are written again.
The suite writes these:

- the quick stop option code `0x605A` and the quick stop deceleration `0x6085`
- the following error objects `0x2017:1`, `0x6065` and `0x6066`

Every bring-up writes the vendor defaults of the following error objects, unless the test sets them itself.
So no test starts with the armed detection of an earlier fault test.

At the end, the suite writes back the `0x6085` it found.
`0x605A` keeps the value of the base config (6 for the example config).
Two bring-up tests change a value for one bring-up and write the old value back right after:
MaxJointTorque (`0x2054`) set to 0, and the torque sensor type (`0x2150:1`) set to None.

To get the drive back exactly as commissioned, power-cycle it after the run.

The drive does a few surprising things during these tests.
[The appendix](#how-the-drive-behaves-on-the-bench) lists them, with the numbers measured on the bench.

### Adding a test

The suite lives in `synapticon_hardware`, as headers for test executables:

- `testing/hal_bench.hpp` holds the fixtures `SingleDrive` and `DrivePair`.
- `testing/cia402_hal_tests.hpp` pulls in the tests, by topic, from `testing/suite/`.

A test asks for its drive settings in `start()`, for example `start({ .quick_stop_deceleration = 2.0 })`.
It uses the HAL only through `Cia402Hal`.

Each HAL brings a `HalBench`, which applies the settings in its own way.
A test that only makes sense for one HAL goes into that HAL's package, next to its bench.

The EtherCAT HALs share one bench.
`testing/ethercat_hal_tests.hpp` holds it, together with the EtherCAT tests and the `main` of the test executable.
Each HAL's test executable only names its HAL and sets it up on the bench's bus.

## 4. Drive check

`drive_check` talks to one drive through a HAL, with no ros2_control in between.
Use it first on a new drive or a new slave_config.

Choose the HAL with `--hal`:

- `ethercat_soem` (the default), with `--network-interface`, in the shell of
  [section 2](#2-before-any-hardware-test)
- `mock_cia402`, a simulated drive, which `colcon test` uses for every check

There are four checks:

| Check | Powers the drive? | What it does |
|---|---|---|
| `read` | no | prints the drive state twice a second |
| `timing` | no | measures how long `read()` and `write()` take |
| `hold` | yes | holds the position the joint had at the start |
| `move` | yes | swings the joint back and forth over `--travel-deg` at 1 rad/s |

`hold` and `move` use CSP, or impedance mode with `--impedance`.

A check ends after `--duration` seconds (10 by default), on Ctrl-C, or when the drive faults.
It then takes the power off and releases the bus.
`move` also ends as soon as the joint gets more than 0.3 rad outside its travel.

### Steps

1. Read the drive without power, and turn the joint by hand meanwhile:

   ```bash
   IF=<interface>   # the network interface of the drive, for all steps
   ros2 run synapticon_hardware drive_check read --config "$CFG" --network-interface "$IF" --ring-position 0
   ```

   Three things show that the drive is fine.
   The log names the units it read from the drive ("one raw step on the motor side").
   The printed position, in rad, follows your hand.
   And the statusword says Switch On Disabled: in its low byte, bit 6 is set and bits 0 to 3 are clear.

2. Time the exchange, still without power:

   ```bash
   ros2 run synapticon_hardware drive_check timing --config "$CFG" --network-interface "$IF"
   ```

   For `read()` and for `write()` it prints the median, the times that 99 % and 99.9 % of the calls stay below, and
   the worst time.
   No call should fail, and hardly a cycle should come late.

3. Now give the drive power, in CSP. Clear the space around the joint first.
   `hold` keeps the joint where it is: push it, and it pushes back.
   `move` then swings it 30 degrees and back.

   ```bash
   ros2 run synapticon_hardware drive_check hold --config "$CFG" --network-interface "$IF"
   ros2 run synapticon_hardware drive_check move --config "$CFG" --network-interface "$IF"
   ```

   A joint with power but no torque has a torque limit of 0 on the drive (`0x2054`).
   Give it a limit for the run with `--max-torque <Nm>`, or store one on the drive.

4. Impedance mode needs the torque sensor configured on the drive.
   Without `--stiffness` and `--damping`, the gains stored on the drive apply.
   Hold with a soft spring, push the joint, and it gives way and comes back.
   For `move`, keep the travel small: the target is where the spring pulls to.

   ```bash
   ros2 run synapticon_hardware drive_check hold --impedance --stiffness 20 --damping 0.5 --config "$CFG" --network-interface "$IF"
   ros2 run synapticon_hardware drive_check move --impedance --stiffness 20 --damping 0.5 --travel-deg 10 --config "$CFG" --network-interface "$IF"
   ```

A fault stays on the drive until a `reset_fault`, or until the drive is power-cycled.

`--rt` asks for real-time scheduling, locked memory and no deep CPU idle states.
It runs at the real-time priority 60 of the example controller manager, `--rt-priority N` picks another one.
It needs root.
`drive_check --help` lists every option.

## 5. Troubleshooting

Problems show up at one of four moments. Find the moment first, then your symptom.

**Before the bus starts**

- The network interface cannot be opened.
  The shell has no `CAP_NET_RAW`, or the name is wrong.
  Use the shell of [section 2](#2-before-any-hardware-test), and check the name with `ip link`.

**While the bus comes up**

- No EtherCAT slave answers.
  If the interface is down, the message says so: bring it up, as the script of section 2 does.
  Otherwise another master may still hold the interface (stop it), or the drive has no power or sits on another
  interface (check power and cable).
- The drive does not reach OP.
  It refuses an index or a type in the PDO mapping of its slave_config.
  Generate the slave_config from the drive's ESI file.
- Bring-up refuses an impedance joint because of its firmware.
  The drive runs a version older than 5.6.11. Update it.

**While the data starts to flow**

- `start()` fails, because no drive sends a statusword other than 0.
  On a large bus the drives need more time after OP. Raise `readiness_timeout_s` (20 s by default).

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
- **Disable.**
  `disable` and the lifecycle `disable()` cut the power only once the joint has stopped.
  `disable_voltage` cuts at once.
  A monitored joint (`energize: false`) refuses every request that would give it power.
- **Tracking.**
  A small streamed move in CSP.
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
  With the window of a fault test (1000 increments, 1 ms) still armed, a 0.05 rad step 200 ms after a re-enable
  faulted the drive 7 ms later.
  Holding still did not fault it.
  That is why every bring-up writes the vendor defaults back.

#### Releasing the bus

- **Right after the bus is released, the drive's mailbox may not be ready.**
  Then either the first config SDO of the next bring-up gets lost, or every SDO read fails and bring-up refuses.
  The SOEM HAL writes a lost SDO once more, and refuses the bring-up when that fails too.
  The suite waits after every release and writes `0x605A` first.
  It repeats a bring-up that failed, or whose read-back shows that a value did not take.
