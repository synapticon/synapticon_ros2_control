# Notes for coding agents

Read [README.md](README.md) and [CONTRIBUTING.md](CONTRIBUTING.md) first.
This file lists pitfalls on real drives that are easy to miss and costly to hit.

## Hardware tests on any bench

- Every hardware test must run on any bench without code changes: other drive models, other ring positions,
  other slave_configs, another network interface or master.
  The bench is described in a robot definition file (`run_hil_tests.py --robot <file>`, `SYNAPTICON_HIL_ROBOT`),
  plus command line options and the `SYNAPTICON_HIL_*` variables of [Testing.md](synapticon_ros2_control/doc/Testing.md).
- Never hard-code a ring position, a slave_config, a drive's units or a gain in a test.
  Read it from the bench description or learn it from the drive, per drive.
- When adding a test or a bench setting, check that a colleague with a different setup can run it unchanged, and
  document new settings in Testing.md.

## Real drives

- Never energise or move a drive without the operator's explicit go for that run.
  Announce every hardware run, and stop before the first motion on a drive that has not moved yet.
- Do not assume a Circulo Safe Motion. On any other drive model, read
  [NewDriveModel.md](synapticon_ros2_control/doc/NewDriveModel.md) before the first run.
- The stack never commissions a drive. It reads the commissioned units at bring-up (`0x60A9`, `0x6092:1`, the
  position encoder and its resolution, `0x6076`, the gear ratio `0x6091` with the commutation encoder `0x2010:12`),
  understands them completely or refuses the joint (`drive_units.hpp`, `drive_checks.cpp`).
  Never assume a unit in code, and never decode only part of one.
- Values whose unit depends on the drive go into a slave_config's `sdo_si:` list in SI units, never as raw `sdo:`
  values. A raw value copied from another model can be off by a factor of 1000 (Circulo milli-rpm, ACTILINK-S rpm).
  Tests write SI values too and let bring-up convert them, per drive.
- A motor without a gearbox needs much lower impedance gains than a geared joint.
  Gains sized for a geared drive make it loud and shaky. Start low and raise them in steps.
- A drive with brake control (`0x2004:4` not 0) keeps power for its disable delay (`0x2004:5`) after a disable.
  Timing checks must allow for it.
- A firmware flash resets the torque sensor type `0x2150:1` to None, and impedance mode then refuses to run.
- Check STO with the e-stop pressed and released before the first move (`0x6621`, encoded per drive model).

## IgH EtherCAT master

- Random SDO `Input/output error` on an Integro based drive (for example ACTILINK-S) can mean that NetworkManager
  uses the drive's `eoe*` interface. Check `ip link`. The fix needs root, so hand it to the user, see
  [NewDriveModel.md](synapticon_ros2_control/doc/NewDriveModel.md#ethernet-over-ethercat).
- Never do a blocking SDO read on a started bus that nobody cycles.
  IgH processes the request only while the owning program sends cycles, so the read never returns.
  The process then sits in state `D` (`ps -o stat,wchan`: `ecrt_master_sdo_upload`), ignores `kill -9` and holds
  the master until the PC reboots. Read SDOs after `connect()` and before `start()`, or while the bus cycles.
- After a bus rescan, IgH 1.6.9 reads each drive's object dictionary and does not lock the mailbox meanwhile.
  Other SDO reads then fail with master error -5 (`Received upload response for wrong SDO` in `dmesg`).
  Releasing the master can trigger such a rescan. Bring-up retries these reads (`sdo_read_attempts`,
  `sdo_retry_wait_s`). Do not treat a single failure of this kind as a drive fault.
- Integro firmware up to at least 5.6.11 latches `0x7500` (`CyclicHb`) after every orderly OP to PRE-OP, which the
  SOEM HAL does at every stop. Known to Synapticon (#5818), fixed in a later firmware. Do not work around it in code,
  see [NewDriveModel.md](synapticon_ros2_control/doc/NewDriveModel.md#known-firmware-bug-cyclichb-with-the-soem-hal).
- Only one program can hold the master. Stop the stack before `ethercat` commands that change the drive.
