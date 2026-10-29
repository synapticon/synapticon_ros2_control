# Setting up a new drive model

The examples and the test suite were developed on a SOMANET Circulo Safe Motion.
Other SOMANET drives run on the same stack, but they differ in ways that are easy to miss.

This page lists what to check before a new drive model moves for the first time.
The examples come from an ACTILINK-S 8-2: a motor with a built-in SOMANET Integro drive, no gearbox and a brake.
The steps of [Prepare a drive](../../README.md#3-prepare-a-drive) still apply on top.

## Wiring and STO

Check how the drive passes the bus on.
The ACTILINK-S with a single connector has one EtherCAT port and no output.
So it must be the last device on the chain.
Two of them need an EtherCAT junction, for example a Beckhoff CU1128, which gives each drive a branch of its own.

The ACTILINK-S has one hybrid cable for power, EtherCAT and STO (Safe Torque Off).
Its thin STO wires need the 24 V of your e-stop circuit, like the STO input of every other drive.
Without it the drive never powers the motor.

Check STO before the first move.
Read the drive's safety status word with the e-stop released, then pressed:

```bash
ethercat upload -p1 --type uint8 0x6621 1   # ACTILINK-S: 0 released, 1 pressed
```

Each drive model encodes this word in its own way.
The Circulo, for example, shows the change in sub-index 4 (`0x41` released, `0x40` pressed).
So look for a change between the two readings, not for a specific number.

## Ethernet over EtherCAT

Some drives, like the SOMANET Integro, also speak Ethernet over EtherCAT (EoE).
The IgH master then creates a network interface on the PC for each of them, named like `eoe0s1`.

On Ubuntu, NetworkManager takes this interface over and sends its own traffic through it.
That traffic blocks the drive's message channel.
Settings reads (SDO) then fail at random with `Input/output error`, and firmware transfers fail too.

Nothing in this stack needs these interfaces. Tell NetworkManager once to leave them alone:

```bash
printf '[keyfile]\nunmanaged-devices+=interface-name:eoe*\n' | sudo tee /etc/NetworkManager/conf.d/99-ethercat-eoe.conf
sudo nmcli general reload
```

Do this on every PC that runs such a drive with the IgH master. The SOEM master creates no such interface.
If a drive has already stopped answering, see
[FirmwareUpdate.md, Before you start](FirmwareUpdate.md#before-you-start) to bring it back.

## Firmware and slave_config

Every drive model has its own firmware package.
The ACTILINK-S is identified by the actuator, not by the drive inside it.
[FirmwareUpdate.md](FirmwareUpdate.md#1-identify-the-drive) shows how to find the right package.

A firmware update resets the torque sensor type `0x2150:1` to None.
Set it again before you use impedance mode, see
[Commission the drive for impedance](../../README.md#33-commission-the-drive-for-impedance).

Generate the slave_config from the ESI file of the firmware you flashed, with the drive's own product code.
The ACTILINK-S has product code `0x401`, the Circulo Safe Motion `0x302`.
A slave_config made for one model is refused by the other.

## Known firmware bug: CyclicHb with the SOEM HAL

Integro firmware up to at least 5.6.11 latches fault `0x7500` after every orderly change from OP to PRE-OP.
The drive's error report `0x203F:1` then reads `CyclicHb`.
Synapticon has fixed it in a later firmware (their issue #5818, "Integro devices on EtherCAT latching a CyclicHb
fault on every orderly drop from OP").

With the SOEM HAL this hits every stop: the next start finds the drive in Fault, and it refuses to power until a
`reset_fault`.
The ICube HAL is not affected in practice, because the drive leaves OP through its own watchdog there.
Until the firmware update, run an Integro based drive on the ICube HAL.

## Commissioning and units

Every new drive must be commissioned before it runs with the stack.
Commissioning sets, among other things, the units the drive counts in.
A commissioning tool such as OBLAC Drives keeps all values consistent with each other, so you rarely need the
details below. They help when something does not fit.

The stack never changes a drive's commissioning. At every bring-up it reads what it relies on, drive by drive:

| What | Read from | What the stack understands |
|---|---|---|
| Velocity unit | `0x60A9` | revolutions per minute or per second, times a power of ten |
| Feed constant | `0x6092:1` | disabled (`0xFFFFFFFF`, the default): positions in encoder counts |
| Position resolution | `0x2012:9`, then `0x2110:3` or `0x2112:3` | counts per turn of the position encoder |
| Rated torque | `0x6076` | torques are thousandths of it, at the motor |
| Gear ratio | `0x6091`, with `0x2012:9` and `0x2010:12` | motor turns per turn of the position shaft, which the drive applies unless the position encoder is the commutation encoder |

From these it converts every value into rad, rad/s and Nm, so drives with different units work side by side.
When a drive reports something the stack does not understand, the stack refuses to power that joint and says why.

The velocity unit shows why the commissioning must be consistent.
It sets the meaning of about a dozen values stored on the drive: the speed limit `0x6080`, the quick stop ramp
`0x6085`, the profile velocities and some thresholds of the control loops.
The Circulo Safe Motion counts in milli-rpm, the ACTILINK-S 8-2 in rpm.
So a quick stop deceleration of 5000 is a gentle ramp on one and 1000 times as hard on the other.
Change a unit only with a commissioning tool, which converts the values that depend on it too.

**Values in the slave_config.**
A value whose unit depends on the drive goes into the `sdo_si:` list, in SI units: velocities and positions on the shaft
of the position encoder, torques at the motor.
Bring-up reads the drive's units first and converts it, so the same file works on drives with different units:

```yaml
sdo_si:
  - { index: 0x6085, sub_index: 0, type: uint32, si_value: 200.0 }  # quick stop ramp, rad/s^2
```

The `sdo:` list holds only values without a unit, written as they are.
A raw value for any other object needs `raw: true`, so that it is a decision and not an accident.
Bring-up refuses a slave_config that breaks these rules, before it writes anything.

**When a drive cannot report its units.**
A `units:` section in the slave_config can state them:

```yaml
units:
  velocity_unit: 0x00B44700     # as in 0x60A9
  counts_per_revolution: 262144
  rated_torque: 3.2             # Nm
  feed_constant: disabled
```

Each entry only stands in for what the drive cannot report or the stack does not understand.
When the drive reports a value the stack understands, the two must agree, or bring-up stops.
The refusal message names the key to use.

## A motor without a gearbox

A motor without a gearbox has a very light rotor, and nothing slows it down.
Impedance gains that suit a geared joint make it buzz, shake and get loud.
On the ACTILINK-S, a stiffness of 15 Nm/rad with a damping of 0.5 Nm s/rad was already too much.
A stiffness of 2.5 Nm/rad with a damping of 0.03 Nm s/rad ran quietly.

Start with low gains, keep a hand on the e-stop, and raise them in small steps.
For a bench, also limit the drive in the slave_config:

```yaml
sdo_si:
  - { index: 0x6072, sub_index: 0, type: uint16, si_value: 0.96 }    # max torque, Nm
  - { index: 0x6080, sub_index: 0, type: uint32, si_value: 15.708 }  # max motor speed, rad/s (150 rpm)
```

## Brake

A drive that controls a brake keeps the motor powered a little longer after a disable.
That gives the brake time to close before the joint goes limp.
On the ACTILINK-S, the brake release strategy `0x2004:4` is 1, and the drive waits 100 ms (`0x2004:5`).
So a disable takes about 100 ms longer than on a Circulo without a brake. This is expected, not a fault.
