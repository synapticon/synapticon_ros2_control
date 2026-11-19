# Diagnostics HowTo

A quick lookup for a running system: one question, one command, and what the answer should look like.
Work from the bottom up. First the bus, then the drives, then ros2_control, then the controllers.

The commands assume the example bring-up (`example_synapticon.launch.py`) with its default controller names.
Source your workspace in every terminal first:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## The bus

The `ethercat` command line tool comes with the IgH master and works with the `ethercat_icube` HAL only.
With `ethercat_soem` there is no such tool: the bring-up log names a drive that is missing or of another model, and
`/diagnostics` shows the bus health and the drives while the stack runs.

**Are the drives on the bus?**

With the SOEM HAL and the stack stopped, in a shell with the raw socket capability (see
[Testing.md](Testing.md#2-before-any-hardware-test)):

```bash
ros2 run synapticon_hardware drive_check read --config <slave_config> --network-interface <interface>
```

It brings the bus up, prints the drive state for 10 seconds and never energises the drive.
A drive that does not answer is a cabling or power problem, not a software one.

**Is the master up and are the drives on the bus?** (IgH master)

```bash
ethercat master
ethercat slaves
```

`ethercat master` shows `Link: UP` for the network card and the number of slaves it found.
`ethercat slaves` lists one line per drive, with its ring position, alias, state and name:

```text
0  0:0  OP  +  SOMANET Circulo CiA402 Drive safe motion
```

While the stack runs, every drive is in `OP`.
`PREOP` without the stack running is normal: nobody has taken the drives to `OP`.
A drive missing from the list is a cabling or power problem, not a software one.
If `ethercat` itself fails with a permission error on `/dev/EtherCAT0`, your user is not in the group that owns the device.
After adding it, log in again, or run `newgrp <group>` in the terminal.

**Is the bus healthy while running?**

```bash
ros2 topic echo /diagnostics --field status
```

The entry `ethercat_bus_monitor_controller: ethercat_bus` reads `EtherCAT link up, process data COMPLETE` when every
drive answers each cycle.
Anything else names the problem: `No EtherCAT link`, or process data that only some drives answer (a drive dropped out).
A drive outside the OP state raises a warning, and a change in the number of drives that answer is reported at once.

## The drives

**Which state is each drive in?**

```bash
ros2 topic echo /drive_controller/drive_status --once
```

One entry per joint. The fields to look at first:

| Field | Healthy value | What else means |
|---|---|---|
| `online` | `true` | `false`: the drive left the bus |
| `device_state_name` | `Operation Enabled` while enabled | `Fault`: see `error_code` and `error_report` |
| `mode_of_operation` | `-6` impedance, `8` CSP, `9` CSV | another value: the drive runs a mode the description did not ask for |
| `error_code` | `0` | the CiA 402 error code (0x603F) of the last fault |
| `latched` | `false` | `true`: the fault reaction holds this joint, see [Clear a latched fault reaction](#clear-a-latched-fault-reaction) |

The same states also appear on `/diagnostics`, one entry per joint, named `drive_controller: <joint>`.

Both topics carry every change of a drive's state as its own message, stamped with the control cycle it happened
in, and repeat every drive's state once a second.
The controllers send their changes every 20 ms.
Their parameter `diagnostics_check_period` sets this, in seconds.
On `/diagnostics`, `diagnostic_updater.period` sets the repeat.

**All of it at a glance?**

`diagnostic_aggregator` sorts the entries into a tree, and `rqt_robot_monitor` shows it.
Both are ROS packages of their own:

```bash
sudo apt install ros-jazzy-diagnostic-aggregator ros-jazzy-rqt-robot-monitor
```

Start the aggregator with the grouping of `synapticon_bringup`:

```bash
ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file \
  "$(ros2 pkg prefix synapticon_bringup)/share/synapticon_bringup/config/diagnostic_analyzers.yaml"
```

Then open the viewer:

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

The tree has `Synapticon/EtherCAT bus` and `Synapticon/Drives` with one entry per joint.
Each branch takes the worst state of its entries, so a single faulted drive turns `Drives` red.

**What does the drive itself report?** (IgH master)
These reads go straight to the drive over the mailbox.
On the IgH master they work while the hardware is active, or with the stack stopped.
After a deactivation the stack still holds the master but sends no frames, and a read then blocks.
Replace `-p0` with the ring position of the drive.

```bash
ethercat upload -p0 --type string 0x100a 0   # firmware version, for example v5.6.11
ethercat upload -p0 --type uint16 0x603f 0   # error code of the last fault, 0x0000 when none
ethercat upload -p0 --type int16 0x605a 0    # quick stop option code, 6 or 2 are supported
ethercat upload -p0 --type uint8 0x2150 1    # torque sensor type, 1 (Analog Input) for impedance
```

## ros2_control

**Are the controllers loaded and active?**

```bash
ros2 control list_controllers
```

Expected, in CSP:

```text
drive_controller                 synapticon_controllers/DriveController                  active
ethercat_bus_monitor_controller  synapticon_controllers/EthercatBusMonitorController     active
forward_position_controller      forward_command_controller/ForwardCommandController     active
joint_state_broadcaster          joint_state_broadcaster/JointStateBroadcaster           active
```

In impedance mode, `motor_command_controller` takes the place of `forward_position_controller`.
A controller stuck in `inactive` usually lost its interfaces to another controller, or the hardware is not active.

**Is the hardware active, and which interfaces does it export?**

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

The component `synapticon` should be `active`.
Each joint exports `position`, `velocity` and `effort` states, the drive states (`status_word`, `device_state`,
`error_code` and more), and the commands its mode needs.

**Does the joint move as commanded?**

```bash
ros2 topic echo /joint_states --field position
```

## Bring-up messages

Bring-up checks the configuration and the drives, and prints a framed block when something needs attention.
Its title line starts with `!!`.

| Block | Meaning | What to do |
|---|---|---|
| `FIRMWARE UPDATE NEEDED: SOMANET 5.6.11 OR NEWER` | A drive runs older firmware, or its version is unreadable | Impedance joints refuse to start. See [FirmwareUpdate.md](FirmwareUpdate.md) |
| `UNITS OF A DRIVE UNKNOWN` | A drive's velocity unit, position resolution, rated torque, gear ratio or feed constant could not be read or understood, or contradicts the `units:` section of its slave_config | Joints that may be energised refuse to start. The warnings above it name the value. See [Commissioning and units](NewDriveModel.md#commissioning-and-units) |
| `IMPEDANCE JOINTS WITHOUT A TORQUE SENSOR` | An impedance joint has no torque sensor type in 0x2150:1 | Commission the drive for impedance, see the README |
| `NO CONTROLLED QUICK STOP: 0x605A IS NOT 6 OR 2` | 0x605A is neither 6 nor 2, so `quick_stop` falls back to `disable` | Set 0x605A, see [QuickStop.md](QuickStop.md) |
| `INTERFACES WITHOUT DRIVE DATA` | The URDF declares interfaces this hardware does not know | Check the listed names for a typo |

The hardware logs as `SynapticonSystemInterface` and, depending on the HAL, `EthercatSoemHal` or `EthercatIcubeHal`
in the launch output.
The logs of past runs are in `~/.ros/log`.

## Drive actions

Every drive action goes through one service of `drive_controller`.
An empty `dof_names` means every configured drive.

### Reset a fault

```bash
ros2 service call /drive_controller/set_drive_action synapticon_control_msgs/srv/SetDriveAction \
  "{action: reset_fault, dof_names: []}"
```

Expected answer:

```text
response:
synapticon_control_msgs.srv.SetDriveAction_Response(success=True, message="Posted 'reset_fault' for all drives")
```

The faulted drives go to `Switch On Disabled`, which `drive_status` shows a moment later.
A reset only clears the fault. The drive stays de-energised until you enable it.
If the fault comes back at once, its cause is still there: read `error_code` first.

### Enable, stop and disable

```bash
ros2 service call /drive_controller/set_drive_action synapticon_control_msgs/srv/SetDriveAction \
  "{action: enable, dof_names: []}"
```

The same call takes the other actions:

| Action | Effect |
|---|---|
| `enable` | Walk to Operation Enabled and follow the setpoints |
| `quick_stop` | Brake and hold, see [QuickStop.md](QuickStop.md) |
| `disable` | Brake, then de-energise |
| `disable_voltage` | De-energise at once, without braking |
| `reset_fault` | Clear a fault, nothing else |

Before `enable`, make sure the command controller streams a target near the current position.
The drive follows the first target it gets.

### Clear a latched fault reaction

With the fault reaction on, a joint it acted on is latched and refuses `enable`.
Clear it, then reset and enable as above:

```bash
ros2 service call /drive_controller/clear_fault_reaction synapticon_control_msgs/srv/ClearFaultReaction \
  "{dof_names: []}"
```

[ControlLayers.md](ControlLayers.md) explains who stops the joint and why the latch exists.
