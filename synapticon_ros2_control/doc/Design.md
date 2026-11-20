# Design

How the packages fit together, and the rules that keep them that way.
Read it before you add a feature, a HAL or a controller.

## The picture

```text
 controllers                  move joints, watch drives, watch the bus
     |   ros2_control command and state interfaces
 SynapticonSystemInterface    one hardware interface for every mode and every way to reach the drives
     |   DriveState, DriveCommand
 SynapticonHalI               the HAL: talks to the drives
     |
 EthercatSoemHal              real drives on EtherCAT, through the SOEM library
 EthercatIcubeHal             real drives on EtherCAT, through ICube's ethercat_driver_ros2 and the IgH master
 MockHal, MockCia402Hal       no drives, or simulated ones
```

Think of the hardware interface as a switchboard.
It connects whatever the URDF declares to the right numbers of the right drive, but it never talks to a drive itself.
That is the job of the HAL (hardware abstraction layer) below it.
ros2_control sometimes calls its own hardware interface a HAL. In this repository the HAL is the layer below it.

The rules in short:

1. One hardware interface serves every mode. A mode is configuration, never a code branch.
2. Everything that depends on how the drives are reached lives in a HAL.
3. The HAL is made and connected at configure. Activation never powers a drive.
4. The control loop allocates nothing, looks nothing up by name, parses nothing and never waits.
5. Mistakes in the setup show up loudly at bring-up, not as a quiet warning later.
6. Nothing in the packages is specific to one robot.

## The layers

`SynapticonSystemInterface` knows nothing about modes or how the drives are reached.
The controller manager loads it through pluginlib, as the `hardware_interface::SystemInterface` named
`synapticon_ros2_control/SynapticonSystemInterface`.
It exports the interfaces that the URDF declares, and at configure `InterfaceBinding` links each one by name to a field
of `DriveState` or `DriveCommand`. Those go to the HAL.
CSP, CSV and impedance mode differ only in configuration: the declared command interfaces and the drive's mode of
operation.

An interface that the hardware does not know is still exported, because another part of the system may use it.
No drive data reaches it, though. All of them are listed in one warning at configure, so a typo stands out.

Everything that depends on the protocol or the library is inside a HAL: parameter parsing, the bus, SDO access, unit
scaling and the checks at bring-up.
At configure, the hardware interface loads the HAL through pluginlib, as a `synapticon_hardware::SynapticonHalI`, by
the name in the `<hardware>` parameter `hal`. It never names a HAL itself.
A new way to reach the drives, for example EtherCAT through another master library, is a new implementation of
`SynapticonHalI` under a new plugin name. Nothing above the HAL changes.

What every CiA 402 drive needs, whatever carries its data, is in the base class `Cia402Hal`: the walks to enable
and disable, the drive actions, the forwarding of setpoints, the quick stop hold in impedance mode and the ramped stop.
A HAL on top of it only moves the data. It exchanges the process data every cycle, reports each drive's statusword,
speed and position, and writes the controlword and the setpoints.

Each HAL with its own dependencies sits in its own package.
`EthercatSoemHal` is in `synapticon_ethercat_soem`, which links the SOEM library of `soem_source`.
`EthercatIcubeHal` is in `synapticon_ethercat_icube`, together with its drive plugin `SynapticonDrive`.
`SynapticonDrive` is a plugin only because ICube's EtherCAT driver loads its slaves by name: inside the HAL, the
driver's `EthercatBusManager` makes one per drive, as an `ethercat_interface::EcSlave`, from the plugin name, the bus
address, the slave_config and the buffer layout that the HAL hands it.
`synapticon_ethercat_icube` is built only with `SYNAPTICON_HAL_ETHERCAT_ICUBE=ON`. Without it, the other HALs remain.
What every EtherCAT HAL shares is in `synapticon_hardware`, without an EtherCAT dependency: the base class
`EthercatPdoHal` with the PDO buffers and the bring-up checks, the slave_config format, its loader and its generator,
the process image that converts a drive's PDO bytes, the robot definition and the hardware tests.

A HAL is named after how it reaches the drives, then the library, then `Hal`:

| How the drives are reached | HAL | `hal` |
|---|---|---|
| EtherCAT through the SOEM library | `EthercatSoemHal` | `ethercat_soem` |
| EtherCAT through ICube's `ethercat_driver_ros2` | `EthercatIcubeHal` | `ethercat_icube` |
| No drives | `MockHal` | `mock` |
| Simulated CiA 402 drives | `MockCia402Hal` | `mock_cia402` |

The rest of the code follows the naming of ros2_control: snake_case functions and methods, CamelCase classes, a
trailing underscore for members, UPPER_SNAKE constants and enum values.
A trailing `I` marks a pure interface, as in `SynapticonHalI`.

## Lifecycle and the control loop

The HAL is made and brought up at configure, never at activate. The bus then stays open through deactivate and
activate:

| Hardware interface | HAL calls | Afterwards |
|---|---|---|
| `on_init` | none | parameters read, drives found, buffers sized, transmissions loaded |
| `on_configure` | made, `setup()`, `connect()` | bus open, interfaces linked |
| `on_activate` | `start()`, one `read()` | data flows, every joint commanded to hold at its measured position |
| `on_deactivate` | `disable()`, `stop()` | drives off, no data |
| `on_cleanup` | `disconnect()`, released | bus closed |
| `on_error` | `disconnect()`, released | bus closed, and ros2_control leaves the hardware unconfigured |
| `on_shutdown` | as `on_cleanup` | bus closed, also when the hardware was only configured |

`on_deactivate` reports an error when `disable()` fails, after `stop()`.
A drive may then stay powered until its own watchdog takes it down, since no process data arrives any more.
A drive that a fault reaction left in Fault counts as off, so deactivating after a fault reaction succeeds.

The HAL also answers where it stands. `is_connected()` is true between `connect()` and `disconnect()`,
`is_streaming()` between `start()` and `stop()`, and `is_enabled()` between `enable()` and `disable()`, or until the
drives are off for another reason. `is_ready()` means all three.

Activation never powers a drive. Powering is always a separate request: the drive action `enable`, through the drive
controller's service `~/set_drive_action`.
The service refuses requests while the drive controller is inactive.
A request that was not carried out before a deactivation is dropped, so an activation never powers a drive by itself.

A joint whose position no controller gives holds.
While its drive has power, it holds the last valid position target.
While it has none, the hold follows the measured position, so that an enable never pulls the joint back.
When a controller releases a joint's command interfaces, they return to "not given": the joint holds, with velocity
and effort 0 and the default gains.
When the drive controller releases a joint's `drive_action`, the hardware disables that drive if it has power.
So a drive keeps power only while a drive controller is active.

The design aims at several dozen drives on one bus.
The control loop runs for all of them at once, 500 times a second in the examples, so all slow work happens before it starts.
The interface binding, the drive map, the transmissions and the PDO buffers are set up at init or configure.
Buffers that exported handles point into never change their size.
Slow drive accesses, such as SDO reads, run during bring-up, while the bus is configured but not yet exchanging data.
In the loop, `read()` and `write()` only copy numbers.

When something goes wrong, the calls say so:

- Lifecycle and setup calls return `bool` and log why they failed.
- `read()` and `write()` return `false` when a cycle's data did not arrive or could not go out. The hardware
  interface turns that into `return_type::ERROR` for ros2_control.
- Parsing a parameter may throw inside the HAL, which catches it in `setup()` and logs it. Nothing in the control
  loop throws.
- Problems the operator must fix first, like old firmware or a missing torque sensor, appear as one framed block at
  bring-up.

## Configuration and transmissions

The URDF describes the robot. The settings of the drives belong to the HAL.
The `<hardware>` of the example carries `hal`, `robot_config`, `network_interface`, `default_slave_config` and
`mode_of_operation`.
`SynapticonSystemInterface` reads `hal`, and from the robot definition named by `robot_config` the command defaults.
It passes everything on to the HAL, in a `HalConfig`.

`robot_config` names the robot definition YAML.
The EtherCAT HALs read its `bus` settings, and per joint, by name, the bus address, slave_config, limits, `invert` and
`energize`.
Every joint with a drive needs an entry there.
`default_slave_config` and `mode_of_operation` count for the joints whose entry does not set them, and a `<param>` in
the URDF itself wins over the file. `MockHal` ignores all of it.

A transmission is like a gearbox between motors and joints.
A joint that a transmission moves, without being one of its motors, has no drive of its own.
`DriveJointMap` sorts the joints into those with and without a drive. The HAL only sees the joints with one.
`TransmissionSet` loads the standard `transmission_interface` plugins and converts position, velocity and effort
between the two sides every cycle. Without transmissions, every joint has its own drive.

The packages serve any robot built from these drives.
Geometry, joint names, drive addresses, gains and limits belong to the robot's own configuration.
The examples are examples, not ready-made robot configurations.
Where a drive setting matters, the HAL reads it back and reports it, instead of writing a value that fits one drive.
