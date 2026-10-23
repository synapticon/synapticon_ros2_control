# synapticon_ros2_control

The metapackage of the ros2_control integration for Synapticon SOMANET drives.
It installs these packages:

| Package | Content |
|---|---|
| `synapticon_hardware` | `SynapticonSystemInterface` (the hardware interface), the HAL interface `SynapticonHalI`, the CiA 402 helpers, the code the EtherCAT HALs share, the slave_config generator, `MockHal` and `drive_check` |
| `synapticon_ethercat_soem` | `EthercatSoemHal`, the `SynapticonHalI` implementation on the SOEM EtherCAT master |
| `soem_source` | The vendored [SOEM](https://github.com/OpenEtherCATsociety/SOEM) EtherCAT master library |
| `synapticon_controllers` | `DriveController` (drive states and actions, fault reaction), `MotorCommandController` (impedance commands), `EthercatBusMonitorController` (bus health) |
| `synapticon_bringup` | The example launch file, robot definitions and controller configurations |
| `synapticon_description` | The example URDF and its xacro macros, with the motor and flywheel meshes |

The user documentation is in the folder `doc`: diagnostics, testing, quick stop, control layers, real-time setup and design.
Installation, the demo and the license are described in the [repository README](https://github.com/synapticon/synapticon_ros2_control).
