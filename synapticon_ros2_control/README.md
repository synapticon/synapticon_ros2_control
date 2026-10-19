# synapticon_ros2_control

The metapackage of the ros2_control integration for Synapticon SOMANET drives.
It installs these packages:

| Package | Content |
|---|---|
| `synapticon_hardware` | The ros2_control hardware interface `SynapticonSystemInterface`, the drive interface `SynapticonHalI` with its mock and CiA 402 parts, and `drive_check` |
| `synapticon_ethercat_soem` | `EthercatSoemHal`, the `SynapticonHalI` implementation on the SOEM EtherCAT master |
| `synapticon_description` | The robot descriptions of the 1 and 2 DOF setups |
| `synapticon_bringup` | Launch files, controller and RViz configurations |
| `soem_source` | The vendored [SOEM](https://github.com/OpenEtherCATsociety/SOEM) EtherCAT master library |

Installation, the demo and the license are described in the [repository README](https://github.com/synapticon/synapticon_ros2_control).
