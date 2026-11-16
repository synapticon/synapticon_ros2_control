# synapticon_ros2_control

The metapackage of the ros2_control integration for Synapticon SOMANET drives.
It installs these packages:

| Package | Content |
|---|---|
| `synapticon_hardware` | The ros2_control hardware interface `SynapticonSystemInterface` and `torque_control_executable` |
| `synapticon_description` | The robot descriptions of the 1 and 2 DOF setups |
| `synapticon_bringup` | Launch files, controller and RViz configurations |
| `soem_source` | The vendored [SOEM](https://github.com/OpenEtherCATsociety/SOEM) EtherCAT master library |

Installation, the demo and the license are described in the [repository README](https://github.com/synapticon/synapticon_ros2_control).
