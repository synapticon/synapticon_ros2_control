# synapticon_ethercat_icube

The EtherCAT HAL for Synapticon SOMANET drives, through ICube's `ethercat_driver_ros2` and the IgH EtherCAT master.

| Part | Role |
|---|---|
| `EthercatIcubeHal` | the `SynapticonHalI` implementation, loaded by `SynapticonSystemInterface` as `hal` value `ethercat_icube` |
| `SynapticonDrive` | the EtherCAT slave plugin the driver loads for each drive |
| hardware tests | the HIL suite, see [doc/Testing.md](../synapticon_ros2_control/doc/Testing.md) |

The package needs the IgH master and the driver stack of `ethercat_icube.repos`.
It is built only with `SYNAPTICON_HAL_ETHERCAT_ICUBE=ON`, otherwise it builds nothing.

See the root [README](../README.md) for setup and use.
