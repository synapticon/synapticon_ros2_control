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

## Hardware parameters

Two hardware parameters matter for this HAL in particular:

- `master_id` [0], the IgH master that runs the bus.
- `max_bad_cycles` [10], a whole number of at least 1: after this many cycles in a row without complete process data
  (no frame back, or a short working counter), `read()` fails until a complete cycle comes back.
  Fewer bad cycles are bridged with the last data received, and the link health shows them.
  Only a cycle whose frame was sent counts.
