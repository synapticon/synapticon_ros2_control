# synapticon_hardware

The ros2_control hardware for Synapticon SOMANET drives, and everything the drive layers share.

The hardware interface `SynapticonSystemInterface` is what ros2_control loads, one for every mode.
It talks to the drives through a HAL, which the `<hardware>` parameter `hal` chooses:
`ethercat_soem` for real drives (the default, from `synapticon_ethercat_soem`), `mock` or `mock_cia402` without them.

What else is in the package:

| Part | What it does |
|---|---|
| `SynapticonHalI` | the HAL interface: lifecycle, cyclic read and write, drive actions |
| CiA 402 helpers | the drive state machine, the walks of the drive actions and the setpoint forwarding, for every HAL |
| `MockHal`, `MockCia402Hal` | HALs without drives: commands come back as states, or a simulated drive answers |
| EtherCAT helpers | what every EtherCAT HAL shares: `EthercatPdoHal`, object indices, PDO channels, the bus, joint and robot configuration, the checks at bring-up |
| `esi_to_slave_config.py` | makes a drive's slave_config from its ESI file |
| `config/example_somanet_circulo_safe_motion.yaml` | an example slave_config for one SOMANET Circulo |
| `drive_check` | basic checks of one drive through any HAL, see [Testing.md](../synapticon_ros2_control/doc/Testing.md) |
| test headers | the drive action suite that runs on any HAL, and the hardware tests the EtherCAT HALs share |

[Design.md](../synapticon_ros2_control/doc/Design.md) explains how the parts fit together.
The root [README](../README.md) explains setup and use.
