# synapticon_ethercat_soem

The EtherCAT HAL for Synapticon SOMANET drives through the SOEM master library, in user space.
It needs no kernel module and no EtherCAT master installed on the machine.

| Part | Role |
|---|---|
| `EthercatSoemHal` | the `SynapticonHalI` implementation, loaded by its plugin name `ethercat_soem` |
| `net_raw_shell.sh` | starts a shell or a command with the capability the HAL needs, see below |
| hardware tests | the HIL suite, see [doc/Testing.md](../synapticon_ros2_control/doc/Testing.md) |

Each drive is described by its slave_config, which `esi_to_slave_config.py` of `synapticon_hardware` generates.
The `<hardware>` parameter (or the robot definition's `bus` entry) `network_interface` names the network interface
the drives are wired to.
To read the state of one drive through this HAL, with the capability of the next section:

```bash
ros2 run synapticon_hardware drive_check read --hal ethercat_soem --network-interface <interface>
```

Optional: `state_timeout_s` [5.0] bounds the wait for the drives to reach SAFE-OP and OP.

## The raw socket capability

SOEM sends and receives Ethernet frames through a raw socket.
Linux allows that only to root or to a process with the capability `CAP_NET_RAW`.
Without it, bring-up fails with a message that names the interface.
Do not run the ROS stack as root. Give it the capability as an ambient capability instead, which every program started
from it inherits:

- Interactively: `net_raw_shell.sh` starts a login shell with it, as your own user, in the current folder (it asks for
  your sudo password). It is installed with this package. From your sourced colcon workspace:

  ```bash
  "$(ros2 pkg prefix synapticon_ethercat_soem)/lib/synapticon_ethercat_soem/net_raw_shell.sh" --interface <interface>
  source /opt/ros/jazzy/setup.bash && source install/setup.bash   # sudo resets the environment
  ros2 run synapticon_hardware drive_check read --hal ethercat_soem --network-interface <interface>
  ```

  A command given after the options runs in place of the shell.
  The script needs `sudo` and `capsh`, which the Debian package `libcap2-bin` provides (rosdep installs it).
- As a service: `AmbientCapabilities=CAP_NET_RAW` in its systemd unit.

The network interface must be up (`ip link set <interface> up`).
With `--interface`, the script also prepares the interface for the time of the shell: it stops the IgH master if it
runs, brings the interface up, and starts the IgH master again when the shell ends.

A file capability (`setcap`) does not suit ROS 2 programs: it starts them in secure-execution mode, in which the dynamic
loader ignores `LD_LIBRARY_PATH`.

Only one EtherCAT master may use the network interface at a time.
If the IgH master also runs on the machine, stop it first, for example with `sudo systemctl stop ethercat`, or let the
script do it.

## License

MIT, like the rest of the repository.
It links the SOEM library of the package `soem_source`, which is under the GPL version 2 with a linking exception, so
the linked code keeps its own license.
See the root [README](../README.md).
