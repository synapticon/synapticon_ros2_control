# A quiet CPU core for the control loop

Every cycle, the control loop sends the drives a new setpoint.
If the loop is late, a drive gets its setpoint late or not at all, like a dancer whose partner misses the beat.

The fix is to give the loop a CPU core of its own and the right to run before anything else.
This page does that on a Linux PC, in the order the steps are done.
The ros2_control guide on [determinism](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#determinism)
has the background.

## Steps

1. **Pick a core.** Take one that nothing else needs: not core 0, which the operating system uses most, and not a
   core that other heavy processes on the machine are pinned to. The examples use core 1.

2. **Keep everything else off that core.** Add these arguments to the kernel command line and reboot:

   ```text
   isolcpus=1 nohz_full=1 rcu_nocbs=1 irqaffinity=0
   ```

   They keep away from core 1, in this order: the ordinary processes of the scheduler, the periodic timer tick while
   a single task runs there, a kind of kernel housekeeping (RCU callbacks), and the hardware interrupts, which go to
   core 0 instead.

   On most PCs they go into `GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub`, followed by `sudo update-grub`.
   Other boot loaders keep the kernel command line in their own configuration file.
   The kernel's [parameter list](https://docs.kernel.org/admin-guide/kernel-parameters.html) describes each argument.
   After the reboot, `cat /sys/devices/system/cpu/isolated` prints `1`.

3. **Let your user run real-time threads.** A normal user may not give a thread a real-time priority or lock its
   memory. Create `/etc/security/limits.d/realtime.conf` with:

   ```text
   @realtime  -  rtprio   99
   @realtime  -  memlock  unlimited
   ```

   Then run `sudo groupadd realtime` and `sudo usermod -aG realtime $USER`, and log in again.
   Without this, the controller manager warns at start-up and runs without both.

4. **Point the controller manager at the core.** The example controller configurations already set three parameters:
   `cpu_affinity: 1` puts the control thread on the core of step 1, `thread_priority: 60` gives it a real-time
   priority, and `lock_memory: true` keeps the process in RAM, so the loop never waits for a page from disk.
   For another core, change `cpu_affinity`.

5. **Check it.** While the stack runs, the control thread shows its core (`PSR`), a real-time class (`FF`) and its
   priority:

   ```bash
   ps -eLo tid,psr,cls,rtprio,comm | grep ros2_control
   ```

   To see how steady the loop is, run the `timing` check of `drive_check`, see [Testing.md](Testing.md).
   It shows how long reads and writes take and counts the cycles that came late.
