#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH
#
# Start a shell, or one command, as your normal user with the capability CAP_NET_RAW. Optionally prepare a network
# interface for the SOEM HAL first, and hand it back to the IgH master afterwards.
#
# What it is for: the SOEM HAL (synapticon_ethercat_soem) is an EtherCAT master in user space. It sends and receives
# raw Ethernet frames through a raw socket on the network interface, and Linux allows that only to root or to a
# process that holds CAP_NET_RAW. Without it, the SOEM HIL tests and a launch with hal:=ethercat_soem fail at bring-up
# with "Operation not permitted". The IgH master does not need this: its kernel module does the network traffic.
#
# How: sudo starts capsh, which switches back to your user (uid, gid, groups, HOME) and keeps CAP_NET_RAW as an
# ambient capability. Every program started from the shell inherits it: ros2 launch, ros2 run, the HIL test binaries,
# and the test runners.
# A file capability (setcap) on a binary is no alternative for ROS 2 programs: it puts the program into
# secure-execution mode, in which the dynamic loader ignores LD_LIBRARY_PATH, so the ROS libraries are not found.
#
# Usage (asks for your sudo password). The script is installed to lib/synapticon_ethercat_soem, so in a sourced
# workspace it is "$(ros2 pkg prefix synapticon_ethercat_soem)/lib/synapticon_ethercat_soem/net_raw_shell.sh":
#   net_raw_shell.sh [--interface <if>]                      an interactive login shell
#   net_raw_shell.sh [--interface <if>] <command> [args...]  one command through a login shell, for example the
#                                                            runner of the SOEM HIL tests
# It needs sudo and capsh (Debian package libcap2-bin).
# The shell or command starts in the current folder. Check inside: `grep CapAmb /proc/self/status` shows
# 0000000000002000 (bit 13, CAP_NET_RAW). sudo resets the environment: the login shell sets PATH and the rest again,
# and the script passes SSH_AUTH_SOCK and DISPLAY on. Source ROS and your workspace again inside.
#
# --interface <if> switches the interface to SOEM for the time of the shell:
#   - stops the IgH master (systemd service `ethercat`) if it runs, since two masters must never share an interface,
#   - brings the interface up, which the IgH generic driver leaves down when it stops,
#   - exports SYNAPTICON_HIL_NETWORK_INTERFACE=<if> for the hardware tests,
#   - starts the IgH master again when the shell or command ends, if this script stopped it.
# Without --interface, stop the IgH master and bring the interface up yourself.
#
# Exit the shell when done: everything started from it can read and send raw frames on every network interface.

set -euo pipefail

if [[ ${EUID} -eq 0 ]]; then
  echo "Run this as your normal user. It calls sudo itself and switches back to you." >&2
  exit 1
fi

interface=""
if [[ ${1:-} == --interface ]]; then
  if [[ -z ${2:-} ]]; then
    echo "--interface needs the name of a network interface." >&2
    exit 1
  fi
  interface=$2
  shift 2
fi

# The command to run in the login shell, quoted for it. Without arguments, an interactive login shell.
if (($# > 0)); then
  run="exec bash -lc $(printf '%q' "$(printf '%q ' "$@")")"
else
  run="exec bash -l"
fi

setup="cd $(printf '%q' "${PWD}")"
for name in SSH_AUTH_SOCK DISPLAY; do
  if [[ -n ${!name:-} ]]; then
    setup+=" && export ${name}=$(printf '%q' "${!name}")"
  fi
done

capsh_command=(sudo capsh --caps="cap_net_raw+eip cap_setpcap,cap_setuid,cap_setgid+ep" --keep=1
  --user="${USER}" --addamb=cap_net_raw --)

if [[ -z ${interface} ]]; then
  exec "${capsh_command[@]}" -c "${setup} && ${run}"
fi

if [[ ! -e /sys/class/net/${interface} ]]; then
  echo "There is no network interface '${interface}' (see ip link)." >&2
  exit 1
fi

igh_stopped=false
restart_igh() {
  if ${igh_stopped}; then
    echo "Starting the IgH master again: sudo systemctl start ethercat"
    sudo systemctl start ethercat
  fi
}
trap restart_igh EXIT
# A handler, not an ignore: the command still gets Ctrl-C, and this script lives on to restart the master.
trap ':' INT

if systemctl is-active --quiet ethercat; then
  echo "Stopping the IgH master, which would share '${interface}': sudo systemctl stop ethercat"
  sudo systemctl stop ethercat
  igh_stopped=true
  echo "It is started again when the shell ends. If that does not happen: sudo systemctl start ethercat"
fi
sudo ip link set "${interface}" up
setup+=" && export SYNAPTICON_HIL_NETWORK_INTERFACE=$(printf '%q' "${interface}")"

status=0
"${capsh_command[@]}" -c "${setup} && ${run}" || status=$?
exit "${status}"
