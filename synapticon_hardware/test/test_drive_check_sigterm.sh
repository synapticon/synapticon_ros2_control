#!/bin/bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

# drive_check takes the power off on SIGTERM, as on Ctrl-C: a move check on a simulated drive ends cleanly.
# Usage: test_drive_check_sigterm.sh <path to drive_check>

set -u
log="$(mktemp)"
trap 'rm -f "$log"' EXIT

"$1" move --hal mock_cia402 --duration 0 >"$log" 2>&1 &
pid=$!
for _ in $(seq 100); do
  grep -q "Powering" "$log" && break
  sleep 0.1
done
sleep 1
kill -TERM "$pid"
wait "$pid"
status=$?
cat "$log"
[ "$status" -eq 0 ] && grep -q "^Disabled" "$log"
