#!/bin/bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

# drive_check refuses option values that are out of range or carry trailing text, with exit code 2.
# Usage: test_drive_check_options.sh <path to drive_check>

set -u
failed=0
refuse() {
  "$1" read --hal mock_cia402 "${@:2}" >/dev/null 2>&1
  local status=$?
  if [ "$status" -ne 2 ]; then
    echo "not refused with exit code 2 (got $status): ${*:2}"
    failed=1
  fi
}

refuse "$1" --ring-position 65536
refuse "$1" --ring-position -1
refuse "$1" --ring-position 1x
refuse "$1" --ring-position 0x1
refuse "$1" --duration 1s
refuse "$1" --duration -1
refuse "$1" --duration ""
refuse "$1" --travel-deg 30deg
refuse "$1" --travel-deg nan
refuse "$1" --max-torque -2
refuse "$1" --stiffness 5x
refuse "$1" --damping inf
refuse "$1" --rt-priority 100
refuse "$1" --rt-priority 50.5
"$1" read --hal mock_cia402 --ring-position 65535 --duration 0.2 --travel-deg -10 >/dev/null 2>&1 \
  || { echo "valid options refused"; failed=1; }
exit "$failed"
