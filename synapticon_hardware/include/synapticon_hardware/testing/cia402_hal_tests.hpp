// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025-2026 Synapticon GmbH
// Copyright (c) 2026 b-robotized GmbH

#pragma once

/// @file
/// @brief The drive action suite: the CiA 402 drive actions behind the fault reaction, on any Cia402Hal.
///
/// Include this header in exactly one source file of a test executable, and set the bench in its
/// main() (see hal_bench.hpp). On real drives the tests ENERGISE AND MOVE the motor, by up to about
/// one output revolution. A watchdog cuts the voltage if a joint travels further than expected.
/// The DrivePair tests need a second drive and are skipped without one.

#include "synapticon_hardware/testing/suite/quick_stop.hpp"
#include "synapticon_hardware/testing/suite/descents.hpp"
#include "synapticon_hardware/testing/suite/faults.hpp"
#include "synapticon_hardware/testing/suite/impedance.hpp"
#include "synapticon_hardware/testing/suite/tracking.hpp"
#include "synapticon_hardware/testing/suite/two_drives.hpp"
