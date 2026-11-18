# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""Tests of run_hil_tests.py: a step never outlives the runner."""

import importlib.util
from pathlib import Path
import signal

import pytest

PACKAGE = Path(__file__).resolve().parents[1]

spec = importlib.util.spec_from_file_location(
    "runner", PACKAGE / "scripts" / "run_hil_tests.py"
)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.fixture(autouse=True)
def keep_signal_handlers():
    """Put back the handlers a test changed."""
    saved = {number: signal.getsignal(number) for number in runner.STOP_SIGNALS}
    yield
    for number, handler in saved.items():
        signal.signal(number, handler)


def test_hangup_and_terminate_stop_the_runner_like_ctrl_c():
    runner.route_stop_signals()
    assert signal.getsignal(signal.SIGHUP) is signal.default_int_handler
    assert signal.getsignal(signal.SIGTERM) is signal.default_int_handler
