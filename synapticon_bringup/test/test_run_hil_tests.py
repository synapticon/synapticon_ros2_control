# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""Tests of run_hil_tests.py: a step never outlives the runner, and a cut short run never passes."""

import importlib.util
import os
from pathlib import Path
import signal
import sys
import threading

import pytest

PACKAGE = Path(__file__).resolve().parents[1]

spec = importlib.util.spec_from_file_location(
    "runner", PACKAGE / "scripts" / "run_hil_tests.py"
)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

SLEEPER = [sys.executable, "-c", "import time; time.sleep(30)"]


@pytest.fixture(autouse=True)
def keep_signal_handlers():
    """Put back the handlers a test changed."""
    saved = {number: signal.getsignal(number) for number in runner.STOP_SIGNALS}
    yield
    for number, handler in saved.items():
        signal.signal(number, handler)


def test_a_timeout_stops_the_step_and_fails_it(tmp_path):
    before = {number: signal.getsignal(number) for number in runner.STOP_SIGNALS}
    step = runner.Step("sleeper", SLEEPER, timeout_s=0.5)
    code, lines, interrupted, timed_out = runner.run_step(step, tmp_path / "step.log")
    assert timed_out and not interrupted
    assert code != 0
    assert any(line.startswith("TIMEOUT") for line in lines)
    # The runner reacts to Ctrl-C again for the next step.
    assert {
        number: signal.getsignal(number) for number in runner.STOP_SIGNALS
    } == before


def test_a_stop_signal_stops_the_step(tmp_path):
    step = runner.Step("sleeper", SLEEPER, timeout_s=30.0)
    threading.Timer(0.5, os.kill, (os.getpid(), signal.SIGINT)).start()
    code, lines, interrupted, timed_out = runner.run_step(step, tmp_path / "step.log")
    assert interrupted and not timed_out
    assert code != 0
    assert any(line.startswith("INTERRUPTED") for line in lines)


def test_hangup_and_terminate_stop_the_runner_like_ctrl_c():
    runner.route_stop_signals()
    assert signal.getsignal(signal.SIGHUP) is signal.default_int_handler
    assert signal.getsignal(signal.SIGTERM) is signal.default_int_handler


@pytest.mark.parametrize(
    "passed, interrupted, code",
    [
        ([True, True], False, 0),
        ([True, False], False, 1),
        ([True], True, 1),
        ([], True, 1),
    ],
    ids=[
        "all passed",
        "one failed",
        "interrupted after passing steps",
        "interrupted before any step",
    ],
)
def test_the_exit_code(passed, interrupted, code):
    results = [(f"step {i}", ok, "", "") for i, ok in enumerate(passed)]
    assert runner.exit_code(results, interrupted) == code
