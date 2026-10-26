# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""Tests of generate_example_robot.py: the examples in config/ come from it, and its two files always match."""

import importlib.util
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]

spec = importlib.util.spec_from_file_location(
    "generator", PACKAGE / "scripts" / "generate_example_robot.py"
)
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


@pytest.mark.parametrize("dof", [1, 2, 28])
def test_the_shipped_examples_match_the_generator(dof):
    """config/example_<n>dof.yaml and its controllers file are exactly what the generator writes for n joints."""
    name, names = f"{dof}dof", generator.joint_names(dof, None)
    assert (
        PACKAGE / "config" / f"example_{name}.yaml"
    ).read_text() == generator.robot_definition(name, names)
    shipped = (PACKAGE / "config" / f"example_{name}_controllers.yaml").read_text()
    assert shipped == generator.controller_config(name, names)


def test_every_joint_list_follows_the_robot():
    """With many joints the lists wrap, and every controller still gets the robot's joints in order."""
    names = generator.joint_names(31, None)
    robot = yaml.safe_load(generator.robot_definition("many", names))
    assert [(joint["name"], joint["position"]) for joint in robot["joints"]] == list(
        zip(names, range(31))
    )
    for part in yaml.safe_load(generator.controller_config("many", names)).values():
        joints = part["ros__parameters"].get("joints")
        assert joints in (None, names)


@pytest.mark.parametrize(
    "command, code, files, message",
    [
        (
            ["--joints", "2", "--robot", "pan_tilt", "--names", "pan , tilt"],
            0,
            2,
            "robot:=pan_tilt",
        ),
        (["--joints", "0"], 2, 0, "at least 1"),
        (["--joints", "3", "--names", "x,y"], 2, 0, "2 name(s)"),
    ],
    ids=["written", "no joints", "too few names"],
)
def test_the_command_line(tmp_path, monkeypatch, capsys, command, code, files, message):
    monkeypatch.setattr(generator, "CONFIG_DIR", tmp_path)
    assert generator.main(command) == code
    assert len(list(tmp_path.iterdir())) == files
    assert message in capsys.readouterr()[0 if code == 0 else 1]


def test_existing_files_are_replaced_only_with_force(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(generator, "CONFIG_DIR", tmp_path)
    command = ["--joints", "2", "--names", "pan,tilt"]
    assert generator.main(command) == 0
    robot = yaml.safe_load((tmp_path / "example_2dof.yaml").read_text())
    assert [joint["name"] for joint in robot["joints"]] == ["pan", "tilt"]
    assert generator.main(command) == 1
    assert "--force" in capsys.readouterr().err
    assert generator.main(command + ["--force"]) == 0


def test_out_dir_writes_both_files_there(tmp_path, monkeypatch, capsys):
    """--out-dir writes the robot and its controllers into that folder and leaves the config folder alone."""
    config = tmp_path / "config"
    monkeypatch.setattr(generator, "CONFIG_DIR", config)
    bench = tmp_path / "bench"
    assert (
        generator.main(["--joints", "1", "--robot", "bench", "--out-dir", str(bench)])
        == 0
    )
    assert sorted(path.name for path in bench.iterdir()) == [
        "example_bench.yaml",
        "example_bench_controllers.yaml",
    ]
    assert not config.exists()
    assert "--robot" in capsys.readouterr().out
