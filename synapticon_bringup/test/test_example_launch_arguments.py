# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""The example launch refuses a robot definition that comes without its controller configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")


def test_robot_config_without_controllers_file_is_refused(capfd):
    example = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("synapticon_bringup"),
                "launch",
                "example_synapticon.launch.py",
            )
        ),
        launch_arguments={
            "robot_config": os.path.join(CONFIG, "geared_offset.yaml"),
            "hal": "mock",
            "launch_rviz": "false",
        }.items(),
    )
    service = LaunchService()
    service.include_launch_description(LaunchDescription([example]))
    assert service.run() != 0
    assert "robot_config is given without controllers_file" in "".join(
        capfd.readouterr()
    )
