"""The launch file builds a LaunchDescription with the documented arguments."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

LAUNCH_PATH = Path(__file__).resolve().parent.parent / 'launch' / 'display.launch.py'


def load_launch_description():
    spec = importlib.util.spec_from_file_location('display_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_description_builds():
    assert isinstance(load_launch_description(), LaunchDescription)


def test_launch_arguments():
    args = {e.name: e for e in load_launch_description().entities
            if isinstance(e, DeclareLaunchArgument)}
    assert args.keys() == {'gui', 'rviz', 'rviz_hidpi_fix'}
    assert args['gui'].choices == ['slider', 'jsp', 'none']
