"""Verify public best-effort Joy routing without starting a driver or a robot."""

import importlib.util
from pathlib import Path

import pytest


@pytest.mark.parametrize('start_joy', ['true', 'false'])
def test_driver_and_external_input_routes(start_joy, monkeypatch):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    from launch import LaunchContext

    spec = importlib.util.spec_from_file_location(
        'teleop_launch_test', Path(__file__).parents[1] / 'launch/teleop.launch.py',
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'load_robot_profile', lambda *args: ({}, True))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'controllersFile': '/unused/controllers.yaml',
        'mpcControllerName': 'controller', 'refBaseHeight': '',
        'startJoy': start_joy, 'joyTopic': '/test/joy',
        'deviceName': 'DualSense Wireless Controller',
        'paramsFile': '/unused/teleop.yaml',
        'commandTopic': '/test/command',
    })
    driver, adapter = module._nodes(context)
    assert driver['remappings'] == [('joy', '/test/joy/raw')]
    assert driver['condition'].evaluate(context) == (start_joy == 'true')
    parameters = adapter['parameters'][-1]
    assert parameters['joy_topic'] == '/test/joy'
    assert parameters['joy_source_topic'] == (
        '/test/joy/raw' if start_joy == 'true' else '/test/joy'
    )
