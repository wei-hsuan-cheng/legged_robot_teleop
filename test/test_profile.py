"""Check robot profiles and override validation using temporary controller YAML."""

from legged_robot_teleop.profile import load_robot_profile
import pytest
import yaml


@pytest.fixture
def controllers(tmp_path):
    """Write a small controller reference fixture."""
    path = tmp_path / 'controllers.yaml'
    config = {'controller': {'ros__parameters': {'ocs2': {'reference': {
        'defaultBaseHeight': 0.22, 'minBaseHeight': 0.2, 'maxBaseHeight': 0.33,
        'maxDisplacementVelocityY': 0.0,
    }}}}}
    path.write_text(yaml.safe_dump(config), encoding='utf-8')
    return path


def test_profile_disables_sideways_drive(controllers):
    profile, lateral = load_robot_profile(controllers, 'controller')
    assert profile == {
        'reference_height': 0.22, 'minimum_height': 0.2, 'maximum_height': 0.33,
    }
    assert not lateral


def test_reference_height_override(controllers):
    profile, _ = load_robot_profile(controllers, 'controller', '0.3')
    assert profile['reference_height'] == 0.3


@pytest.mark.parametrize('height', ['0.19', '0.34', 'nan', 'inf'])
def test_invalid_override(controllers, height):
    with pytest.raises(ValueError):
        load_robot_profile(controllers, 'controller', height)


def test_missing_controller_fails_instead_of_using_wrong_robot(controllers):
    with pytest.raises(KeyError):
        load_robot_profile(controllers, 'missing_controller')
