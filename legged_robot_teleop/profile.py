"""Read robot reference bounds from the controller's existing YAML."""

import math
from pathlib import Path

import yaml


def load_robot_profile(path, controller_name, height_override=''):
    """Return height limits and lateral-drive support without duplicating profiles."""
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    reference = config[controller_name]['ros__parameters']['ocs2']['reference']
    height = float(height_override or reference['defaultBaseHeight'])
    minimum = float(reference['minBaseHeight'])
    maximum = float(reference['maxBaseHeight'])
    lateral = float(reference['maxDisplacementVelocityY'])
    if not all(math.isfinite(v) for v in (height, minimum, maximum, lateral)):
        raise ValueError('Robot reference parameters must be finite')
    if not 0 < minimum <= height <= maximum or lateral < 0:
        raise ValueError('Invalid robot height bounds or lateral velocity limit')
    return {
        'reference_height': height,
        'minimum_height': minimum,
        'maximum_height': maximum,
        'posture_controller': ('/' + controller_name if config[controller_name]['ros__parameters']
                               .get('bringUp', {}).get('standUpTrajectoryFile') else ''),
    }, lateral > 0
