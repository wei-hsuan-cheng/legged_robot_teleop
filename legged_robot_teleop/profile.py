"""Read robot reference bounds from the controller's existing YAML."""

import math
from pathlib import Path

import yaml


def load_robot_profile(path, controller_name, height_override=''):
    """Return height limits, lateral drive and gait buttons without duplicating profiles."""
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
    parameters = config[controller_name]['ros__parameters']
    posture = bool(parameters.get('bringUp', {}).get('standUpTrajectoryFile'))
    # Gait buttons for the robots with a planned stand-up (Upkie, Pineapple): Circle cycles
    # the stepping gaits, Triangle holds the jump, each only if the controller defines it.
    templates = parameters.get('ocs2', {}).get('gait', {}).get('templates', {}).get('names', [])
    cycle = [name for name in ('walk', 'trot') if name in templates] if posture else []
    hold = 'jump' if posture and 'jump' in templates else ''
    # Trunk roll (D-pad left/right) for controllers that take it; 0 leaves the roll buttons off.
    max_roll = float(reference.get('maxBaseRoll', 0.0) or 0.0)
    if not math.isfinite(max_roll) or max_roll < 0:
        raise ValueError('maxBaseRoll must be a finite nonnegative angle')
    return {
        'reference_height': height,
        'maximum_roll': max_roll,
        'orientation_topic': parameters.get('target', {}).get(
            'baseOrientationTopic', '/humanoid/base_orientation_command'),
        'minimum_height': minimum,
        'maximum_height': maximum,
        'posture_controller': '/' + controller_name if posture else '',
        'gait_topic': (parameters.get('target', {}).get('gaitTopic', '/humanoid/gait_command')
                       if cycle or hold else ''),
        'gait_cycle': ','.join(cycle),
        'gait_hold': hold,
    }, lateral > 0
