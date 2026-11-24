"""Start the built C++ teleop executable for ROS-level tests on an isolated domain."""

import json
import os
import signal
import subprocess
import time

import pytest

DOMAIN_ID = 156


@pytest.fixture
def teleop_process():
    """Return a starter for the base_twist executable; every process is stopped afterwards."""
    executable = os.environ.get('TELEOP_EXECUTABLE')
    if not executable:
        pytest.skip('TELEOP_EXECUTABLE is set by colcon test (CMake)')
    processes = []

    def start(*ros_args):
        env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN_ID))
        process = subprocess.Popen(
            [executable, '--ros-args', *ros_args], env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        processes.append(process)
        return process

    yield start
    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


class Status:
    """Latest ~/status JSON of the teleop node."""

    def __init__(self):
        self.history = []

    def __call__(self, message):
        self.history.append(json.loads(message.data))

    @property
    def last(self):
        return self.history[-1] if self.history else {}

    @property
    def enabled(self):
        return bool(self.last.get('enabled'))

    @property
    def reason(self):
        return self.last.get('reason')


def wait_for(executor, condition, timeout=10.0, what='condition'):
    """Spin until condition() holds; fail with `what` otherwise."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return
        executor.spin_once(timeout_sec=0.01)
    raise AssertionError('Timed out waiting for ' + what)
