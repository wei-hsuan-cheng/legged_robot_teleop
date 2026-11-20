"""Apply standard ROS Python lint checks to this package."""

from pathlib import Path

import pytest


def test_flake8():
    pytest.importorskip('ament_flake8')
    from ament_flake8.main import main_with_errors
    result, errors = main_with_errors(argv=[str(Path(__file__).parents[1])])
    assert result == 0, '\n'.join(errors)


def test_pep257():
    pytest.importorskip('ament_pep257')
    from ament_pep257.main import main
    assert main(argv=[str(Path(__file__).parents[1])]) == 0
