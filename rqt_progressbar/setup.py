#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_progressbar'],
    package_dir={'': 'src/ros2_tiny_utils'},
    scripts=['scripts/rqt_progressbar']
)

setup(**d)
