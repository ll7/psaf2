"""
Setup for side_traffic_detector
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['side_traffic_detector'],
    package_dir={'': 'src'}
)

setup(**d)
