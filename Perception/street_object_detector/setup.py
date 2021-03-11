"""
Setup for street_object_detector
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['street_object_detector'],
    package_dir={'': 'src'}
)

setup(**d)
