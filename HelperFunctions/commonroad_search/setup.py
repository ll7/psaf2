"""
Setup for helper_functions
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['SMP'],
    package_dir={'': 'src/commonroad_search'}
)

setup(**d)
