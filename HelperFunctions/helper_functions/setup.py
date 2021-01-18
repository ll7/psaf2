"""
Setup for helper_functions
"""
import setuptools
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['helper_functions', "helper_functions.*"],
 #   packages=setuptools.find_packages(include=["helper_functions", "helper_functions.*"]),
    package_dir={'': 'src'}
)

setup(**d)
