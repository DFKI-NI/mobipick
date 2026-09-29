from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(packages=['mobipick_servo'], package_dir={'': 'src'}))
