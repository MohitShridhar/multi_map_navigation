from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['multi_map_navigation'],
    package_dir={'multi_map_navigation': 'ros/src/multi_map_navigation'}
)

setup(**d)

