from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_multi_map_navigation'],
    package_dir={'mdr_multi_map_navigation': 'src/mdr_multi_map_navigation'}
)

setup(**d)

