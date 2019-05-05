from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['ll4ma_robot_control'],
    package_dir={'': 'src'},
)

setup(**setup_args)
