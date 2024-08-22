from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup, find_packages

setup_args = generate_distutils_setup(
    name = 'picarx_ackermann_drive',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    test_suite='tests',
    install_requires=['rospy', 'std_msgs', 'message_generation', 'pyyaml', 'pytest', 'pytest-cov']
)

setup(**setup_args)
