# This Python file uses the following encoding: utf-8

# if__name__ == "__main__":
#     pass
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['preparing'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'message_filters', 'gps_common', 'sensor_msgs']
)

setup(**setup_args)
