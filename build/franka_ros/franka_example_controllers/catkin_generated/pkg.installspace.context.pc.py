# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/zmc/libfranka/include".split(';') if "${prefix}/include;/home/zmc/libfranka/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;dynamic_reconfigure;eigen_conversions;franka_hw;geometry_msgs;hardware_interface;tf;tf_conversions;message_runtime;pluginlib;realtime_tools;roscpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_example_controllers;/home/zmc/libfranka/build/libfranka.so.0.8.0".split(';') if "-lfranka_example_controllers;/home/zmc/libfranka/build/libfranka.so.0.8.0" != "" else []
PROJECT_NAME = "franka_example_controllers"
PROJECT_SPACE_DIR = "/home/zmc/Desktop/master_arbeit/install"
PROJECT_VERSION = "0.7.0"
