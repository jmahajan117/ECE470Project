# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;geometry_msgs;moveit_core;moveit_kinematics;moveit_ros_planning;pluginlib;tf_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lur3_kin;-lur5_kin;-lur10_kin;-lur3_moveit_plugin;-lur5_moveit_plugin;-lur10_moveit_plugin;/usr/lib/x86_64-linux-gnu/libboost_system.so".split(';') if "-lur3_kin;-lur5_kin;-lur10_kin;-lur3_moveit_plugin;-lur5_moveit_plugin;-lur10_moveit_plugin;/usr/lib/x86_64-linux-gnu/libboost_system.so" != "" else []
PROJECT_NAME = "ur_kinematics"
PROJECT_SPACE_DIR = "/home/ur3/catkin_jaym2/install"
PROJECT_VERSION = "1.2.5"