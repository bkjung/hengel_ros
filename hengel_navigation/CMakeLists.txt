################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hengel_navigation)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

SET(PY_PROJECT_NAME hengel_path_manager)
SET(PY_PACKAGE_DIR src/hengel_path_manager)

# The first argument is the name of the project. This is the library name
# that you have specified in module.cpp (with lib prepended), and
# in your __init__.py file. In this example, ${PROJECT_NAME} is
# rocket_python, so the resulting library will be called librocket_python
#
# The second argument tells CMake where to find the __init__.py file so
# that it can be installed
add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  scripts/pathmaker_ui.py
)

################################################################################
# Python scripts and modules
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  scripts/start_program.py
  scripts/navigation_control.py
  scripts/printer_drawing.py
  scripts/printer_letter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
