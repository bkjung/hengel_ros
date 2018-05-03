################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hengel_navigation)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  geometry_msgs
  std_msgs
)

################################################################################
# Python scripts and modules
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
    FILES
    ValveInput.msg
    OperationMode.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    )
################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime rospy geometry_msgs std_msgs
  )

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  scripts/main_program.py
  scripts/motor_input_calculate.py
  scripts/navigation_control.py
  scripts/odometry_publisher.py
  scripts/paint_drawing.py
  scripts/paint_letter.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################