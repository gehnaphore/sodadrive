cmake_minimum_required(VERSION 2.8.3)
project(rpi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS 
  system
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rpi
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################
# diff_drive_controller                        #
################################################

add_executable(diff_drive_controller_node 
  src/diff_drive_controller/main.cpp
  src/diff_drive_controller/MD49/EMG49.cpp
  src/diff_drive_controller/MD49/MD49.cpp
  src/diff_drive_controller/MD49/MD49Message.cpp
  src/diff_drive_controller/MD49/MD49MessageCreator.cpp
  src/diff_drive_controller/MD49/Serial.cpp
  src/diff_drive_controller/Regulator/LoopPeriodAnalyser.cpp
  src/diff_drive_controller/Regulator/Odometry.cpp
  src/diff_drive_controller/Regulator/PIDRegulator.cpp

)

target_link_libraries(diff_drive_controller_node
  ${catkin_LIBRARIES}
)

################################################
# drive_controller                             #
################################################

add_executable(drive_controller_node 
  src/drive_controller/main.cpp
  src/drive_controller/DifferentialDriveRegulator.cpp
  src/drive_controller/DriveController.cpp
  src/drive_controller/Odometry.cpp
  src/wheel_controller/PIDRegulator.cpp
)

target_link_libraries(drive_controller_node
  ${catkin_LIBRARIES}
)

################################################
# teleop_keyboard                              #
################################################

add_executable(teleop_keyboard_node 
  src/teleop_keyboard/main.cpp
)

target_link_libraries(teleop_keyboard_node
  ${catkin_LIBRARIES}
)

################################################
# wheel_controller                             #
################################################

add_executable(wheel_controller_node 
  src/wheel_controller/EMG49.cpp
  src/wheel_controller/LoopPeriodAnalyser.cpp
  src/wheel_controller/main.cpp
  src/wheel_controller/MD49.cpp
  src/wheel_controller/MD49Message.cpp
  src/wheel_controller/MD49MessageCreator.cpp
  src/wheel_controller/PIDRegulator.cpp
  src/wheel_controller/Serial.cpp
  src/wheel_controller/VelocityTrackingLoop.cpp
  src/wheel_controller/WheelController.cpp
  src/wheel_controller/WheelSpeedRegulator.cpp
)

target_link_libraries(wheel_controller_node
  ${catkin_LIBRARIES}
)

################################################
# wheel_controller_test                        #
################################################

add_executable(wheel_controller_test 
  src/wheel_controller_test/main.cpp
)

target_link_libraries(wheel_controller_test
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rpi rpi_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

