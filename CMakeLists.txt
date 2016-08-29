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

add_executable(drive_controller_node 
  src/drive_controller/main.cpp
  src/drive_controller/DifferentialDriveRegulator.cpp
  src/drive_controller/DriveController.cpp
  src/drive_controller/Odometry.cpp
  src/MD49/PIRegulator.cpp
)

target_link_libraries(drive_controller_node
  ${catkin_LIBRARIES}
)

add_executable(rpi_teleop_keyboard_node 
  src/rpi_teleop_keyboard/main.cpp
)

target_link_libraries(rpi_teleop_keyboard_node
  ${catkin_LIBRARIES}
)

add_executable(md49_node 
  src/MD49/EMG49.cpp
  src/MD49/LoopPeriodAnalyser.cpp
  src/MD49/main.cpp
  src/MD49/MD49.cpp
  src/MD49/MD49Message.cpp
  src/MD49/MD49MessageCreator.cpp
  src/MD49/PIRegulator.cpp
  src/MD49/Serial.cpp
  src/MD49/WheelController.cpp
  src/MD49/WheelSpeedRegulator.cpp
)

target_link_libraries(md49_node
  ${catkin_LIBRARIES}
)

add_executable(md49_test 
  src/MD49_test/main.cpp
)

target_link_libraries(md49_test
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

