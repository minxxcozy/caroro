cmake_minimum_required(VERSION 3.0.2)
project(caroro_code)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES caroro_wego
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Build C++ Executables
#############

add_executable(convert_lidar src/convert_lidar.cpp)
target_link_libraries(convert_lidar ${catkin_LIBRARIES})

#############
## Install Python Scripts
#############

catkin_install_python(PROGRAMS
  scripts/final.py
  scripts/obstacle_avoid.py
  scripts/rotary_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install Launch Files
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
