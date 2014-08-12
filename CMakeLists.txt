cmake_minimum_required(VERSION 2.8.3)
project(ax2850_nh)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  serial
  serial_utils
  tf
  ax2550
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
#  DEPENDS
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp serial serial_utils tf ax2550
#  INCLUDE_DIRS include
   LIBRARIES  ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## ax2850 ROS Node
add_executable(ax2850_node src/ax2850_node.cc)
target_link_libraries(ax2850_node ${serial_LIBRARIES} ${serial_utils_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## ax2550 Omnidirectional Velocity ROS Node
add_executable(omni_cmd_vel_2 src/omni_cmd_vel_2.cpp)
target_link_libraries(omni_cmd_vel_2 ${catkin_LIBRARIES})

## ax2550 Omnidirectional Odometry ROS Node
add_executable(omni_odom_2 src/omni_odom_2.cpp)
target_link_libraries(omni_odom_2 ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ax2850_node omni_cmd_vel_2 omni_odom_2
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

