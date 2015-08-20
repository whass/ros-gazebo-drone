cmake_minimum_required(VERSION 2.8.3)
project(drone_controller)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    angles
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
    std_msgs 
    geometry_msgs
    genmsg
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    angles
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
    drone_msgs_srvs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/drone_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

add_executable(way_point_topic_pub src/way_point_topic_pub.cpp)
target_link_libraries(way_point_topic_pub ${catkin_LIBRARIES})




