cmake_minimum_required(VERSION 2.8.3)
project(drone_controller_gazebo)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    angles
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
    drone_controller
)

find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    gazebo_ros_control
    drone_controller
    imu_sensor_interface
)



add_library(${PROJECT_NAME} src/drone_controller_gazebo.cpp)
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
    
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  drone_controller_gazebo_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)  

