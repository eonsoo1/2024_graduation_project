cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  visualization_msgs
  lanelet2_core
  lanelet2_projection
  lanelet2_io
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PoseMsg.msg
)


# ## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES localization
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf visualization_msgs
 DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(IMU_node src/IMU.cpp)
add_dependencies(IMU_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(IMU_node ${catkin_LIBRARIES})


add_executable(GPS_node src/GPS.cpp)
add_dependencies(GPS_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GPS_node ${catkin_LIBRARIES})



add_executable(EKF_node src/ekf.cpp)
add_dependencies(EKF_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(EKF_node ${catkin_LIBRARIES})

add_executable(csv_reader_node src/map_maker.cpp)
add_dependencies(csv_reader_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(csv_reader_node ${catkin_LIBRARIES})

add_executable(csv_to_markers src/map_visual.cpp)
add_dependencies(csv_to_markers  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(csv_to_markers ${catkin_LIBRARIES})