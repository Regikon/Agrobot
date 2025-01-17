cmake_minimum_required(VERSION 3.0.2)
project(robot_simulation)

find_package(gazebo REQUIRED)
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(robot_control_plugin SHARED robot_control_plugin.cc)
target_link_libraries(robot_control_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )