cmake_minimum_required(VERSION 2.4.6)
project(dso_ros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cv_bridge
  tf
)

find_package(Dso REQUIRED)
find_package(Pangolin 0.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    cv_bridge
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
  "${SSE_FLAGS} -O2 -g -std=c++0x -fno-omit-frame-pointer"
)

set(ros_IOWrapper_source
  ${PROJECT_SOURCE_DIR}/include/IOWrapper/ROS/ROSOutputPublisher.cpp
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
add_executable(dso_node src/dso_node.cpp src/ros_output_wrapper.cpp)

target_link_libraries(dso_node
    ${DSO_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})
