CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
PROJECT(attitude_estimation)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_CXX_FLAGS "-Wall -g -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wpedantic")

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
catkin_package(
  DEPENDS Eigen3
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES attitude_estimation
)

INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})

ADD_EXECUTABLE(test_att_est test/test_att_est.cpp)
TARGET_LINK_LIBRARIES(test_att_est ${LINK_LIBS})

