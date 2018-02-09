cmake_minimum_required(VERSION 2.8.3)
project(serial_asio)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_asio
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(src)

add_library(serial_api src/serial_api.cpp)

add_executable(serial_asio src/serial_asio.cpp )
target_link_libraries(serial_asio
    ${catkin_LIBRARIES}
    serial_api)



