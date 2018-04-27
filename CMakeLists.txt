cmake_minimum_required(VERSION 2.8.3)
project(slam_car)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  cv_bridge
  message_filters
)

find_package(OpenCV 2.4 REQUIRED)

add_message_files(
   FILES
   pc_to_stm.msg
   stm_to_pc.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  "./src/serial_api"
  "./src/color"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(./src/serial_api SRC_DIR_SERIAL_API)

add_library(serial_api_shared SHARED
    ${SRC_DIR_SERIAL_API}
)
add_dependencies(serial_api_shared ${PROJECT_NAME}_generate_messages_cpp)

## 将用到的库放在一起使用
LIST(APPEND LINK_LIBS
    ${catkin_LIBRARIES}
    serial_api_shared
)

add_executable(serial_asio ./src/serial_asio.cpp )
target_link_libraries(serial_asio
    ${LINK_LIBS}
)

add_executable(base_controller ./src/base_controller.cpp )
target_link_libraries(base_controller
    ${LINK_LIBS}
)

add_executable(odom_tf_pub ./src/odom_tf_pub.cpp )
target_link_libraries(odom_tf_pub
    ${LINK_LIBS}
)

add_executable(sample_data ./src/sample_data.cpp )
target_link_libraries(sample_data
    ${LINK_LIBS}
)
