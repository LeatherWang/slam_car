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
  "./src"
  "./src/serial_api"
  "./src/color"
  "./src/diff_drive_api"
  "./src/math_toolkit"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(./src/serial_api SRC_DIR_SERIAL_API)
AUX_SOURCE_DIRECTORY(./src/diff_drive_api SRC_DIR_DIFF_DRIVE_API)
AUX_SOURCE_DIRECTORY(./src/math_toolkit SRC_DIR_MATH_TOOLKIT)

## 添加库: serial_api
add_library(serial_api_shared SHARED
    ${SRC_DIR_SERIAL_API}
)
add_dependencies(serial_api_shared
    ${PROJECT_NAME}_generate_messages_cpp
)

## 添加库: math_toolkit
add_library(math_toolkit_shared SHARED
    ${SRC_DIR_MATH_TOOLKIT}
)
target_link_libraries(math_toolkit_shared
    ${catkin_LIBRARIES}
)

## 添加库: diff_drive_api
add_library(diff_drive_api_shared SHARED
    ${SRC_DIR_DIFF_DRIVE_API}
)
target_link_libraries(diff_drive_api_shared
    ${catkin_LIBRARIES}
    math_toolkit_shared
)


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
    ${catkin_LIBRARIES}
)

add_executable(odom_tf_pub ./src/odom_tf_pub.cpp )
target_link_libraries(odom_tf_pub
    ${catkin_LIBRARIES}
)

add_executable(sample_data ./src/sample_data.cpp )
target_link_libraries(sample_data
    ${catkin_LIBRARIES}
)

add_executable(diff_drive_controller ./src/diff_drive_controller.cpp )
target_link_libraries(diff_drive_controller
    ${catkin_LIBRARIES}
    diff_drive_api_shared
)
