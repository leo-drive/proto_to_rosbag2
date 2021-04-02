cmake_minimum_required(VERSION 3.5)
project(proto_to_rosbag2)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Wpedantic -g3 -O3)

find_package(Protobuf REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set(EXECUTABLE_NAME ${PROJECT_NAME}_node)

add_executable(${EXECUTABLE_NAME}
  src/proto_to_rosbag2_node_main.cpp
  src/proto_to_rosbag2_node.cpp
  src/proto_rosbag2.pb.cc)

ament_target_dependencies(${EXECUTABLE_NAME} rclcpp rosbag2_cpp)

target_link_libraries(${EXECUTABLE_NAME}
  protobuf::libprotobuf)

target_include_directories(${EXECUTABLE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
