cmake_minimum_required(VERSION 3.8)
project(conveyor_autospawn)

# Find ament and your deps BEFORE using ament_* macros
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(conveyorbelt_msgs REQUIRED)

# Node
add_executable(conveyor_autospawn_node src/conveyor_autospawn_node.cpp)

# Header include paths (target-scoped)
target_include_directories(conveyor_autospawn_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against ROS 2 deps
ament_target_dependencies(conveyor_autospawn_node
  rclcpp
  ament_index_cpp
  gazebo_msgs
  conveyorbelt_msgs
)

target_link_libraries(conveyor_autospawn_node  tinyxml2)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(conveyor_autospawn_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Install binary and headers
install(TARGETS conveyor_autospawn_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
