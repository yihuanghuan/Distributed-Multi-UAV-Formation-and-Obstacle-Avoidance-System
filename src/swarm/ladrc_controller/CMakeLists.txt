cmake_minimum_required(VERSION 3.8)
project(ladrc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# LADRC Library
add_library(ladrc_lib
  src/ladrc_core.cpp
  src/leso.cpp
  src/lsef.cpp
)

ament_target_dependencies(ladrc_lib
  rclcpp
)

# LADRC Controller Node
add_executable(ladrc_position_controller_node
  src/ladrc_position_controller_node.cpp
)

target_link_libraries(ladrc_position_controller_node
  ladrc_lib
)

ament_target_dependencies(ladrc_position_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  px4_msgs
)

target_link_libraries(ladrc_position_controller_node Eigen3::Eigen)

# Install
install(TARGETS
  ladrc_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ladrc_position_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()