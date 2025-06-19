cmake_minimum_required(VERSION 3.5)
project(xarm_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter 
    -Wno-unused-variable
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs" "moveit_msgs" "xarm_msgs" "moveit")

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(xarm_planner SHARED
  src/xarm_planner.cpp
)
ament_target_dependencies(xarm_planner 
  moveit_ros_planning_interface
  ${dependencies}
)

add_executable(xarm_planner_node src/xarm_planner_node.cpp)
ament_target_dependencies(xarm_planner_node
  ${dependencies}
)
target_link_libraries(xarm_planner_node 
  xarm_planner
  ${ament_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
  xarm_planner
  xarm_planner_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()