cmake_minimum_required(VERSION 3.8)
project(pedestrian_autoware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# General requirements
set(DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  derived_object_msgs
  mpc_planner_msgs
  tier4_simulation_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

## BUILD ###
include_directories(
)

add_executable(${PROJECT_NAME}_node src/node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})

## INSTALL ##
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

## FINALIZE PACKAGE ##
ament_export_dependencies(${DEPENDENCIES})
ament_package()
