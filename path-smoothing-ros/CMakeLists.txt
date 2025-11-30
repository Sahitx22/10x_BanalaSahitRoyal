cmake_minimum_required(VERSION 2.8.3)
project(path_smoothing_ros)

## -------------------------------------------------------------
## Find Catkin Packages
## -------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  message_generation
)

## -------------------------------------------------------------
## Message Files
## -------------------------------------------------------------
add_message_files(
  FILES
  TrajectoryPoint.msg
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## -------------------------------------------------------------
## Catkin Package
## -------------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_smoothing_ros
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs message_runtime
)

## -------------------------------------------------------------
## Include Directories
## -------------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## -------------------------------------------------------------
## Library (Spline / Smoothing Code)
## -------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/cubic_spline_interpolator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## -------------------------------------------------------------
## Executables
## -------------------------------------------------------------
# DEMO
add_executable(demo src/demo.cpp)
target_link_libraries(demo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(demo
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# TRAJECTORY GENERATOR
add_executable(trajectory_generator src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(trajectory_follower src/trajectory_follower.cpp)
target_link_libraries(trajectory_follower ${catkin_LIBRARIES})

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
target_link_libraries(pure_pursuit_controller ${catkin_LIBRARIES})

add_dependencies(trajectory_generator
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

