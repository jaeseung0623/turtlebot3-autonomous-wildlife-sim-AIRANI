cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(move_linear_plugin)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(move_linear_plugin SHARED move_linear_plugin.cpp)
target_link_libraries(move_linear_plugin ${GAZEBO_LIBRARIES})

