# cmake_minimum_required(VERSION 3.8)
# project(move_box_plugin)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
cmake_minimum_required(VERSION 3.5)
project(move_box_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(${gazebo_dev_INCLUDE_DIRS})

add_library(move_box_plugin SHARED src/move_box_plugin.cpp)
target_link_libraries(move_box_plugin ${gazebo_dev_LIBRARIES})

install(TARGETS move_box_plugin
  LIBRARY DESTINATION lib
)

ament_package()
