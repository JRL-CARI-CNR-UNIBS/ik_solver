cmake_minimum_required(VERSION 3.16)
project(ik_solver_test)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE DebWithRelInfo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ik_solver REQUIRED)
find_package(rclcpp REQUIRED)

add_library(test_ik_solver_plugin SHARED
  src/test_ik_solver_plugin.cpp)

target_include_directories(test_ik_solver_plugin PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(test_ik_solver_plugin PUBLIC
  ik_solver::ik_solver)

ament_target_dependencies(test_ik_solver_plugin PUBLIC
  rclcpp
)

pluginlib_export_plugin_description_file(ik_solver test.plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS test_ik_solver_plugin
  EXPORT export_test_ik_solver_plugin
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include)

ament_export_targets(export_test_ik_solver_plugin)

ament_package()
