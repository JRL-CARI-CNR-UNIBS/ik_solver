cmake_minimum_required(VERSION 3.16)
project(ik_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Only ROS 2 is supported. ROS_X is kept as a public definition for the plugins that still test it.
set(ROS_VERSION 2)

find_package(Eigen3 REQUIRED NO_MODULE)


###########
## SETUP ##
###########
set(IKSERVICES_SOURCE src/internal/services_common.cpp)

###############
    ## ROS 2 ##
###############
find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)

  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(pluginlib REQUIRED)

  find_package(geometry_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(ik_solver_msgs REQUIRED)

  find_package(ik_solver_core REQUIRED)

  list(APPEND IKSERVICES_SOURCE src/internal/services.ros2.cpp)
  set(NODE_SOURCE src/node.ros2.cpp include/ik_solver/internal/ik_solver_node.ros2.hpp)
###############
## END SETUP ##
###############

include(ProcessorCount)
ProcessorCount(N)

if(${N} GREATER 3)
  math(EXPR MAX_NUM_PARALLEL_IK_SOLVER ${N}-2)
else()
  SET(MAX_NUM_PARALLEL_IK_SOLVER 0)
endif()

#############
## TARGETS ##
#############
add_library(${PROJECT_NAME} SHARED ${IKSERVICES_SOURCE} src/internal/utils.cpp src/internal/types.cpp
  include/ik_solver/ik_solver.hpp
  include/ik_solver/ros_params.hpp
)
target_compile_definitions(${PROJECT_NAME} PUBLIC
  _MAX_NUM_PARALLEL_IK_SOLVER=${MAX_NUM_PARALLEL_IK_SOLVER}
  -DROS_X=${ROS_VERSION})

add_executable(${PROJECT_NAME}_node ${NODE_SOURCE})
target_compile_definitions(${PROJECT_NAME}_node PUBLIC -DROS_X=${ROS_VERSION})

###########
## LINK  ##
###########


###############
    ## ROS 2 ##
###############
if(ROS_VERSION EQUAL 2)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${rclcpp_TARGETS} ${tf2_TARGETS} ${tf2_ros_TARGETS} ${tf2_eigen_TARGETS} ${pluginlib_TARGETS} ${geometry_msgs_TARGETS} ${std_srvs_TARGETS} ${ik_solver_msgs_TARGETS})
  target_link_libraries(ik_solver PUBLIC
    Eigen3::Eigen
    ik_solver_core::ik_solver_core)
  add_library(ik_solver::ik_solver ALIAS ik_solver)
  target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  target_link_libraries(ik_solver_node PUBLIC
    ik_solver::ik_solver
    Eigen3::Eigen)
endif()

###########
## TESTS ##
###########
if(ROS_VERSION EQUAL 2)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gtest(test_node_ros2 test/test_node_2.cpp)
  target_include_directories(test_node_ros2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_node_ros2
    ik_solver::ik_solver)
  target_link_libraries(test_node_ros2 ${rclcpp_TARGETS} ${ros2_control_test_assets_TARGETS})
endif()

#############
## INSTALL ##
#############
if(ROS_VERSION EQUAL 2)
  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(rclcpp
                            tf2
                            tf2_ros
                            tf2_eigen
                            pluginlib
                            geometry_msgs
                            std_srvs
                            ik_solver_msgs
                            ik_solver_core
                            )

  install(TARGETS ${PROJECT_NAME}_node
      DESTINATION lib/${PROJECT_NAME})

  install(PROGRAMS
    scripts/get_tf_ik_ros2.py
    scripts/visualizer_script.py
    scripts/viusalizer_script.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

  ament_package()
endif()
