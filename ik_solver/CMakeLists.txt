cmake_minimum_required(VERSION 3.16)
project(ik_solver)

set(CMAKE_BUILD_TYPE Debug)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ROS_VERSION $ENV{ROS_VERSION})
if(ROS_VERSION STREQUAL "")
  message(FATAL_ERROR "No supported version of ROS found!")
endif()
if(NOT((ROS_VERSION EQUAL 1) OR (ROS_VERSION EQUAL 2)))
  message(FATAL_ERROR "No supported version of ROS found!")
endif()

option(SEARCH_IN_CATKIN "search for _core and cnr_* dependencies in catkin (only if ROS_VERSION=1)" OFF)

if(ROS_VERSION EQUAL 1)
  message(ERROR "ROS 1 support is not stable! Use at your own risk!")
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)


###########
## SETUP ##
###########
set(IKSERVICES_SOURCE src/internal/services_common.cpp)

###############
    ## ROS 1 ##
###############
if(ROS_VERSION EQUAL 1)
  
  if(SEARCH_IN_CATKIN)
    set(LIB_IN_CATKIN cnr_param;cnr_logger;ik_solver_core)
  else()
    find_package(cnr_logger REQUIRED)
    find_package(ik_solver_core REQUIRED)
    find_package(cnr_param REQUIRED)
    find_package(cnr_yaml REQUIRED)
    set(LIB_NON_CATKIN cnr_yaml;cnr_param;cnr_logger;ik_solver_core)
  endif()

  find_package(catkin REQUIRED COMPONENTS
    eigen_conversions
    ik_solver_msgs
    pluginlib
    roscpp
    tf_conversions
    urdf
    std_srvs
    geometry_msgs
    tf
    ${LIB_IN_CATKIN}
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS eigen_conversions ik_solver_msgs pluginlib roscpp tf_conversions urdf geometry_msgs tf ${LIB_IN_CATKIN}
    DEPENDS EIGEN3 ${LIB_NON_CATKIN}
  )

  include_directories(
    include
    ${ik_solver_core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  list(APPEND IKSERVICES_SOURCE src/internal/services.ros1.cpp)
  set(NODE_SOURCE src/node.ros1.cpp)
###############
    ## ROS 2 ##
###############
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)

  find_package(cnr_yaml REQUIRED)
  find_package(cnr_param REQUIRED)
  find_package(cnr_logger REQUIRED)

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
endif()
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
    ## ROS 1 ##
###############
if(ROS_VERSION EQUAL 1)
  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}
    ${cnr_param_LIBRARIES}
    ${ik_solver_core_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_node
    ${cnr_param_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${ik_solver_core_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  catkin_install_python(PROGRAMS launch/ik_solver_1.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

###############
    ## ROS 2 ##
###############
if(ROS_VERSION EQUAL 2)
  ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    tf2
    tf2_ros
    tf2_eigen
    pluginlib
    geometry_msgs
    std_srvs
    ik_solver_msgs
  )
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
    ik_solver::ik_solver
    cnr_param::cnr_param)
  ament_target_dependencies(test_node_ros2
    rclcpp
    ros2_control_test_assets
  )
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

  install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

  ament_package()
endif()
