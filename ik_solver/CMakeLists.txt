cmake_minimum_required(VERSION 3.8)
project(ik_solver)

set(CMAKE_BUILD_TYPE Debug)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ROS_VERSION $ENV{ROS_VERSION})
if(ROS_VERSION STREQUAL "")
  message(FATAL_ERROR "No supported version of ROS found!")
endif()
if(NOT((ROS_VERSION EQUAL 1) OR (ROS_VERSION EQUAL 2)))
  message(FATAL_ERROR "No supported version of ROS found!")
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ik_solver_core REQUIRED)


###########
## SETUP ##
###########
set(IKSERVICES_SOURCE src/internal/services_common.cpp)
## ROS 1
if(ROS_VERSION EQUAL 1)
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
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_node
    CATKIN_DEPENDS eigen_conversions ik_solver_msgs pluginlib roscpp tf_conversions urdf geometry_msgs tf
    DEPENDS EIGEN3 ik_solver_core
  )

  include_directories(
    include
    ${ik_solver_core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  list(APPEND IKSERVICES_SOURCE src/internal/services_1.cpp)
  set(NODE_SOURCE src/node_1.cpp)
## ROS 2
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)

  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(pluginlib REQUIRED)

  find_package(geometry_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(ik_solver_msgs REQUIRED)

  list(APPEND IKSERVICES_SOURCE src/internal/services_2.cpp)
  set(NODE_SOURCE src/node_2.cpp)
endif()
########################

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
add_library(${PROJECT_NAME} SHARED ${IKSERVICES_SOURCE} src/internal/utils.cpp src/internal/types.cpp)
target_compile_definitions(${PROJECT_NAME} PUBLIC
  _MAX_NUM_PARALLEL_IK_SOLVER=${MAX_NUM_PARALLEL_IK_SOLVER}
  -DROS_X=${ROS_VERSION})

add_executable(${PROJECT_NAME}_node ${NODE_SOURCE})
target_compile_definitions(${PROJECT_NAME}_node PUBLIC -DROS_X=${ROS_VERSION})

###########
## LINK  ##
###########

## ROS 1
if(ROS_VERSION EQUAL 1)
  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}
    ${ik_solver_core_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${ik_solver_core_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endif()

## ROS 2
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
  target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    ${ik_solver_core_LIBRARIES})
  target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )
  add_dependencies(${PROJECT_NAME} ik_solver_core::ik_solver_core)

  target_link_libraries(${PROJECT_NAME}_node PUBLIC
    ${PROJECT_NAME}
    Eigen3::Eigen
    ${ik_solver_core_LIBRARIES})
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
                            )
  ament_export_include_directories(
    include
  )

  install(TARGETS ${PROJECT_NAME}_node
      DESTINATION lib/${PROJECT_NAME})

  ament_package()
endif()
