cmake_minimum_required(VERSION 3.0.2)
project(ik_solver_core)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
	eigen_conversions
	tf_conversions
	# urdf
  ik_solver_msgs)

find_package(urdfdom         REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(Eigen3 REQUIRED COMPONENTS Geometry Core)
find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system date_time)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_utilities
  CATKIN_DEPENDS eigen_conversions tf_conversions urdf ik_solver_msgs
  DEPENDS EIGEN3 Boost cnr_param cnr_logger YAML_CPP urdfdom urdfdom_headers
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${cnr_param_INCLUDE_DIRS}
  ${cnr_logger_INCLUDE_DIRS}
  ${ik_solver_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_utilities SHARED
  src/${PROJECT_NAME}/internal/utils.cpp
 )
add_dependencies(${PROJECT_NAME}_utilities ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_utilities PUBLIC 
  ${catkin_LIBRARIES} 
  ${YAML_CPP_LIBRARIES} 
  ${cnr_logger_LIBRARIES} 
  ${cnr_param_LIBRARIES}
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/ik_solver_base_class.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC 
  ${urdfdom_LIBRARIES} 
  ${urdfdom_headers_LIBRARIES}
  ${catkin_LIBRARIES} 
  ${YAML_CPP_LIBRARIES} 
  ${PROJECT_NAME}_utilities  
  ${ik_solver_msgs_LIBRARIES}
)
