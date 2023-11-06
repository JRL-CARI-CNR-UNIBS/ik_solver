cmake_minimum_required(VERSION 3.0.2)
project(ik_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  ik_solver_msgs
  pluginlib
  roscpp
  tf_conversions
  urdf
)

find_package(Eigen3 REQUIRED COMPONENTS Geometry Core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS eigen_conversions ik_solver_msgs pluginlib roscpp tf_conversions urdf
  DEPENDS EIGEN3
)

message("${catkin_INCLUDE_DIRS}")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


include(ProcessorCount)
ProcessorCount(N)

add_library(${PROJECT_NAME} src/internal/services.cpp src/internal/utils.cpp src/internal/types.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE _MAX_NUM_PARALLEL_IK_SOLVER=${N})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/node.cpp src/internal/services.cpp src/internal/utils.cpp src/internal/types.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE _MAX_NUM_PARALLEL_IK_SOLVER=${N})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})
