cmake_minimum_required(VERSION 3.0.2)
project(ik_solver_core VERSION 0.1.0)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 20)

option(USE_ROS1 "Compile using ROS1 catkin and search for cnr_* libraries in catkin workspace" OFF)

find_package(urdfdom         REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS filesystem system date_time) # for cnr_logger missing dependency

find_package(PkgConfig REQUIRED) # for cnr_param (version of yaml-cpp?)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)

if(USE_ROS1)
  message(WARNING "cnr_param and cnr_logger should be installed outside ROS")
  find_package(catkin REQUIRED)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_utilities
#    CATKIN_DEPENDS
    DEPENDS urdfdom urdfdom_headers EIGEN3 Boost YAML_CPP cnr_param cnr_logger
  )
endif()
###########
## Build ##
###########

# Utilities
add_library(${PROJECT_NAME}_utilities SHARED
  src/${PROJECT_NAME}/internal/utils.cpp
  include/${PROJECT_NAME}/internal/SafeQueue.h
  include/${PROJECT_NAME}/internal/types.h
  include/${PROJECT_NAME}/internal/utils.h
  include/${PROJECT_NAME}/internal/wsq.h
  )

# Core
add_library(${PROJECT_NAME} INTERFACE
  include/${PROJECT_NAME}/ik_solver_base_class.h
  include/${PROJECT_NAME}/internal/ik_solver_base_class_impl.h
  )

if(USE_ROS1)
  add_dependencies(${PROJECT_NAME}_utilities
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${cnr_param_LIBRARIES})
  add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${cnr_param_LIBRARIES}
    )
endif()

#Utils
target_link_libraries(${PROJECT_NAME}_utilities PUBLIC 
  cnr_logger::cnr_logger
  cnr_param::cnr_param
  Eigen3::Eigen
  Boost::filesystem Boost::system Boost::date_time
  ${YAML_CPP_LIBRARIES}
)
# Core
target_link_libraries(${PROJECT_NAME} INTERFACE 
  ${PROJECT_NAME}_utilities
  ${urdfdom_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  Eigen3::Eigen
  Boost::filesystem Boost::system Boost::date_time
  ${YAML_CPP_LIBRARIES}
)

if(USE_ROS1)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
  )
else()
  target_include_directories(${PROJECT_NAME}_utilities PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PROJECT_NAME} INTERFACE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
endif()

#############
## Install ##
#############
if(NOT USE_ROS1)
  set(CONFIG_DEST share/cmake/${PROJECT_NAME})
  set(EXPORT_DEST ${CONFIG_DEST})
  set(CONFIG_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include) # for config
  set(config_filename "${PROJECT_NAME}Config")

  install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_utilities
          EXPORT export_${PROJECT_NAME}
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)

  install(EXPORT export_${PROJECT_NAME}
          FILE export_${PROJECT_NAME}.cmake
          NAMESPACE ik_solver_core::
          DESTINATION ${EXPORT_DEST})

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
          DESTINATION include)

  # Config
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    ${config_filename}.cmake.in ${config_filename}.cmake
    INSTALL_DESTINATION ${CONFIG_DEST}
  )
  write_basic_package_version_file(${config_filename}Version.cmake
      VERSION ${PACKAGE_VERSION}
      COMPATIBILITY AnyNewerVersion)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${config_filename}.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/${config_filename}Version.cmake"
          DESTINATION ${CONFIG_DEST}
  )


  # make uninstall
  add_custom_target("uninstall" COMMENT "Uninstall installed files")
  add_custom_command(
      TARGET "uninstall"
      POST_BUILD
      COMMENT "Uninstall files with install_manifest.txt"
      COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
              install_manifest.txt to be uninstalled!
  )

endif()
