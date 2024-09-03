cmake_minimum_required(VERSION 3.8)
project(ik_solver_core VERSION 0.0.0)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 20)

option(USE_ROS1 "Compile using ROS1 catkin and search for cnr_* libraries in catkin workspace" OFF)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS filesystem system date_time) # for cnr_logger missing dependency

find_package(PkgConfig REQUIRED) # temporary for urdfdom
pkg_check_modules(urdfdom REQUIRED urdfdom IMPORTED_TARGET)

find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)

if(USE_ROS1)
  message(WARNING "cnr_param and cnr_logger should be installed outside ROS")
  find_package(catkin REQUIRED)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_utilities
#    CATKIN_DEPENDS
    DEPENDS urdfdom EIGEN3 Boost cnr_param cnr_logger
  )
endif()
###########
## Build ##
###########

# Utilities
add_library(ik_solver_core_utilities SHARED
  src/${PROJECT_NAME}/internal/utils.cpp
  include/${PROJECT_NAME}/internal/SafeQueue.h
  include/${PROJECT_NAME}/internal/types.h
  include/${PROJECT_NAME}/internal/utils.h
  include/${PROJECT_NAME}/internal/wsq.h
  )

# Core
add_library(ik_solver_core INTERFACE
  include/${PROJECT_NAME}/ik_solver_base_class.h
  include/${PROJECT_NAME}/internal/ik_solver_base_class_impl.h
  )

if(USE_ROS1)
  add_dependencies(ik_solver_core_utilities
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${cnr_param_LIBRARIES}
    ${cnr_logger_LIBRARIES})
  add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${cnr_param_LIBRARIES}
    ${cnr_logger_LIBRARIES})
endif()

# Utils
target_link_libraries(ik_solver_core_utilities PUBLIC
  cnr_logger::cnr_logger
  cnr_param::cnr_param
  Eigen3::Eigen
  Boost::filesystem Boost::system Boost::date_time
)
add_library(ik_solver_core::ik_solver_core_utilities ALIAS ik_solver_core_utilities)
# Core
target_link_libraries(ik_solver_core INTERFACE
  ik_solver_core::ik_solver_core_utilities
  cnr_logger::cnr_logger
  cnr_param::cnr_param
  PkgConfig::urdfdom
  Eigen3::Eigen
  Boost::filesystem Boost::system Boost::date_time
)
add_library(ik_solver_core::ik_solver_core ALIAS ik_solver_core)

if(USE_ROS1)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
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
  set(CONFIG_FILENAME "ik_solver_coreConfig")

  install(TARGETS ik_solver_core ik_solver_core_utilities
          EXPORT export_ik_solver_core
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)

  install(EXPORT export_ik_solver_core
          FILE export_ik_solver_core.cmake
          NAMESPACE ik_solver_core::
          DESTINATION ${EXPORT_DEST})

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
          DESTINATION include)

  # Config
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    ${CONFIG_FILENAME}.cmake.in ${CONFIG_FILENAME}.cmake
    INSTALL_DESTINATION ${CONFIG_DEST}
  )
  write_basic_package_version_file(${CONFIG_FILENAME}Version.cmake
      VERSION ${PACKAGE_VERSION}
      COMPATIBILITY AnyNewerVersion)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_FILENAME}.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_FILENAME}Version.cmake"
          DESTINATION ${CONFIG_DEST}
  )


  # make uninstall
  # add_custom_target("uninstall" COMMENT "Uninstall installed files")
  # add_custom_command(
  #     TARGET "uninstall"
  #     POST_BUILD
  #     COMMENT "Uninstall files with install_manifest.txt"
  #     COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
  #             install_manifest.txt to be uninstalled!
  # )

endif()
